from communication.arinc429 import ARINC429
from communication.afdx import AFDXNetwork
from constantes import *
import threading
import math
from collections import deque

class AvionicsCalculator:
    """
    Implémente la machine à états et tous les algorithmes de calcul.

    CORRECTIONS APPLIQUÉES :
      [FIX-1] Thread-safety : threading.Lock() sur toutes les méthodes
              partagées entre le thread UI et le thread de simulation.
      [FIX-3] Agrégateur décode les trames AFDX (ARINC 429) au lieu
              d'accéder directement aux attributs du calculateur.
      [FIX-4] Chute libre : drapeau _stalling + gestion dédiée dans tick().
    """

    def __init__(self, afdx: AFDXNetwork):
        self.afdx = afdx

        # [FIX-1] Verrou partagé entre thread UI et thread simulation
        self._lock = threading.Lock()

        # État interne
        self.state           = STATE_AU_SOL
        self.altitude_ft     = 0.0
        self.climb_m_min     = 0.0
        self.attack_deg      = 0.0
        self.motor_power_pct = 0.0
        self.speed_m_s       = 0.0   # vitesse sol GS — composante horizontale (m/s)

        # Consignes
        self.desired_alt_ft  = 0.0
        self.input_attack    = 0.0   # consigne angle attaque (0 = auto)

        # [FIX-4] Drapeau de chute libre
        self._stalling       = False

        # Messages
        self.error_msg       = ""
        self.warnings        = []

        # Historique pour affichage
        self.alt_history     = deque(maxlen=120)

    # ── Validation des entrées (stateless, pas de lock nécessaire) ─────
    def validate_altitude(self, value: str) -> tuple[bool, float, str]:
        if value.strip() == "":
            # Champ vide → garder l'altitude désirée courante (pour ne pas
            # forcer la re-saisie quand on veut juste changer l'angle ou le taux)
            return True, self.desired_alt_ft, ""
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Altitude : valeur numérique requise"
        if not (ALT_MIN_FT <= v <= ALT_MAX_FT):
            return False, 0.0, f"Altitude hors limites [0, {ALT_MAX_FT}] ft"
        return True, v, ""

    def validate_climb(self, value: str) -> tuple[bool, float, str]:
        if value.strip() in ("", "0"):
            return True, 0.0, ""
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Taux de montée : valeur numérique requise"
        if not (-CLIMB_MAX_M_MIN <= v <= CLIMB_MAX_M_MIN):
            return False, 0.0, f"Taux de montée hors limites [±{CLIMB_MAX_M_MIN}] m/min"
        return True, v, ""

    def validate_attack(self, value: str) -> tuple[bool, float, str]:
        if value.strip() == "":
            return True, 0.0, ""   # champ vide → auto
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Angle d'attaque : valeur numérique requise"
        if not (ATTACK_MIN <= v <= ATTACK_MAX):
            return False, 0.0, f"Angle d'attaque hors limites [±{ATTACK_MAX}]°"
        return True, v, ""

    # ── Reset sécurisé (bouton ATTERRIR) ──────────────────────────────
    def reset(self):
        """[FIX-1][FIX-4] Remet le système à zéro de manière thread-safe.
        Envoie une trame AFDX immédiate pour que l'agrégateur se synchronise
        sans attendre le prochain tick (évite altitude/erreur bloquées)."""
        with self._lock:
            self.state           = STATE_AU_SOL
            self.altitude_ft     = 0.0
            self.desired_alt_ft  = 0.0
            self.climb_m_min     = 0.0
            self.attack_deg      = 0.0
            self.motor_power_pct = 0.0
            self.speed_m_s       = 0.0
            self.input_attack    = 0.0
            self.error_msg       = ""
            self.warnings        = []
            self._stalling       = False
            # Trame immédiate — l'agrégateur voit altitude=0 et error=""
            # dès le prochain update(), sans attendre le tick suivant.
            self._broadcast_state()

    # ── Calcul de l'angle d'attaque auto ──────────────────────────────
    def compute_auto_attack(self, climb: float) -> float:
        """
        Angle d'attaque automatique, décalé autour de CRUISE_ANGLE :
          angle = CRUISE_ANGLE + (climb / CLIMB_MAX) × (STALL_ANGLE - RES - CRUISE_ANGLE)

          climb =    0  →  3.0°   (croisière, palier)
          climb = +800  → +14.9°  (montée maximale)
          climb = -800  →  -8.9°  (descente maximale, asymétrique = plus réaliste)

        Note : cet offset ne s'applique QU'en mode auto. Le mode manuel
        utilise la formule directe climb = ceil × (angle / 14.9°).
        """
        if CLIMB_MAX_M_MIN == 0:
            return CRUISE_ANGLE
        angle_range = (STALL_ANGLE - CLIMB_RES) - CRUISE_ANGLE   # 12.9°
        angle = CRUISE_ANGLE + (climb / CLIMB_MAX_M_MIN) * angle_range
        # Limites : ne pas dépasser ±(STALL_ANGLE - RES) en absolu
        return max(-(STALL_ANGLE - CLIMB_RES), min(STALL_ANGLE - CLIMB_RES, angle))

    # ── Calcul de la vitesse sol (GS) ─────────────────────────────────
    def _compute_gs(self) -> float:
        """
        Vitesse sol GS = (power/100) × MAX_SPEED × cos(attack)

        Physique :
          - La puissance moteur détermine la magnitude totale de la vitesse.
          - L'angle d'attaque redistribue entre vertical (climb) et horizontal (GS).
          - En croisière (attack=3°, climb=0) : GS maximale pour la puissance donnée.
          - En montée steep (attack=14.9°)    : GS légèrement réduite (cos↘).
          - Au sol (power=0 ou state AU_SOL)  : GS = 0.
        """
        return ((self.motor_power_pct / 100.0)
                * MAX_SPEED_M_S
                * math.cos(math.radians(abs(self.attack_deg))))

    # ── Machine à états ───────────────────────────────────────────────
    def set_desired_altitude(self, alt_ft: float, climb: float, attack: float):
        """[FIX-1][FIX-4][FIX-5] Point d'entrée principal thread-safe."""
        with self._lock:
            # [FIX-4] Commandes bloquées en cas de décrochage
            if self._stalling:
                self.error_msg = "⚠ DÉCROCHAGE — Commandes bloquées. Utilisez ATTERRIR."
                return

            self.warnings  = []
            self.error_msg = ""

            if abs(attack) >= STALL_ANGLE:
                self.warnings.append(f"⚠ Risque de décrochage ! Angle = {attack}°")

            self.desired_alt_ft = alt_ft
            angle_max = STALL_ANGLE - CLIMB_RES  # 14.9°

            if climb != 0.0:
                # Back-calcul angle depuis taux de montée — inverse EXACT de tick() :
                #   tick()  : climb = ceil × (angle - CRUISE_ANGLE) / angle_range
                #   ici     : angle = CRUISE_ANGLE + (climb / ceil) × angle_range
                ceil = min(self.motor_power_pct * 10.0, CLIMB_MAX_M_MIN)
                if ceil > 0:
                    angle_range      = STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE  # 11.9°
                    angle_from_climb = CRUISE_ANGLE + (climb / ceil) * angle_range
                    # Clamp dans la plage valide (±STALL_ANGLE)
                    angle_from_climb = max(-STALL_ANGLE, min(STALL_ANGLE, angle_from_climb))
                    self.input_attack = angle_from_climb
                    self.warnings.append(
                        f"↺ Taux {climb:+.1f} m/min → angle calculé : "
                        f"{angle_from_climb:+.1f}°")
                else:
                    self.error_msg = "Puissance moteur nulle — augmentez la puissance avant d'entrer un taux de montée"
                    return
            elif attack != 0.0:
                # Input angle direct
                self.input_attack = attack
            else:
                # Mode auto
                self.input_attack = 0.0

            if self.state == STATE_AU_SOL:
                # Au sol, une altitude > 0 est obligatoire
                if alt_ft <= 0:
                    self.error_msg = ("AU_SOL : fournir une altitude > 0 pour décoller")
                    return
                # Cas 1 : altitude seule → mode auto
                if self.input_attack == 0.0:
                    self.motor_power_pct = 50.0
                    self._transition_to(STATE_CHANGEMENT)
                # Cas 2 : altitude + taux ou angle
                else:
                    self.motor_power_pct = 50.0
                    self._transition_to(STATE_CHANGEMENT)

            elif self.state == STATE_CHANGEMENT:
                if alt_ft == self.altitude_ft:
                    self._transition_to(STATE_VOL_CROISIERE)

            elif self.state == STATE_VOL_CROISIERE:
                if alt_ft != self.altitude_ft:
                    self._transition_to(STATE_CHANGEMENT)

    def set_motor_power(self, pct: float):
        """[FIX-1] Thread-safe."""
        with self._lock:
            self.motor_power_pct = max(0.0, min(100.0, pct))

    def _transition_to(self, new_state: int):
        """Transition d'état + notification AFDX. Appelée dans le lock."""
        old = STATE_NAMES[self.state]
        self.state = new_state
        new = STATE_NAMES[new_state]
        self.afdx.send("CALCULATEUR", "AGGREGATEUR",
                        {"event": "state_change", "from": old, "to": new})

    # ── Tick de simulation ────────────────────────────────────────────
    def tick(self, dt: float):
        """[FIX-1][FIX-4][FIX-5] Mise à jour physique thread-safe."""
        with self._lock:

            # ── [FIX-4] Chute libre ──────────────────────────────────
            if self._stalling:
                self.climb_m_min = -CLIMB_MAX_M_MIN
                self.attack_deg  = STALL_ANGLE
                climb_ft_s       = (self.climb_m_min * FT_PER_M) / 60.0
                self.altitude_ft = max(0.0, self.altitude_ft + climb_ft_s * dt)

                if self.altitude_ft <= 0.0:
                    self._stalling   = False
                    self.altitude_ft = 0.0
                    self.error_msg   = "⚠ IMPACT AU SOL suite au décrochage"
                    self._transition_to(STATE_AU_SOL)

                self.alt_history.append(self.altitude_ft)
                self._broadcast_state()
                return

            # ── AU_SOL ───────────────────────────────────────────────
            if self.state == STATE_AU_SOL:
                self.altitude_ft  = 0.0
                self.climb_m_min  = 0.0
                self.attack_deg   = 0.0
                self.speed_m_s    = 0.0
                self.alt_history.append(0.0)
                # Broadcast indispensable : met à jour le cache de
                # l'agrégateur après un reset (sinon altitude et messages
                # d'erreur restent figés à la dernière valeur connue).
                self._broadcast_state()
                return

            # ── VOL_CROISIÈRE ────────────────────────────────────────
            if self.state == STATE_VOL_CROISIERE:
                self.climb_m_min = 0.0
                self.attack_deg  = CRUISE_ANGLE
                self.speed_m_s   = self._compute_gs()   # GS pleine à angle de croisière
                self.alt_history.append(self.altitude_ft)
                self._broadcast_state()
                return

            # ── CHANGEMENT_ALT ───────────────────────────────────────
            #
            # Formule UNIFIÉE (auto et manuel) :
            #   climb = ceil × (angle - CRUISE_ANGLE) / angle_range
            #   angle = CRUISE_ANGLE + (climb / ceil) × angle_range
            #
            #   angle_range = STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE = 11.9°
            #   ceil        = min(power×10, 800)
            #
            #   angle = +14.9° → climb = +ceil  (+800 à 80%)
            #   angle = + 3.0° → climb =   0    (palier, croisière)
            #   angle = - 8.9° → climb = -ceil  (-800 à 80%)
            #   angle ≥ +15°   → décrochage positif
            #   angle ≤ - 9.0° → décrochage négatif

            angle_range    = STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE  # 11.9°
            base_power     = (self.motor_power_pct / 10.0) * 100.0
            effective_ceil = min(base_power, CLIMB_MAX_M_MIN)
            delta_ft       = self.desired_alt_ft - self.altitude_ft

            if self.input_attack != 0.0:
                # MODE MANUEL — formule avec offset CRUISE_ANGLE
                raw_climb = effective_ceil * (self.input_attack - CRUISE_ANGLE) / angle_range
            else:
                # MODE AUTO — direction vers la cible, plafond plein
                direction = 1.0 if delta_ft >= 0 else -1.0
                raw_climb = direction * effective_ceil

            # Décélération douce à l'approche de la cible (montée ou descente)
            if abs(delta_ft) < 0.1:
                self.climb_m_min = 0.0
            else:
                delta_m     = delta_ft / FT_PER_M
                slow_zone_m = max(1.0, abs(raw_climb) * 0.1)
                factor      = (max(0.05, abs(delta_m) / slow_zone_m)
                               if abs(delta_m) < slow_zone_m else 1.0)
                self.climb_m_min = raw_climb * factor

            self.climb_m_min = max(-CLIMB_MAX_M_MIN,
                                   min(CLIMB_MAX_M_MIN, self.climb_m_min))

            # Angle d'attaque — calculé depuis le taux final (sens unique)
            if self.input_attack != 0.0:
                self.attack_deg = self.input_attack
            else:
                self.attack_deg = self.compute_auto_attack(self.climb_m_min)

            # Détecter le décrochage :
            #   positif : angle ≥ +STALL_ANGLE (+15°)
            #   négatif : angle ≤  NEG_STALL_ANGLE (-9.0°)
            stalled = (self.attack_deg >= STALL_ANGLE or
                       self.attack_deg <= NEG_STALL_ANGLE)
            if stalled:
                self._stalling = True
                self.error_msg = (f"⚠ DÉCROCHAGE ! angle={self.attack_deg:.1f}° "
                                  f"— Chute libre initiée")
                self.alt_history.append(self.altitude_ft)
                self._broadcast_state()
                return

            # Mise à jour altitude
            climb_ft_s        = (self.climb_m_min * FT_PER_M) / 60.0
            self.altitude_ft += climb_ft_s * dt

            # Bornes altitude
            if self.altitude_ft >= ALT_MAX_FT:
                self.altitude_ft = ALT_MAX_FT
                self._transition_to(STATE_VOL_CROISIERE)
            elif self.altitude_ft <= 0.0:
                self.altitude_ft = 0.0
                self._transition_to(STATE_AU_SOL)

            # Altitude cible atteinte (tolérance 2 ft)
            # Si la cible est 0 ft (le sol), c'est AU_SOL et non VOL_CROISIÈRE
            if (self.state == STATE_CHANGEMENT and
                    abs(self.altitude_ft - self.desired_alt_ft) < 2.0):
                self.altitude_ft = self.desired_alt_ft
                if self.desired_alt_ft <= ALT_MIN_FT:
                    self._transition_to(STATE_AU_SOL)
                else:
                    self._transition_to(STATE_VOL_CROISIERE)

            # Vitesse sol GS — composante horizontale du vecteur vitesse
            # GS = (power/100) × MAX_SPEED × cos(attack)
            # Puissance → magnitude totale, angle → fraction horizontale.
            # En croisière (attack=3°) → presque toute la puissance en GS.
            # En montée steep (attack=14.9°) → légèrement réduit.
            self.speed_m_s = self._compute_gs()

            self.alt_history.append(self.altitude_ft)
            self._broadcast_state()

    def _broadcast_state(self):
        """
        [FIX-3] Encode les mots ARINC 429 et enrichit le payload AFDX
        avec les métadonnées nécessaires à l'agrégateur
        (desired_alt, power, speed, warnings, error).
        """
        try:
            w001 = ARINC429.encode_label001_altitude(int(self.altitude_ft),
                                                     self.state)
            w002 = ARINC429.encode_label002_climb(self.climb_m_min)
            w003 = ARINC429.encode_label003_attack(self.attack_deg)
            payload = {
                # Mots ARINC 429 sur le bus
                "label001":    ARINC429.word_to_hex(w001),
                "label002":    ARINC429.word_to_hex(w002),
                "label003":    ARINC429.word_to_hex(w003),
                # Métadonnées AFDX (hors ARINC)
                "power":       round(self.motor_power_pct, 1),
                "speed":       round(self.speed_m_s, 2),
                "desired_alt": round(self.desired_alt_ft, 0),
                "warnings":    list(self.warnings),
                "error":       self.error_msg,
            }
            self.afdx.send("CALCULATEUR", "AGGREGATEUR", payload)
        except Exception:
            pass   # silencieux pour éviter de bloquer la simulation
