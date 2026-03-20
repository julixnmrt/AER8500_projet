"""
AER8500 – Informatique embarquée de l'avionique
Mini-Projet Hiver 2026 — Simulation complète
Protocoles : ARINC 429 + AFDX
"""

import tkinter as tk
import threading
import time
import math
import json
from datetime import datetime
from collections import deque

# ─────────────────────────────────────────────
#  CONSTANTES DU SYSTÈME
# ─────────────────────────────────────────────
ALT_MAX_FT       = 40000      # pieds
ALT_MIN_FT       = 0
CLIMB_MAX_M_MIN  = 800.0      # m/min
CLIMB_RES        = 0.1        # résolution taux de montée
ALT_RES          = 1          # résolution altitude (pied)
STALL_ANGLE      = 15.0       # angle de décrochage positif (°)
CRUISE_ANGLE     = 3.0        # angle d'attaque en vol de croisière (palier)
# Décrochage négatif : angle en dessous duquel l'avion décroche en descente
# = CRUISE_ANGLE - (STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE) - CLIMB_RES = -9.0°
NEG_STALL_ANGLE  = CRUISE_ANGLE - (STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE) - CLIMB_RES
ATTACK_MAX       = STALL_ANGLE              # 15.0° — permet la démo du décrochage positif
ATTACK_MIN       = -(STALL_ANGLE)           # -15.0° — permet la démo du décrochage négatif
MAX_SPEED_M_S    = 150.0      # vitesse sol max à 100% puissance (m/s ≈ 540 km/h)
SIM_TICK_S       = 0.05       # secondes par tick de simulation

# Conversion
FT_PER_M         = 3.28084

# États
STATE_AU_SOL        = 0
STATE_CHANGEMENT    = 1
STATE_VOL_CROISIERE = 2
STATE_NAMES = {
    STATE_AU_SOL:        "AU_SOL",
    STATE_CHANGEMENT:    "CHANGEMENT_ALT",
    STATE_VOL_CROISIERE: "VOL_CROISIÈRE",
}


# ─────────────────────────────────────────────
#  ENCODAGE ARINC 429
# ─────────────────────────────────────────────
class ARINC429:
    """
    Mot ARINC 429 = 32 bits :
      [1:8]   = label (octal, LSB first)
      [9:10]  = SDI
      [11:28] = données
      [29:30] = SSM
      [31]    = parity (odd)
    """

    @staticmethod
    def odd_parity(word: int) -> int:
        """Calcule le bit de parité impaire sur 32 bits."""
        return 1 - (bin(word).count('1') % 2)

    @staticmethod
    def encode_label001_altitude(altitude_ft: int, state: int) -> int:
        """
        Label 001 — Altitude + État
          Altitude  → bits [13:28]  (16 bits binaire, résolution 1 pied)
          État      → bits [11:12]  (2 bits)
          Label     → bits [1:8]    = 0x01
        """
        if not (ALT_MIN_FT <= altitude_ft <= ALT_MAX_FT):
            raise ValueError(f"Altitude hors limites : {altitude_ft} ft")
        if state not in (0, 1, 2):
            raise ValueError(f"État invalide : {state}")

        label      = 0b00000001
        state_bits = (state & 0x3) << 10
        alt_bits   = (altitude_ft & 0xFFFF) << 12

        word   = label | state_bits | alt_bits
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label001(word: int):
        """Décode label 001 → (altitude_ft, state)"""
        altitude_ft = (word >> 12) & 0xFFFF
        state       = (word >> 10) & 0x3
        return altitude_ft, state

    @staticmethod
    def encode_bcd(value: float, digits: int, decimals: int) -> int:
        """Encode un flottant en BCD (digits chiffres, decimals décimales)."""
        factor  = 10 ** decimals
        int_val = round(abs(value) * factor)
        sign    = 0 if value >= 0 else 1
        bcd     = 0
        for i in range(digits):
            bcd |= (int_val % 10) << (i * 4)
            int_val //= 10
        return bcd | (sign << (digits * 4))

    @staticmethod
    def decode_bcd(bcd: int, digits: int, decimals: int) -> float:
        """Décode un BCD → flottant."""
        sign = (bcd >> (digits * 4)) & 1
        val  = 0
        for i in range(digits - 1, -1, -1):
            val = val * 10 + ((bcd >> (i * 4)) & 0xF)
        result = val / (10 ** decimals)
        return -result if sign else result

    @staticmethod
    def encode_label002_climb(climb_m_min: float) -> int:
        """Label 002 — Taux de montée BCD 4 chiffres, résolution 0.1 m/min"""
        label  = 0b00000010
        bcd    = ARINC429.encode_bcd(climb_m_min, 4, 1)
        word   = label | (bcd << 8)
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label002(word: int) -> float:
        bcd = (word >> 8) & 0x1FFFFF
        return ARINC429.decode_bcd(bcd, 4, 1)

    @staticmethod
    def encode_label003_attack(angle_deg: float) -> int:
        """Label 003 — Angle d'attaque BCD 3 chiffres, résolution 0.1°"""
        label  = 0b00000011
        bcd    = ARINC429.encode_bcd(angle_deg, 3, 1)
        word   = label | (bcd << 8)
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label003(word: int) -> float:
        bcd = (word >> 8) & 0x1FFFFF
        return ARINC429.decode_bcd(bcd, 3, 1)

    @staticmethod
    def word_to_hex(word: int) -> str:
        return f"0x{word:08X}"

    @staticmethod
    def word_to_bin(word: int) -> str:
        return f"{word:032b}"


# ─────────────────────────────────────────────
#  SIMULATION AFDX (réseau virtuel)
# ─────────────────────────────────────────────
class AFDXNetwork:
    """
    Simulation d'un réseau AFDX simplifié.
    Deux canaux (A et B) pour la redondance.
    Les trames sont des dictionnaires sérialisés en JSON.
    """

    def __init__(self):
        self.channel_a: deque = deque(maxlen=50)
        self.channel_b: deque = deque(maxlen=50)
        self.log: list = []

    def _build_frame(self, src: str, dst: str, payload: dict) -> dict:
        return {
            "timestamp": datetime.now().isoformat(),
            "src":       src,
            "dst":       dst,
            "payload":   payload,
        }

    def send(self, src: str, dst: str, payload: dict):
        """Envoie sur les deux canaux (redondance A + B)."""
        frame = self._build_frame(src, dst, payload)
        serialised = json.dumps(frame)
        self.channel_a.append(serialised)
        self.channel_b.append(serialised)
        self.log.append(f"[AFDX A+B] {src}→{dst} | {payload}")

    def receive(self, channel: str = 'A') -> dict | None:
        """Reçoit la trame la plus récente sur un canal (A ou B)."""
        buf = self.channel_a if channel == 'A' else self.channel_b
        if buf:
            return json.loads(buf[-1])
        return None

    def get_log(self, n: int = 8) -> list:
        return self.log[-n:]


# ─────────────────────────────────────────────
#  CALCULATEUR AVIONIQUE (logique métier)
# ─────────────────────────────────────────────
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


# ─────────────────────────────────────────────
#  AGRÉGATEUR
# ─────────────────────────────────────────────
class Aggregator:
    """
    [FIX-3] L'agrégateur décode maintenant les trames AFDX reçues
    (labels ARINC 429) au lieu d'accéder directement aux attributs
    du calculateur.  Failover automatique canal A → canal B.
    """

    def __init__(self, afdx: AFDXNetwork, calculator: AvionicsCalculator):
        self.afdx       = afdx
        self.calc       = calculator  # conservé uniquement pour reset()
        self._lock      = threading.Lock()
        # Cache interne — initialisé à des valeurs neutres
        self._data: dict = {
            "altitude":  0.0,
            "climb":     0.0,
            "power":     0.0,
            "speed":     0.0,
            "attack":    0.0,
            "state":     "AU_SOL",
            "state_id":  STATE_AU_SOL,
            "desired":   0.0,
            "warnings":  [],
            "error":     "",
            # Mots ARINC bruts (pour l'affichage hex)
            "w001_hex":  "0x00000000",
            "w002_hex":  "0x00000000",
            "w003_hex":  "0x00000000",
        }

    def update(self):
        """[FIX-3] Décode les labels ARINC 429 depuis les trames AFDX.
        Failover automatique : tente canal A puis canal B."""
        frame = self.afdx.receive('A') or self.afdx.receive('B')
        if not frame:
            return

        p = frame.get("payload", {})
        # Ignorer les trames d'événements (state_change) sans labels ARINC
        if "label001" not in p:
            return

        try:
            w001 = int(p["label001"], 16)
            w002 = int(p["label002"], 16)
            w003 = int(p["label003"], 16)

            # Décodage ARINC 429
            alt_ft, state_id = ARINC429.decode_label001(w001)
            climb            = ARINC429.decode_label002(w002)
            attack           = ARINC429.decode_label003(w003)

            with self._lock:
                self._data.update({
                    "altitude":  float(alt_ft),
                    "climb":     climb,
                    "attack":    attack,
                    "state_id":  state_id,
                    "state":     STATE_NAMES.get(state_id, "INCONNU"),
                    "power":     p.get("power",       0.0),
                    "speed":     p.get("speed",        0.0),
                    "desired":   p.get("desired_alt",  0.0),
                    "warnings":  p.get("warnings",     []),
                    "error":     p.get("error",        ""),
                    # Mots bruts pour affichage hex
                    "w001_hex":  p["label001"],
                    "w002_hex":  p["label002"],
                    "w003_hex":  p["label003"],
                })
        except Exception:
            pass

    def get_display_data(self) -> dict:
        """Retourne un snapshot thread-safe des données décodées."""
        with self._lock:
            return dict(self._data)


# ─────────────────────────────────────────────
#  INTERFACE TKINTER
# ─────────────────────────────────────────────
COLORS = {
    "bg":        "#0d1117",
    "panel":     "#161b22",
    "border":    "#30363d",
    "accent":    "#58a6ff",
    "green":     "#3fb950",
    "yellow":    "#d29922",
    "red":       "#f85149",
    "text":      "#e6edf3",
    "text_dim":  "#8b949e",
    "au_sol":    "#8b949e",
    "changement":"#d29922",
    "croisiere": "#3fb950",
}

STATE_COLORS = {
    STATE_AU_SOL:        COLORS["au_sol"],
    STATE_CHANGEMENT:    COLORS["changement"],
    STATE_VOL_CROISIERE: COLORS["croisiere"],
}


class AvionicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AER8500 – Simulation Avionique")
        self.configure(bg=COLORS["bg"])
        self.resizable(True, True)
        self.minsize(900, 680)

        # Couche métier
        self.afdx       = AFDXNetwork()
        self.calculator = AvionicsCalculator(self.afdx)
        self.aggregator = Aggregator(self.afdx, self.calculator)

        # Altitude mise en cache pour _draw_alt_bar (évite l'accès direct
        # au calculateur depuis le thread UI)
        self._last_alt_ft: float = 0.0

        # Thread de simulation
        self._running    = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)

        self._build_ui()
        self._sim_thread.start()
        self._update_ui()

    # ── Build UI ─────────────────────────────────────────────────────
    def _build_ui(self):
        title_bar = tk.Frame(self, bg=COLORS["bg"])
        title_bar.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(title_bar, text="✈  AER8500 — Informatique embarquée de l'avionique",
                 font=("Courier New", 13, "bold"),
                 bg=COLORS["bg"], fg=COLORS["accent"]).pack(side="left")
        tk.Label(title_bar, text="Protocoles : ARINC 429 + AFDX",
                 font=("Courier New", 9),
                 bg=COLORS["bg"], fg=COLORS["text_dim"]).pack(side="right")

        sep = tk.Frame(self, bg=COLORS["border"], height=1)
        sep.pack(fill="x", padx=16)

        main = tk.Frame(self, bg=COLORS["bg"])
        main.pack(fill="both", expand=True, padx=16, pady=10)

        left  = tk.Frame(main, bg=COLORS["bg"])
        right = tk.Frame(main, bg=COLORS["bg"])
        left.pack(side="left",  fill="both", expand=True)
        right.pack(side="right", fill="both", expand=True, padx=(12, 0))

        self._build_state_display(left)
        self._build_screens(left)
        self._build_arinc_display(left)

        self._build_input_panel(right)
        self._build_motor_panel(right)
        self._build_afdx_log(right)
        self._build_altitude_bar(right)

        self.error_bar = tk.Label(self, text="", font=("Courier New", 9, "bold"),
                                  bg="#300", fg=COLORS["red"],
                                  anchor="w", padx=10)
        self.error_bar.pack(fill="x", padx=16, pady=(0, 8))

    def _section(self, parent, title: str) -> tk.Frame:
        outer = tk.Frame(parent, bg=COLORS["panel"],
                         highlightbackground=COLORS["border"],
                         highlightthickness=1)
        outer.pack(fill="x", pady=5)
        tk.Label(outer, text=f"  {title}",
                 font=("Courier New", 9, "bold"),
                 bg=COLORS["border"], fg=COLORS["accent"],
                 anchor="w").pack(fill="x")
        inner = tk.Frame(outer, bg=COLORS["panel"])
        inner.pack(fill="x", padx=8, pady=6)
        return inner

    def _screen(self, parent, label: str, width=14):
        f = tk.Frame(parent, bg=COLORS["panel"])
        f.pack(side="left", padx=6, pady=4)
        tk.Label(f, text=label, font=("Courier New", 8),
                 bg=COLORS["panel"], fg=COLORS["text_dim"]).pack()
        var = tk.StringVar(value="---")
        lbl = tk.Label(f, textvariable=var,
                       font=("Courier New", 14, "bold"),
                       bg="#0a0f15", fg=COLORS["green"],
                       width=width, anchor="e",
                       relief="sunken", bd=2, padx=4)
        lbl.pack()
        return var, lbl

    def _build_state_display(self, parent):
        f = self._section(parent, "MODE AVIONIQUE")
        self.state_var = tk.StringVar(value="AU_SOL")
        self.state_lbl = tk.Label(f, textvariable=self.state_var,
                                  font=("Courier New", 18, "bold"),
                                  bg=COLORS["panel"], fg=COLORS["au_sol"],
                                  width=20, anchor="center")
        self.state_lbl.pack(pady=4)

    def _build_screens(self, parent):
        f = self._section(parent, "ÉCRANS SYSTÈME")
        row1 = tk.Frame(f, bg=COLORS["panel"])
        row1.pack(fill="x")
        self.scr_alt,    _ = self._screen(row1, "ALTITUDE (ft)")
        self.scr_climb,  _ = self._screen(row1, "TAUX MONTÉE (m/min)")
        self.scr_power,  _ = self._screen(row1, "PUISSANCE (%)")

        row2 = tk.Frame(f, bg=COLORS["panel"])
        row2.pack(fill="x")
        self.scr_speed,   _ = self._screen(row2, "VITESSE SOL GS (m/s)")
        self.scr_attack,  _ = self._screen(row2, "ANGLE ATTAQUE (°)")
        self.scr_desired, _ = self._screen(row2, "ALT. DÉSIRÉE (ft)")

    def _build_arinc_display(self, parent):
        f = self._section(parent, "MOTS ARINC 429 (hex)")
        self.arinc_vars = {}
        for label, desc in [("001", "Altitude + État"),
                             ("002", "Taux de montée (BCD)"),
                             ("003", "Angle d'attaque (BCD)")]:
            row = tk.Frame(f, bg=COLORS["panel"])
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"Label {label} ({desc}) :",
                     font=("Courier New", 8), fg=COLORS["text_dim"],
                     bg=COLORS["panel"], width=30, anchor="w").pack(side="left")
            var = tk.StringVar(value="0x00000000")
            tk.Label(row, textvariable=var,
                     font=("Courier New", 9, "bold"),
                     fg=COLORS["accent"], bg=COLORS["panel"]).pack(side="left")
            self.arinc_vars[label] = var

    def _build_input_panel(self, parent):
        f = self._section(parent, "PANNEAU DE CONTRÔLE")
        inputs = [
            ("Altitude désirée (ft) [0–40000]", "alt"),
            ("Taux de montée (m/min) [±800]",   "climb"),
            ("Angle d'attaque (°)   [±16]",     "attack"),
        ]
        self.input_vars = {}
        for label, key in inputs:
            row = tk.Frame(f, bg=COLORS["panel"])
            row.pack(fill="x", pady=3)
            tk.Label(row, text=label, font=("Courier New", 9),
                     bg=COLORS["panel"], fg=COLORS["text"],
                     width=34, anchor="w").pack(side="left")
            var = tk.StringVar()
            entry = tk.Entry(row, textvariable=var, width=10,
                             font=("Courier New", 11, "bold"),
                             bg="#0a0f15", fg=COLORS["green"],
                             insertbackground=COLORS["green"],
                             relief="sunken", bd=2)
            entry.pack(side="left", padx=4)
            self.input_vars[key] = var

        btn = tk.Button(f, text="▶  ENVOYER CONSIGNE",
                        font=("Courier New", 10, "bold"),
                        bg=COLORS["accent"], fg="#000",
                        activebackground="#79c0ff",
                        relief="flat", padx=10, pady=4,
                        command=self._on_send)
        btn.pack(pady=(8, 2))

        reset_btn = tk.Button(f, text="⏹  ATTERRIR (AU_SOL)",
                              font=("Courier New", 9),
                              bg=COLORS["red"], fg="#fff",
                              activebackground="#ff6b6b",
                              relief="flat", padx=8, pady=3,
                              command=self._on_land)
        reset_btn.pack(pady=2)

    def _build_motor_panel(self, parent):
        f = self._section(parent, "PUISSANCE MOTEUR (%)")
        self.power_slider = tk.Scale(f, from_=0, to=100,
                                     orient="horizontal",
                                     bg=COLORS["panel"], fg=COLORS["text"],
                                     troughcolor="#0a0f15",
                                     activebackground=COLORS["accent"],
                                     highlightthickness=0,
                                     font=("Courier New", 8),
                                     command=self._on_power_change)
        self.power_slider.set(50)
        self.power_slider.pack(fill="x", padx=4)

    def _build_afdx_log(self, parent):
        f = self._section(parent, "JOURNAL AFDX (Canaux A & B — Redondance)")
        self.afdx_text = tk.Text(f, height=7,
                                  font=("Courier New", 8),
                                  bg="#0a0f15", fg=COLORS["text_dim"],
                                  state="disabled", relief="flat",
                                  wrap="none")
        scrollbar = tk.Scrollbar(f, command=self.afdx_text.yview)
        self.afdx_text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.afdx_text.pack(fill="both", expand=True)

    def _build_altitude_bar(self, parent):
        f = self._section(parent, "ALTITUDE COURANTE (barre)")
        self.alt_canvas = tk.Canvas(f, height=30, bg="#0a0f15",
                                     highlightthickness=0)
        self.alt_canvas.pack(fill="x", padx=4, pady=4)
        self.alt_canvas.bind("<Configure>", self._draw_alt_bar)

    # ── Callbacks ────────────────────────────────────────────────────
    def _on_send(self):
        """Lecture, validation des entrées, puis envoi au calculateur."""
        self.calculator.error_msg = ""

        ok_a, alt,    err_a = self.calculator.validate_altitude(
            self.input_vars["alt"].get())
        ok_c, climb,  err_c = self.calculator.validate_climb(
            self.input_vars["climb"].get())
        ok_k, attack, err_k = self.calculator.validate_attack(
            self.input_vars["attack"].get())

        errors = [e for e in [err_a, err_c, err_k] if e]
        if errors:
            self.error_bar.config(text="  ✘  " + " | ".join(errors), bg="#300")
            return
        self.error_bar.config(text="", bg=COLORS["bg"])
        self.calculator.set_desired_altitude(alt, climb, attack)

    def _on_land(self):
        """[FIX-1][FIX-4] Réinitialisation via reset() thread-safe."""
        self.calculator.reset()
        self.power_slider.set(0)
        self.input_vars["alt"].set("")
        self.input_vars["climb"].set("")
        self.input_vars["attack"].set("")
        self.error_bar.config(text="", bg=COLORS["bg"])

    def _on_power_change(self, val):
        """[FIX-1] Passe par set_motor_power() protégé."""
        self.calculator.set_motor_power(float(val))

    # ── Boucle simulation (thread dédié) ────────────────────────────
    def _sim_loop(self):
        while self._running:
            self.calculator.tick(SIM_TICK_S)
            self.aggregator.update()
            time.sleep(SIM_TICK_S)

    # ── Mise à jour UI ───────────────────────────────────────────────
    def _update_ui(self):
        data = self.aggregator.get_display_data()

        # Mise en cache pour _draw_alt_bar (évite accès direct au calc)
        self._last_alt_ft = data["altitude"]

        # Mode
        sid = data["state_id"]
        self.state_var.set(data["state"])
        self.state_lbl.config(fg=STATE_COLORS.get(sid, COLORS["text"]))

        # Écrans
        self.scr_alt.set(f"{data['altitude']:,.0f}")
        self.scr_climb.set(f"{data['climb']:+.1f}")
        self.scr_power.set(f"{data['power']:.1f}")
        self.scr_speed.set(f"{data['speed']:.2f}")
        self.scr_attack.set(f"{data['attack']:+.2f}")
        self.scr_desired.set(f"{data['desired']:,.0f}")

        # [FIX-3] Mots ARINC 429 — lus depuis les trames décodées
        self.arinc_vars["001"].set(data.get("w001_hex", "0x00000000"))
        self.arinc_vars["002"].set(data.get("w002_hex", "0x00000000"))
        self.arinc_vars["003"].set(data.get("w003_hex", "0x00000000"))

        # Journal AFDX
        log = self.afdx.get_log(6)
        self.afdx_text.config(state="normal")
        self.afdx_text.delete("1.0", "end")
        for line in log:
            self.afdx_text.insert("end", line[:120] + "\n")
        self.afdx_text.config(state="disabled")

        # Barre altitude
        self._draw_alt_bar()

        # Erreurs / avertissements
        err   = data.get("error",    "")
        warns = data.get("warnings", [])
        msg_parts = []
        if err:
            msg_parts.append(err)
        if warns:
            msg_parts.extend(warns)
        if msg_parts:
            self.error_bar.config(text="  ⚠  " + "  |  ".join(msg_parts), bg="#300")
        elif not self.error_bar["text"].startswith("  ✘"):
            self.error_bar.config(text="", bg=COLORS["bg"])

        # Sync slider avec puissance calculateur
        self.power_slider.set(int(data["power"]))

        self.after(100, self._update_ui)

    def _draw_alt_bar(self, event=None):
        """[FIX-1] Utilise le cache _last_alt_ft — plus d'accès direct au calc."""
        c = self.alt_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        ratio = self._last_alt_ft / ALT_MAX_FT
        bar_w = int(w * ratio)

        c.create_rectangle(0, 0, w, h, fill="#0a0f15", outline="")

        if bar_w > 0:
            if ratio < 0.5:
                r_val = int(255 * ratio * 2)
                g_val = 180
            else:
                r_val = 200
                g_val = int(180 * (1 - ratio) * 2)
            color = f"#{r_val:02x}{g_val:02x}30"
            c.create_rectangle(0, 2, bar_w, h - 2, fill=color, outline="")

        pct = ratio * 100
        c.create_text(w // 2, h // 2,
                      text=f"{self._last_alt_ft:,.0f} ft  ({pct:.1f}%)",
                      fill=COLORS["text"], font=("Courier New", 9, "bold"))

        for mark in [0.25, 0.5, 0.75]:
            x = int(w * mark)
            c.create_line(x, 0, x, h, fill=COLORS["border"], width=1)

    def on_close(self):
        self._running = False
        self.destroy()


# ─────────────────────────────────────────────
#  POINT D'ENTRÉE
# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = AvionicsApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()