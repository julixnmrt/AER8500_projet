from panel.control_panel import ControlPanel
from aggregateur.aggregateur import Aggregateur
from calculateur.calculateur import Calculateur

"""
def main():

    calculateur = Calculateur()
    aggregateur = Aggregateur(calculateur)
    panel = ControlPanel(aggregateur)

    panel.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSystem stopped")

        """

"""
AER8500 – Informatique embarquée de l'avionique
Mini-Projet Hiver 2026 — Simulation complète
Protocoles : ARINC 429 + AFDX
"""

import tkinter as tk
from tkinter import ttk, font
import threading
import time
import math
import struct
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
ATTACK_MAX       = 16.0       # degrés
ATTACK_MIN       = -16.0
STALL_ANGLE      = 15.0       # angle de décrochage
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
      [29]    = SSM bit 1
      [30]    = SSM bit 2
      [31]    = parity (odd)
      (bit 32 = unused / MSB)
    """

    @staticmethod
    def odd_parity(word: int) -> int:
        """Calcule le bit de parité impaire sur 32 bits."""
        return 1 - (bin(word).count('1') % 2)

    @staticmethod
    def encode_label001_altitude(altitude_ft: int, state: int) -> int:
        """
        Label 001 — Altitude + État
          Altitude  → bits [13:28]  (16 bits, binaire, résolution 1 pied)
          État      → bits [11:12]  (2 bits)
          Label     → bits [1:8]    = 0x01
        """
        if not (ALT_MIN_FT <= altitude_ft <= ALT_MAX_FT):
            raise ValueError(f"Altitude hors limites : {altitude_ft} ft")
        if state not in (0, 1, 2):
            raise ValueError(f"État invalide : {state}")

        label = 0b00000001                       # label 001 (octal 001 = 0x01)
        state_bits = (state & 0x3) << 10         # bits 11:12 (0-indexé bit 10,11)
        alt_bits   = (altitude_ft & 0xFFFF) << 12  # bits 13:28 (0-indexé 12..27)

        word = label | state_bits | alt_bits
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word |= (parity << 31)
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
        """
        Label 002 — Taux de montée (BCD 4 chiffres, résolution 0.1 m/min)
        bits [9:28] = BCD (4 chiffres + signe)
        """
        label = 0b00000010
        bcd   = ARINC429.encode_bcd(climb_m_min, 4, 1)   # 4 chiffres, 1 décimale
        word  = label | (bcd << 8)
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label002(word: int) -> float:
        bcd = (word >> 8) & 0x1FFFFF
        return ARINC429.decode_bcd(bcd, 4, 1)

    @staticmethod
    def encode_label003_attack(angle_deg: float) -> int:
        """
        Label 003 — Angle d'attaque (BCD 3 chiffres, résolution 0.1 deg)
        bits [9:24] = BCD (3 chiffres + signe)
        """
        label = 0b00000011
        bcd   = ARINC429.encode_bcd(angle_deg, 3, 1)
        word  = label | (bcd << 8)
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
            "src": src,
            "dst": dst,
            "payload": payload,
        }

    def send(self, src: str, dst: str, payload: dict):
        """Envoie sur les deux canaux (redondance)."""
        frame = self._build_frame(src, dst, payload)
        self.channel_a.append(json.dumps(frame))
        self.channel_b.append(json.dumps(frame))
        self.log.append(f"[AFDX] {src}→{dst} | {payload}")

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
    """

    def __init__(self, afdx: AFDXNetwork):
        self.afdx = afdx

        # État interne
        self.state           = STATE_AU_SOL
        self.altitude_ft     = 0.0          # altitude courante (pieds)
        self.climb_m_min     = 0.0          # taux de montée courant (m/min)
        self.attack_deg      = 0.0          # angle d'attaque courant (degrés)
        self.motor_power_pct = 0.0          # puissance moteur (%)
        self.speed_m_s       = 0.0          # vitesse horizontale (m/s) — affichage

        # Consignes
        self.desired_alt_ft  = 0.0
        self.input_climb     = 0.0          # consigne taux montée (0 = auto)
        self.input_attack    = 0.0          # consigne angle attaque (0 = auto)

        # Messages d'erreur
        self.error_msg       = ""
        self.warnings        = []

        # Historique pour affichage
        self.alt_history     = deque(maxlen=120)

    # ── Validation des entrées ────────────────
    def validate_altitude(self, value: str) -> tuple[bool, float, str]:
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Altitude : valeur numérique requise"
        if not (ALT_MIN_FT <= v <= ALT_MAX_FT):
            return False, 0.0, f"Altitude hors limites [0, {ALT_MAX_FT}] ft"
        return True, v, ""

    def validate_climb(self, value: str) -> tuple[bool, float, str]:
        if value.strip() == "" or value.strip() == "0":
            return True, 0.0, ""
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Taux de montée : valeur numérique requise"
        if not (-CLIMB_MAX_M_MIN <= v <= CLIMB_MAX_M_MIN):
            return False, 0.0, f"Taux de montée hors limites [±{CLIMB_MAX_M_MIN}] m/min"
        return True, v, ""

    def validate_attack(self, value: str) -> tuple[bool, float, str]:
        if value.strip() == "" or value.strip() == "0":
            return True, 0.0, ""
        try:
            v = float(value)
        except ValueError:
            return False, 0.0, "Angle d'attaque : valeur numérique requise"
        if not (ATTACK_MIN <= v <= ATTACK_MAX):
            return False, 0.0, f"Angle d'attaque hors limites [±{ATTACK_MAX}]°"
        return True, v, ""

    # ── Calcul du taux de montée nominal ──────
    def compute_nominal_climb(self) -> float:
        """
        Taux de montée = 100 m/min par 10% de puissance moteur.
        Si on approche de l'altitude désirée, on décélère progressivement (rampe).
        """
        base_climb = (self.motor_power_pct / 10.0) * 100.0   # m/min

        # Direction : montée ou descente
        delta_ft = self.desired_alt_ft - self.altitude_ft
        if abs(delta_ft) < 0.1:
            return 0.0
        direction = 1.0 if delta_ft > 0 else -1.0

        # Décélération douce à l'approche de l'altitude cible
        delta_m = delta_ft / FT_PER_M
        slow_zone_m = base_climb * 0.8   # zone de décélération (en mètres)
        if slow_zone_m < 1:
            slow_zone_m = 1
        if abs(delta_m) < slow_zone_m:
            factor = abs(delta_m) / slow_zone_m
            factor = max(0.05, factor)   # minimum 5% pour ne pas stagner
        else:
            factor = 1.0

        return direction * base_climb * factor

    # ── Calcul de l'angle d'attaque auto ──────
    def compute_auto_attack(self) -> float:
        """
        Angle d'attaque basé sur le taux de montée normalisé.
        montée max → +15°, descente max → -15°, croisière → 0°
        Limité à ±15° (seuil de décrochage)
        """
        if CLIMB_MAX_M_MIN == 0:
            return 0.0
        angle = (self.climb_m_min / CLIMB_MAX_M_MIN) * STALL_ANGLE
        return max(-STALL_ANGLE, min(STALL_ANGLE, angle))

    # ── Machine à états ───────────────────────
    def set_desired_altitude(self, alt_ft: float, climb: float, attack: float):
        """Point d'entrée principal : nouvelle consigne de l'utilisateur."""
        self.warnings = []

        # Décrochage
        if abs(attack) >= STALL_ANGLE:
            self.warnings.append(f"⚠ Risque de décrochage ! Angle = {attack}°")

        self.desired_alt_ft = alt_ft
        self.input_climb    = climb
        self.input_attack   = attack

        if self.state == STATE_AU_SOL:
            # Cas 1 : altitude fournie, taux/angle nuls → auto
            if alt_ft > 0 and climb == 0.0 and attack == 0.0:
                self.motor_power_pct = 50.0    # puissance initiale 50%
                self._transition_to(STATE_CHANGEMENT)
            # Cas 2 : altitude + taux + angle tous non nuls
            elif alt_ft > 0 and climb != 0.0 and attack != 0.0:
                self.motor_power_pct = 50.0
                self._transition_to(STATE_CHANGEMENT)
            else:
                self.error_msg = "AU_SOL : fournir une altitude > 0 (et optionnellement taux + angle non nuls)"

        elif self.state == STATE_CHANGEMENT:
            # On réagit immédiatement à la nouvelle consigne
            if alt_ft == self.altitude_ft:
                self._transition_to(STATE_VOL_CROISIERE)

        elif self.state == STATE_VOL_CROISIERE:
            if alt_ft != self.altitude_ft:
                self._transition_to(STATE_CHANGEMENT)

    def set_motor_power(self, pct: float):
        self.motor_power_pct = max(0.0, min(100.0, pct))

    def _transition_to(self, new_state: int):
        old = STATE_NAMES[self.state]
        self.state = new_state
        new = STATE_NAMES[new_state]
        self.afdx.send("CALCULATEUR", "AGGREGATEUR",
                        {"event": "state_change", "from": old, "to": new})

    # ── Tick de simulation ────────────────────
    def tick(self, dt: float):
        """Mise à jour physique du système (dt en secondes)."""
        if self.state == STATE_AU_SOL:
            self.altitude_ft  = 0.0
            self.climb_m_min  = 0.0
            self.attack_deg   = 0.0
            self.speed_m_s    = 0.0
            self.alt_history.append(0.0)
            return

        if self.state == STATE_VOL_CROISIERE:
            self.climb_m_min = 0.0
            # Maintenir altitude constante
            self.alt_history.append(self.altitude_ft)
            self._broadcast_state()
            return

        # ── CHANGEMENT_ALT ──
        # Taux de montée
        if self.input_climb != 0.0:
            # L'utilisateur a imposé un taux
            self.climb_m_min = self.input_climb
        else:
            self.climb_m_min = self.compute_nominal_climb()

        # Clamp
        self.climb_m_min = max(-CLIMB_MAX_M_MIN,
                               min(CLIMB_MAX_M_MIN, self.climb_m_min))

        # Angle d'attaque
        if self.input_attack != 0.0:
            self.attack_deg = self.input_attack
        else:
            self.attack_deg = self.compute_auto_attack()

        # Décrochage ?
        if abs(self.attack_deg) >= STALL_ANGLE:
            if "décrochage" not in self.error_msg:
                self.error_msg = f"⚠ DÉCROCHAGE ! angle={self.attack_deg:.1f}°"

        # Mise à jour altitude (conversion m/min → ft/s)
        climb_ft_s = (self.climb_m_min * FT_PER_M) / 60.0
        self.altitude_ft += climb_ft_s * dt

        # Bornes
        if self.altitude_ft >= ALT_MAX_FT:
            self.altitude_ft = ALT_MAX_FT
            self._transition_to(STATE_VOL_CROISIERE)
        elif self.altitude_ft <= 0.0:
            self.altitude_ft = 0.0
            self._transition_to(STATE_AU_SOL)

        # Vérification altitude cible atteinte
        delta = abs(self.altitude_ft - self.desired_alt_ft)
        if delta < 2.0 and self.state == STATE_CHANGEMENT:
            self.altitude_ft = self.desired_alt_ft
            self._transition_to(STATE_VOL_CROISIERE)

        # Vitesse horizontale (approximation)
        self.speed_m_s = abs(self.climb_m_min / 60.0) * math.cos(
            math.radians(abs(self.attack_deg))) * 5.0

        self.alt_history.append(self.altitude_ft)
        self._broadcast_state()

    def _broadcast_state(self):
        """Encode et envoie les mots ARINC 429 via AFDX."""
        try:
            w001 = ARINC429.encode_label001_altitude(
                int(self.altitude_ft), self.state)
            w002 = ARINC429.encode_label002_climb(self.climb_m_min)
            w003 = ARINC429.encode_label003_attack(self.attack_deg)
            payload = {
                "label001": ARINC429.word_to_hex(w001),
                "label002": ARINC429.word_to_hex(w002),
                "label003": ARINC429.word_to_hex(w003),
                "alt_ft":   round(self.altitude_ft, 1),
                "climb":    round(self.climb_m_min, 1),
                "attack":   round(self.attack_deg, 2),
                "power":    round(self.motor_power_pct, 1),
                "state":    STATE_NAMES[self.state],
            }
            self.afdx.send("CALCULATEUR", "AGGREGATEUR", payload)
        except Exception:
            pass   # silencieux pour éviter de bloquer la sim


# ─────────────────────────────────────────────
#  AGRÉGATEUR
# ─────────────────────────────────────────────
class Aggregator:
    """Reçoit les données du calculateur et les expose au panneau."""

    def __init__(self, afdx: AFDXNetwork, calculator: AvionicsCalculator):
        self.afdx       = afdx
        self.calc       = calculator
        self.last_frame = None

    def update(self):
        frame = self.afdx.receive('A')
        if frame:
            self.last_frame = frame

    def get_display_data(self) -> dict:
        """Renvoie les valeurs à afficher sur le panneau."""
        return {
            "altitude":  round(self.calc.altitude_ft, 0),
            "climb":     round(self.calc.climb_m_min, 1),
            "power":     round(self.calc.motor_power_pct, 1),
            "speed":     round(self.calc.speed_m_s, 2),
            "attack":    round(self.calc.attack_deg, 2),
            "state":     STATE_NAMES[self.calc.state],
            "state_id":  self.calc.state,
            "desired":   round(self.calc.desired_alt_ft, 0),
            "warnings":  self.calc.warnings[:],
            "error":     self.calc.error_msg,
        }


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

        # Thread de simulation
        self._running   = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)

        self._build_ui()
        self._sim_thread.start()
        self._update_ui()

    # ── Build UI ─────────────────────────────
    def _build_ui(self):
        # Titre
        title_bar = tk.Frame(self, bg=COLORS["bg"])
        title_bar.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(title_bar, text="✈  AER8500 — Informatique embarquée de l'avionique",
                 font=("Courier New", 13, "bold"),
                 bg=COLORS["bg"], fg=COLORS["accent"]).pack(side="left")
        tk.Label(title_bar, text="Protocoles : ARINC 429 + AFDX",
                 font=("Courier New", 9),
                 bg=COLORS["bg"], fg=COLORS["text_dim"]).pack(side="right")

        # Séparateur
        sep = tk.Frame(self, bg=COLORS["border"], height=1)
        sep.pack(fill="x", padx=16)

        # Corps principal
        main = tk.Frame(self, bg=COLORS["bg"])
        main.pack(fill="both", expand=True, padx=16, pady=10)

        left  = tk.Frame(main, bg=COLORS["bg"])
        right = tk.Frame(main, bg=COLORS["bg"])
        left.pack(side="left", fill="both", expand=True)
        right.pack(side="right", fill="both", expand=True, padx=(12, 0))

        # ── Colonne gauche ──
        self._build_state_display(left)
        self._build_screens(left)
        self._build_arinc_display(left)

        # ── Colonne droite ──
        self._build_input_panel(right)
        self._build_motor_panel(right)
        self._build_afdx_log(right)
        self._build_altitude_bar(right)

        # ── Barre d'erreur ──
        self.error_bar = tk.Label(self, text="", font=("Courier New", 9, "bold"),
                                  bg="#300", fg=COLORS["red"],
                                  anchor="w", padx=10)
        self.error_bar.pack(fill="x", padx=16, pady=(0, 8))

    def _section(self, parent, title: str) -> tk.Frame:
        """Crée un cadre avec titre."""
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
        """Widget d'affichage style LCD."""
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
        self.scr_speed,  _ = self._screen(row2, "VITESSE (m/s)")
        self.scr_attack, _ = self._screen(row2, "ANGLE ATTAQUE (°)")
        self.scr_desired,_ = self._screen(row2, "ALT. DÉSIRÉE (ft)")

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
            ("Altitude désirée (ft) [0–40000]",   "alt"),
            ("Taux de montée (m/min) [±800]",      "climb"),
            ("Angle d'attaque (°)   [±16]",        "attack"),
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

    # ── Callbacks ────────────────────────────
    def _on_send(self):
        """Lecture et validation des entrées, puis envoi au calculateur."""
        self.calculator.error_msg = ""

        ok_a, alt, err_a = self.calculator.validate_altitude(
            self.input_vars["alt"].get())
        ok_c, climb, err_c = self.calculator.validate_climb(
            self.input_vars["climb"].get())
        ok_k, attack, err_k = self.calculator.validate_attack(
            self.input_vars["attack"].get())

        errors = [e for e in [err_a, err_c, err_k] if e]
        if errors:
            self.error_bar.config(text="  ✘  " + " | ".join(errors),
                                  bg="#300")
            return
        self.error_bar.config(text="", bg=COLORS["bg"])
        self.calculator.set_desired_altitude(alt, climb, attack)

    def _on_land(self):
        """Forcer retour AU_SOL."""
        self.calculator.state        = STATE_AU_SOL
        self.calculator.altitude_ft  = 0.0
        self.calculator.desired_alt_ft = 0.0
        self.calculator.climb_m_min  = 0.0
        self.calculator.attack_deg   = 0.0
        self.calculator.motor_power_pct = 0.0
        self.calculator.error_msg    = ""
        self.power_slider.set(0)
        self.input_vars["alt"].set("")
        self.input_vars["climb"].set("")
        self.input_vars["attack"].set("")
        self.error_bar.config(text="", bg=COLORS["bg"])

    def _on_power_change(self, val):
        self.calculator.set_motor_power(float(val))

    # ── Boucle simulation (thread) ────────────
    def _sim_loop(self):
        while self._running:
            self.calculator.tick(SIM_TICK_S)
            self.aggregator.update()
            time.sleep(SIM_TICK_S)

    # ── Mise à jour UI ────────────────────────
    def _update_ui(self):
        data = self.aggregator.get_display_data()

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

        # Mots ARINC 429
        try:
            alt_i = int(data["altitude"])
            w001  = ARINC429.encode_label001_altitude(alt_i, sid)
            w002  = ARINC429.encode_label002_climb(data["climb"])
            w003  = ARINC429.encode_label003_attack(data["attack"])
            self.arinc_vars["001"].set(ARINC429.word_to_hex(w001))
            self.arinc_vars["002"].set(ARINC429.word_to_hex(w002))
            self.arinc_vars["003"].set(ARINC429.word_to_hex(w003))
        except Exception:
            pass

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
        err = data.get("error", "")
        warns = data.get("warnings", [])
        msg_parts = []
        if err:
            msg_parts.append(err)
        if warns:
            msg_parts.extend(warns)
        if msg_parts:
            self.error_bar.config(text="  ⚠  " + "  |  ".join(msg_parts),
                                  bg="#300")
        elif not self.error_bar["text"].startswith("  ✘"):
            self.error_bar.config(text="", bg=COLORS["bg"])

        # Sync slider avec puissance calculateur
        self.power_slider.set(int(self.calculator.motor_power_pct))

        self.after(100, self._update_ui)

    def _draw_alt_bar(self, event=None):
        c = self.alt_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return
        ratio = self.calculator.altitude_ft / ALT_MAX_FT
        bar_w = int(w * ratio)
        # Fond
        c.create_rectangle(0, 0, w, h, fill="#0a0f15", outline="")
        # Barre
        if bar_w > 0:
            # Dégradé de couleur : vert → jaune → rouge
            if ratio < 0.5:
                r_val = int(255 * ratio * 2)
                g_val = 180
            else:
                r_val = 200
                g_val = int(180 * (1 - ratio) * 2)
            color = f"#{r_val:02x}{g_val:02x}30"
            c.create_rectangle(0, 2, bar_w, h - 2,
                                fill=color, outline="")
        # Texte
        pct = ratio * 100
        c.create_text(w // 2, h // 2,
                      text=f"{self.calculator.altitude_ft:,.0f} ft  ({pct:.1f}%)",
                      fill=COLORS["text"], font=("Courier New", 9, "bold"))
        # Graduation 25 / 50 / 75 %
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