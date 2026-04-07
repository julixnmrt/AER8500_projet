import tkinter as tk
import threading
import time
from constantes import *
import psutil
import os

from communication.afdx import AFDXNetwork
from calculateur import AvionicsCalculator
from aggregateur import Aggregator

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
        self.title("AER8500 : Simulation Avionique")
        self.configure(bg=COLORS["bg"])
        self.resizable(True, True)
        self.minsize(900, 680)

        self.afdx       = AFDXNetwork()
        self.calculator = AvionicsCalculator(self.afdx)
        self.aggregator = Aggregator(self.afdx, self.calculator)
        self.process = psutil.Process(os.getpid())
        
        self._last_alt_ft: float = 0.0

        # Thread de simulation
        self._running    = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)

        self._build_ui()
        self._sim_thread.start()
        self._update_ui()

        
    def _build_ui(self):
        title_bar = tk.Frame(self, bg=COLORS["bg"])
        title_bar.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(title_bar, text="AER8500 : Informatique embarquée de l'avionique",
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

        # écran ressources système
        row3 = tk.Frame(f, bg=COLORS["panel"])
        row3.pack(fill="x")
        self.scr_cpu, _ = self._screen(row3, "CPU (%)")
        self.scr_ram, _ = self._screen(row3, "RAM (MB)")

    def _build_arinc_display(self, parent):
        f = self._section(parent, "MOTS ARINC 429 (hex)")
        self.arinc_vars = {}
        for label, desc in [("001", "Altitude + État"),
                             ("002", "Taux de montée (BCD) : "),
                             ("003", "Angle d'attaque (BCD) : ")]:
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
        self.calculator.reset()
        self.power_slider.set(0)
        self.input_vars["alt"].set("")
        self.input_vars["climb"].set("")
        self.input_vars["attack"].set("")
        self.error_bar.config(text="", bg=COLORS["bg"])

    def _on_power_change(self, val):
        self.calculator.set_motor_power(float(val))

    def _sim_loop(self):
        while self._running:
            self.calculator.tick(SIM_TICK_S)
            self.aggregator.update()
            time.sleep(SIM_TICK_S)

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

        # Mots ARINC 429 lus depuis les trames décodées
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
            self.error_bar.config(text="  " .join(msg_parts), bg="#300")
        elif not self.error_bar["text"].startswith("  ✘"):
            self.error_bar.config(text="", bg=COLORS["bg"])

        # Sync slider avec puissance calculateur
        self.power_slider.set(int(data["power"]))

        cpu = psutil.cpu_percent(interval=None)
        mem = self.process.memory_info().rss / (1024*1024)

        self.scr_cpu.set(f"{cpu:.1f}")
        self.scr_ram.set(f"{mem:.1f}")
        
        self.after(100, self._update_ui)

    def _draw_alt_bar(self, event=None):
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

if __name__ == "__main__":
    app = AvionicsApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()