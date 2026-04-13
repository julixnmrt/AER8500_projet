import sys
sys.stdout.reconfigure(encoding='utf-8')
sys.path.insert(0, '.')

from communication.afdx import AFDXNetwork
from communication.arinc429 import ARINC429
from calculateur import AvionicsCalculator
from aggregateur import Aggregator
from constantes import *

passes = 0
fails = 0

def check(condition, label, detail=""):
    global passes, fails
    status = "PASS" if condition else "FAIL"
    line = f"{label} : {status}"
    if detail:
        line += f"  ({detail})"
    print(line)
    if condition:
        passes += 1
    else:
        fails += 1

def make():
    afdx = AFDXNetwork()
    calc = AvionicsCalculator(afdx)
    agg  = Aggregator(afdx, calc)
    return afdx, calc, agg

def run_to_cruise(calc, agg, max_ticks=40000):
    for _ in range(max_ticks):
        calc.tick(SIM_TICK_S)
        agg.update()
        if calc.state == STATE_VOL_CROISIERE:
            break
    calc.tick(SIM_TICK_S)

def run_ticks(calc, agg, n):
    for _ in range(n):
        calc.tick(SIM_TICK_S)
        agg.update()


print("Phrases formelles :")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(1000.0, 0.0, 0.0)
run_ticks(calc, agg, 10)
d = agg.get_display_data()
check("altitude" in d and "climb" in d and "power" in d,
      "Test phrase formelle 1",
      f"altitude={d['altitude']:.0f} ft  climb={d['climb']:+.1f} m/min  power={d['power']:.1f}%")

check("state" in d and d["state_id"] in (STATE_AU_SOL, STATE_CHANGEMENT, STATE_VOL_CROISIERE),
      "Test phrase formelle 2",
      f"etat={d['state']}")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(2000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
alt_croisiere = calc.altitude_ft
run_ticks(calc, agg, 100)
check(calc.state == STATE_VOL_CROISIERE and calc.climb_m_min == 0.0 and abs(calc.altitude_ft - alt_croisiere) < 1.0,
      "Test phrase formelle 3",
      f"alt={calc.altitude_ft:.1f} ft  climb={calc.climb_m_min:.1f} m/min")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(3000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc.set_desired_altitude(5000.0, 0.0, 0.0)
check(calc.state == STATE_CHANGEMENT,
      "Test phrase formelle 4",
      f"etat={STATE_NAMES[calc.state]}")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(15000.0, 0.0, 0.0)
for _ in range(40000):
    calc.tick(SIM_TICK_S)
    agg.update()
    if calc.altitude_ft >= 5000:
        break
alt_pivot = calc.altitude_ft
calc.set_desired_altitude(4000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
check(abs(calc.altitude_ft - 4000.0) < 2.0 and calc.state == STATE_VOL_CROISIERE,
      "Test phrase formelle 5",
      f"pivot a {alt_pivot:.0f} ft -> arrivee a {calc.altitude_ft:.1f} ft")

_, calc, _ = make()
ok_a, _, err_a = calc.validate_altitude("99999")
ok_c, _, err_c = calc.validate_climb("1500")
ok_k, _, err_k = calc.validate_attack("20")
check(not ok_a and not ok_c and not ok_k,
      "Test phrase formelle 6",
      f"alt:'{err_a}' | climb:'{err_c}' | attack:'{err_k}'")


print("\nContraintes ARINC 429 :")

errors = []
for alt in [0, 1, 999, 1000, 10000, 39999, 40000]:
    w = ARINC429.encode_label001_altitude(alt, 0)
    dec_alt, _ = ARINC429.decode_label001(w)
    if dec_alt != alt:
        errors.append(f"alt={alt} decode={dec_alt}")
check(len(errors) == 0,
      "Test contrainte 1",
      "altitude binaire bits [13:28] resolution 1 ft max 40000 ft")

state_ok = True
for s in [0, 1, 2]:
    w = ARINC429.encode_label001_altitude(1000, s)
    _, dec_s = ARINC429.decode_label001(w)
    if dec_s != s:
        state_ok = False
check(state_ok,
      "Test contrainte 2",
      f"etat bits [11:12] : AU_SOL={ARINC429.word_to_hex(ARINC429.encode_label001_altitude(0,0))}  CHGT={ARINC429.word_to_hex(ARINC429.encode_label001_altitude(0,1))}  CROIS={ARINC429.word_to_hex(ARINC429.encode_label001_altitude(0,2))}")

errors = []
for climb in [0.0, 0.1, 100.0, 499.9, 800.0, -800.0, -123.4]:
    w = ARINC429.encode_label002_climb(climb)
    if abs(ARINC429.decode_label002(w) - climb) > 0.05:
        errors.append(str(climb))
_, calc_max, agg_max = make()
agg_max.set_motor_power(100.0)
calc_max.set_desired_altitude(40000.0, 0.0, 0.0)
run_ticks(calc_max, agg_max, 5)
check(len(errors) == 0 and abs(calc_max.climb_m_min - 800.0) < 1.0,
      "Test contrainte 3",
      f"BCD 4 chiffres resolution 0.1 m/min max={calc_max.climb_m_min:.1f} m/min a 100%")

errors = []
for angle in [0.0, 0.1, 3.0, 8.5, 14.9, -14.9, -9.0, -0.1]:
    w = ARINC429.encode_label003_attack(angle)
    if abs(ARINC429.decode_label003(w) - angle) > 0.05:
        errors.append(str(angle))
check(len(errors) == 0,
      "Test contrainte 4",
      f"BCD 3 chiffres resolution 0.1 deg plage +-15 deg")

afdx5, calc5, agg5 = make()
agg5.set_motor_power(50.0)
calc5.set_desired_altitude(1000.0, 0.0, 0.0)
run_ticks(calc5, agg5, 3)
log = afdx5.get_log(5)
has_state_str = any("AU_SOL" in l or "CHANGEMENT" in l or "VOL_CROISIERE" in l or "VOL_CROISI" in l for l in log)
check(has_state_str,
      "Test contrainte 5",
      "chaine d'etat envoyee via AFDX")


print("\nFormule du taux de montee :")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(1000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
check(calc.climb_m_min == 0.0,
      "Test taux de montee 1",
      f"taux en croisiere = {calc.climb_m_min:.1f} m/min")

results = []
for pwr, expected in [(10, 100), (30, 300), (50, 500), (80, 800)]:
    _, c, a = make()
    a.set_motor_power(float(pwr))
    c.set_desired_altitude(40000.0, 0.0, 0.0)
    run_ticks(c, a, 5)
    results.append((pwr, expected, c.climb_m_min))
all_ok = all(abs(obs - exp) < 1.0 for _, exp, obs in results)
check(all_ok,
      "Test taux de montee 2",
      "  ".join(f"{pwr}%={obs:.0f}m/min" for pwr, _, obs in results))

_, calc, agg = make()
agg.set_motor_power(80.0)
calc.set_desired_altitude(500.0, 0.0, 0.0)
rates = []
for _ in range(5000):
    calc.tick(SIM_TICK_S)
    agg.update()
    if calc.altitude_ft > 400:
        rates.append(calc.climb_m_min)
    if calc.state == STATE_VOL_CROISIERE:
        break
check(len(rates) >= 2 and rates[-1] < rates[0],
      "Test taux de montee 3",
      f"taux a 400ft={rates[0]:.1f} m/min -> a l'arrivee={rates[-1]:.1f} m/min")


print("\nRobustesse :")

_, calc, agg = make()
agg.set_motor_power(50.0)
calc.set_desired_altitude(2000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc.set_desired_altitude(800.0, 0.0, 0.0)
descent_seen = False
for _ in range(20000):
    calc.tick(SIM_TICK_S)
    agg.update()
    if calc.climb_m_min < 0:
        descent_seen = True
    if calc.state == STATE_VOL_CROISIERE:
        break
calc.tick(SIM_TICK_S)
check(descent_seen and abs(calc.altitude_ft - 800.0) < 2.0 and calc.state == STATE_VOL_CROISIERE,
      "Test robustesse 1",
      f"descente a {calc.altitude_ft:.1f} ft")

_, calc, agg = make()
agg.set_motor_power(100.0)
calc.set_desired_altitude(40000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
check(calc.state == STATE_VOL_CROISIERE and abs(calc.altitude_ft - 40000.0) < 2.0,
      "Test robustesse 2",
      f"altitude max 40000 ft -> etat={STATE_NAMES[calc.state]}")

_, calc, agg = make()
agg.set_motor_power(100.0)
calc.set_desired_altitude(5000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc.set_desired_altitude(8000.0, 0.0, 15.0)
calc.tick(SIM_TICK_S)
check(calc._stalling and calc.attack_deg == 15.0 and "DÉCROCHAGE" in calc.error_msg,
      "Test robustesse 3",
      f"decrochage positif angle={calc.attack_deg:.1f} deg")

_, calc, agg = make()
agg.set_motor_power(100.0)
calc.set_desired_altitude(5000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc.set_desired_altitude(3000.0, 0.0, -9.0)
calc.tick(SIM_TICK_S)
check(calc._stalling and calc.attack_deg == -9.0 and "DÉCROCHAGE" in calc.error_msg,
      "Test robustesse 4",
      f"decrochage negatif angle={calc.attack_deg:.1f} deg")

_, calc, agg = make()
agg.set_motor_power(100.0)
calc.set_desired_altitude(2000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc._stalling = True
calc.state = STATE_CHANGEMENT
for _ in range(30000):
    calc.tick(SIM_TICK_S)
    if calc.altitude_ft <= 0.0:
        break
check(calc.altitude_ft == 0.0 and calc.state == STATE_AU_SOL and "IMPACT" in calc.error_msg,
      "Test robustesse 5",
      f"impact au sol -> etat={STATE_NAMES[calc.state]}")

_, calc, agg = make()
agg.set_motor_power(100.0)
calc.set_desired_altitude(5000.0, 0.0, 0.0)
run_to_cruise(calc, agg)
calc._stalling = True
calc.state = STATE_CHANGEMENT
calc.set_desired_altitude(1000.0, 0.0, 0.0)
check("bloquées" in calc.error_msg,
      "Test robustesse 6",
      f"commandes bloquees pendant decrochage")

_, calc, agg = make()
calc.set_desired_altitude(10000.0, 600.0, 0.0)
check(calc.motor_power_pct == 60.0 and calc.state == STATE_CHANGEMENT,
      "Test robustesse 7",
      f"taux 600 m/min en vol -> puissance auto={calc.motor_power_pct:.0f}%")

_, calc, _ = make()
calc.set_desired_altitude(10000.0, 300.0, 0.0)
check(calc.motor_power_pct == 30.0 and calc.state == STATE_CHANGEMENT,
      "Test robustesse 8",
      f"puissance deduite du taux au decollage : 300 m/min -> {calc.motor_power_pct:.0f}%")

_, calc, _ = make()
cases = [("altitude","99999",calc.validate_altitude), ("altitude","-1",calc.validate_altitude),
         ("altitude","abc",calc.validate_altitude), ("climb","900",calc.validate_climb),
         ("climb","-900",calc.validate_climb), ("attack","20",calc.validate_attack),
         ("attack","-20",calc.validate_attack)]
all_rejected = all(not fn(v)[0] for _, v, fn in cases)
check(all_rejected,
      "Test robustesse 9",
      "7 entrees invalides rejetees : " + "  ".join(f"{k}={v}" for k, v, _ in cases))


print("\nRessources systeme :")

try:
    import psutil, os
    proc = psutil.Process(os.getpid())
    cpu  = psutil.cpu_percent(interval=0.2)
    mem  = proc.memory_info().rss / (1024 * 1024)
    check(mem > 0 and cpu >= 0,
          "Test ressources 1",
          f"CPU={cpu:.1f}%  RAM={mem:.1f} MB")
except ImportError:
    check(False, "Test ressources 1", "psutil non installe")


print("\nAFDX et redondance :")

afdx6, calc6, agg6 = make()
agg6.set_motor_power(50.0)
calc6.set_desired_altitude(1000.0, 0.0, 0.0)
run_ticks(calc6, agg6, 3)
frame_a = afdx6.receive('A')
frame_b = afdx6.receive('B')
check(frame_a is not None and frame_b is not None and frame_a["payload"] == frame_b["payload"],
      "Test AFDX 1",
      f"canaux A et B identiques")

afdx7, calc7, agg7 = make()
agg7.set_motor_power(75.0)
frame = afdx7.receive('A')
check(frame["src"] == "PANNEAU" and frame["dst"] == "CALCULATEUR" and frame["payload"]["motor_power"] == 75.0,
      "Test AFDX 2",
      f"puissance transitee par agregateur : src={frame['src']} dst={frame['dst']} power={frame['payload']['motor_power']}%")

afdx8, calc8, agg8 = make()
agg8.set_motor_power(50.0)
calc8.set_desired_altitude(1000.0, 0.0, 0.0)
run_ticks(calc8, agg8, 10)
frame8 = afdx8.receive('A')
p = frame8["payload"]
check("label001" in p and "label002" in p and "label003" in p,
      "Test AFDX 3",
      f"label001={p['label001']}  label002={p['label002']}  label003={p['label003']}")

afdx9, calc9, agg9 = make()
agg9.set_motor_power(50.0)
calc9.set_desired_altitude(5000.0, 0.0, 0.0)
run_to_cruise(calc9, agg9)
d = agg9.get_display_data()
check(abs(d["altitude"] - calc9.altitude_ft) < 1.0 and d["state_id"] == calc9.state,
      "Test AFDX 4",
      f"agregateur synchronise : alt={d['altitude']:.0f} ft  etat={d['state']}")


print(f"\nBilan : {passes}/{passes+fails} tests passes")
