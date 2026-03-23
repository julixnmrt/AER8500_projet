from communication.afdx import AFDXNetwork
from communication.arinc429 import ARINC429
from calculateur import AvionicsCalculator
from constantes import *
import threading

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