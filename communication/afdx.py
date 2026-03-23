from collections import deque
import json
from datetime import datetime

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
