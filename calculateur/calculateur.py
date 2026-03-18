class Calculateur:

    def __init__(self):

        self.altitude = 0
        self.state = "AU_SOL"
        self.roc = 0

    def update(self, target_altitude, aoa, power):

        safe = "OK"

        if aoa >= 15:
            safe = "STALL_WARNING"

        if self.state == "AU_SOL":

            if target_altitude > 0:
                self.state = "CHANGEMENT_ALT"

        elif self.state == "CHANGEMENT_ALT":

            self.roc = power * 10

            error = target_altitude - self.altitude

            if abs(error) < 100:
                self.roc *= error / 100

            self.altitude += self.roc * 0.1

            if abs(error) < 10 or self.altitude >= 40000:
                self.state = "VOL_CROISIERE"
                self.roc = 0

        elif self.state == "VOL_CROISIERE":

            self.roc = 0

        return {
            "state": self.state,
            "altitude": round(self.altitude,2),
            "roc": round(self.roc,2),
            "safe": safe
        }