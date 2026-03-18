from communication.arinc429 import encode_altitude
from communication.afdx import send_status

class Aggregateur:

    def __init__(self, calculator):
        self.calculator = calculator

    def send_inputs(self, altitude, aoa, power):

        # encode ARINC
        arinc_alt = encode_altitude(altitude)

        result = self.calculator.update(altitude,aoa,power)

        # send AFDX message
        send_status(result)

        return result