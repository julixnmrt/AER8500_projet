class ControlPanel:

    def __init__(self, aggregator):
        self.aggregator = aggregator

    def run(self):

        print("=== AVIONICS CONTROL PANEL ===")

        while True:

            altitude_input = input("Desired altitude (ft) (q to quit): ")

            if altitude_input.lower() == "q":
                print("Stopping system...")
                break

            try:
                altitude = float(altitude_input)
                aoa = float(input("Angle of attack (deg): "))
                power = float(input("Engine power (%): "))

                result = self.aggregator.send_inputs(
                    altitude,
                    aoa,
                    power
                )

                print("\n--- SYSTEM STATE ---")
                print(result)

            except ValueError:
                print("Invalid input")