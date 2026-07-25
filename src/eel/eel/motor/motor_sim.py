class MotorSimulator:
    def stop(self) -> None:
        print("simulating stopping motor")

    def forward(self, signal: float) -> None:
        print("simulating going forward, signal {}".format(signal))

    def backward(self, signal: float) -> None:
        print("simulating going backward, signal {}".format(signal))
