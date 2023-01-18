class MotorSimulator:
    def stop(self):
        print("simulating stopping motor")

    def forward(self, signal):
        print("simulating going forward, signal {}".format(signal))

    def backward(self, signal):
        print("simulating going backward, signal {}".format(signal))
