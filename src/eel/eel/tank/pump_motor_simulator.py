class PumpMotorControlSimulator:
    def __init__(self) -> None:
        self.is_on = False
        self.is_forward = False

    def _set_forward(self):
        self.is_forward = True

    def _set_backward(self):
        self.is_forward = False

    def _start(self):
        self.is_on = True

    def stop(self):
        self.is_on = False

    def fill(self):
        self._set_forward()
        self._start()

    def empty(self):
        self._set_backward()
        self._start()

    def get_is_forward(self):
        return self.is_forward

    def get_is_on(self):
        return self.is_on
