FILLING_UP = "filling up"
EMPTYING = "emptying"
NO_DIRECTION = "no direction"


class PumpStateControl:
    def __init__(self) -> None:
        self._is_running = False
        self._current_direction = NO_DIRECTION

    def stop(self):
        self._is_running = False
        self._current_direction = NO_DIRECTION

    def fill(self):
        self._is_running = True
        self._current_direction = FILLING_UP

    def empty(self):
        self._is_running = True
        self._current_direction = EMPTYING

    def get_is_emptying(self):
        return self._is_running and self._current_direction is EMPTYING

    def get_is_filling_up(self):
        return self._is_running and self._current_direction is FILLING_UP
