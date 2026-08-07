from rclpy.logging import get_logger

logger = get_logger(__name__)


class MotorSimulator:
    def stop(self) -> None:
        logger.debug("Simulating motor stop")

    def forward(self, signal: float) -> None:
        logger.debug("Simulating motor forward, signal=%s", signal)

    def backward(self, signal: float) -> None:
        logger.debug("Simulating motor backward, signal=%s", signal)
