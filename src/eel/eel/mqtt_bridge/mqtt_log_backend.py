import json
from typing import Mapping, Protocol

DEFAULT_ROBOT_NAME = "robot"


class MqttBackend(Protocol):
    robot_name: str

    def connect(self) -> None:
        pass

    def publish(self, topic: str, message: Mapping[str, object]) -> None:
        pass

    def subscribe(self, topic: str, callback: object) -> None:
        pass


class _Logger(Protocol):
    def info(self, msg: str) -> None:
        pass

    def debug(self, msg: str) -> None:
        pass


class LoggingMqttBackend:
    def __init__(self, logger: _Logger, robot_name: str = DEFAULT_ROBOT_NAME) -> None:
        self.robot_name = robot_name
        self._logger = logger

    def connect(self) -> None:
        self._logger.info(f"MQTT log backend ready for {self.robot_name}")

    def publish(self, topic: str, message: Mapping[str, object]) -> None:
        self._logger.debug(f"MQTT log publish {topic}: {json.dumps(message)}")

    def subscribe(self, topic: str, callback: object) -> None:
        self._logger.debug(f"MQTT log subscribe {topic}")
