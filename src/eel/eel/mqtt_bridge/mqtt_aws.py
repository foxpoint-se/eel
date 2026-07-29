import json
from typing import Mapping, TypedDict

from awscrt import mqtt
from awsiot import mqtt_connection_builder
from rclpy.logging import get_logger

logger = get_logger(__name__)


class CertData(TypedDict):
    endpoint: str
    port: int
    certificatePath: str
    privateKeyPath: str
    rootCAPath: str
    clientID: str


class AwsIotMqttBackend:
    def __init__(self, cert_data: CertData) -> None:
        self.robot_name = cert_data["clientID"]
        self._cert_data = cert_data
        self._conn: mqtt.Connection | None = None

    def connect(self) -> None:
        cert_data = self._cert_data
        logger.info(f"Connecting directly to endpoint {cert_data['endpoint']}")
        self._conn = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port=cert_data["port"],
            cert_filepath=cert_data["certificatePath"],
            pri_key_filepath=cert_data["privateKeyPath"],
            ca_filepath=cert_data["rootCAPath"],
            client_id=cert_data["clientID"],
            http_proxy_options=None,
        )
        connected_future = self._conn.connect()
        connected_future.result()
        logger.info("Connected!")

    def publish(self, topic: str, message: Mapping[str, object]) -> None:
        if self._conn is None:
            return
        json_payload = json.dumps(message)
        self._conn.publish(topic=topic, payload=json_payload, qos=mqtt.QoS.AT_LEAST_ONCE)

    def subscribe(self, topic: str, callback: object) -> None:
        if self._conn is None:
            return
        self._conn.subscribe(
            topic=topic,
            qos=mqtt.QoS.AT_LEAST_ONCE,
            callback=callback,
        )
        logger.info(f"Subscribed to MQTT topic {topic}")
