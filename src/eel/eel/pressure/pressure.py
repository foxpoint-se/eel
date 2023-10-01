from typing import List
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from eel_interfaces.msg import ImuStatus



class Logger:
    def debug(self, message: str, **kwargs) -> None:
        pass

    def info(self, message: str, **kwargs) -> None:
        pass

    def error(self, message: str, **kwargs) -> None:
        pass


class RosHandler:
    def get_logger(self) -> Logger:
        pass


class Pressure:
    def __init__(
        self,
    ) -> None:
        pass


class PressureControl:
    pass


# ===========


class PressureNode(Node):
    def __init__(
        self,
        node_name: str,
        *,
        context: Context = None,
        cli_args: List[str] = None,
        namespace: str = None,
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: List[Parameter] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides,
        )



        imu_sub = self.create_subscription(ImuStatus, "imu/status", self.handle_imu_msg, 10)

        self.controller = PressureControl()

    def handle_imu_msg(self, msg):
        pass

    def receive_imu_msg(self, msg: ImuStatus):
        return {"pitch": msg}

    def


