from .utils import to_json_filtered, instance_to_str
from .eel_side import (
    EelState,
    from_json_to_state,
    CommandMessage,
    from_json_to_command,
)
from .client_side import ClientEelState

# TODO: fix modules or whatever, so that this file can be run directly, with relative imports working


class ExampleRosCoordinateMsg:
    def __init__(self) -> None:
        self.lat = 59.30940628051758
        self.lon = 17.974245071411133

    def __str__(self) -> str:
        return instance_to_str(self)


class ExampleRosImuMsg:
    def __init__(self) -> None:
        self.is_calibrated = True
        self.sys = 3
        self.gyro = 3
        self.accel = 3
        self.mag = 3
        self.euler_heading = 180.0

    def __str__(self) -> str:
        return instance_to_str(self)


class ExampleRosGnssMsg:
    def __init__(self) -> None:
        coordinate = ExampleRosCoordinateMsg()
        self.lat = coordinate.lat
        self.lon = coordinate.lon

    def __str__(self) -> str:
        return instance_to_str(self)


class ExampleNavMsg:
    def __init__(self) -> None:
        self.meters_to_target = 150.0
        self.tolerance_in_meters = 5.0
        self.next_target = [ExampleRosCoordinateMsg()]
        self.auto_mode_enabled = True

    def __str__(self) -> str:
        return instance_to_str(self)


if __name__ == "__main__":
    print()
    print("=== CONVERT FROM RADIO MESSAGE TO COMMAND ===")
    print()
    incoming_cmd_obj = CommandMessage(m=1.0)
    print(">> initialize incoming cmd obj")
    print(incoming_cmd_obj)
    print()

    incoming_cmd_str = to_json_filtered(incoming_cmd_obj)
    print(">> get this over radio")
    print(incoming_cmd_str)
    print()

    incoming_cmd_converted = from_json_to_command(incoming_cmd_str)
    print(">> convert to command")
    print(incoming_cmd_converted)
    print()

    print(">> DONE!")
    print()
    print()
    print("=== UPDATE STATE AND CONVERT TO RADIO MESSAGE ===")
    print()

    current_state = EelState()
    print(">> empty state")
    print(current_state)
    print()

    ros_imu = ExampleRosImuMsg()
    ros_gnss = ExampleRosGnssMsg()
    current_state.update_gnss(ros_gnss)
    current_state.update_imu(ros_imu)
    print(">> partially update state with")
    print(ros_gnss)
    print(ros_imu)
    print()
    print(">> updated state")
    print(current_state)
    print()

    radio_msg_out = to_json_filtered(current_state)
    print(">> send this")
    print(radio_msg_out)
    print()

    ros_nav = ExampleNavMsg()
    current_state.update_nav(ros_nav)
    print(">> update state again with")
    print(ros_nav)
    print(">> updated state")
    print(current_state)

    print()
    radio_msg_out_2 = to_json_filtered(current_state)
    print(">> send this")
    print(radio_msg_out_2)
    print()

    print(">> DONE!")
    print()
    print()
    print("=== CONVERT FROM STATE RADIO MESSAGE TO CLIENT STATE ===")
    print()

    client_state = ClientEelState()
    print(">> initial client state")
    print(client_state)
    print()

    print(">> getting this")
    print(radio_msg_out)
    print()

    incoming_state = from_json_to_state(radio_msg_out)
    print(">> converted")
    print(incoming_state)
    print()

    print(">> update client state")
    client_state.update_eel_state(incoming_state)
    print(client_state)
    print()

    print(">> and then getting this")
    print(radio_msg_out_2)
    print()

    incoming_state_2 = from_json_to_state(radio_msg_out_2)
    print(">> converted")
    print(incoming_state_2)
    print()

    print(">> update client state again")
    client_state.update_eel_state(incoming_state_2)
    print(client_state)
    print()

    print(">> DONE!")
    print()
    print("========================================================")
