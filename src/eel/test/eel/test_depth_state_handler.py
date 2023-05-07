from eel.depth_control.depth_state_handler import (
    DepthController,
    TargetState,
    ProgressState,
)
from eel.depth_control.target_checker import TargetChecker
from eel.depth_control.base import DepthConfig, EelDepthState, TankLevels
from eel.depth_control.pid_merger import MergerInterface
from eel.depth_control.pid_controller import PidInterface


class MockTargetChecker(TargetChecker):
    def __init__(self, at_target: bool) -> None:
        super().__init__()
        self.__at_target = at_target

    def is_at_target(self, value: float = None) -> bool:
        return self.__at_target

    def update_target(self, value: float = None) -> None:
        return


class MockMerger(MergerInterface):
    def __init__(self) -> None:
        super().__init__()

    def merge(
        self, depth_ctrl: TankLevels, pitch_ctrl: TankLevels
    ) -> TankLevels:
        merged_front = depth_ctrl.front + pitch_ctrl.front
        merged_rear = depth_ctrl.rear + pitch_ctrl.rear
        new_tank_levels = TankLevels(front=merged_front, rear=merged_rear)
        return new_tank_levels


class MockPid(PidInterface):
    def __init__(self, computed: float = None) -> None:
        super().__init__()
        self.__computed = computed

    def compute(self, system_current_value: float) -> float:
        return self.__computed

    def update_target(self, set_point: float = None) -> None:
        return


def create_target_checker(at_target: bool) -> MockTargetChecker:
    return MockTargetChecker(at_target=at_target)


def create_pid(computed: float) -> MockPid:
    return MockPid(computed)


is_good = create_target_checker(True)
is_bad = create_target_checker(False)
merge_add = MockMerger()
compute_to_one_pid = create_pid(computed=1)


def test__should_init():
    actual = DepthController()
    assert actual is not None


def test__when_init__should_have_no_target_and_no_action():
    actual = DepthController()
    assert actual.get_target_state() is TargetState.NO_TARGET
    assert actual.get_progress_state() is ProgressState.NO_ACTION


def test__when_init__should_not_have_next_state():
    dsh = DepthController()
    actual = dsh.update(current_state=None)
    assert actual is None


def test__when_target_is_set_to_none__should_return_none():
    dsh = DepthController(
        depth_target_checker=is_good,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_good,
        pitch_velocity_target_checker=is_good,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(None)
    not_important_state = None
    actual = dsh.update(not_important_state)
    assert actual is None


def test__when_target_is_surface_and_tanks_are_empty__should_return_none():
    dsh = DepthController(
        depth_target_checker=is_good,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_good,
        pitch_velocity_target_checker=is_good,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(DepthConfig(depth=0.0))
    basically_empty = 0.04
    next_levels = dsh.update(
        EelDepthState(
            front_tank_level=basically_empty,
            rear_tank_level=basically_empty,
        )
    )
    assert next_levels is None


def test__when_target_surface_and_tanks_not_empty__should_return_zeros():
    dsh = DepthController(
        depth_target_checker=is_good,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_good,
        pitch_velocity_target_checker=is_good,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(DepthConfig(depth=0.0))
    not_empty = 0.05
    next_levels = dsh.update(
        EelDepthState(
            front_tank_level=not_empty,
            rear_tank_level=not_empty,
        )
    )
    assert next_levels.front == 0.0
    assert next_levels.rear == 0.0


def test__when_target_depth_and_not_at_target__should_not_return_none():
    dsh = DepthController(
        merger=merge_add,
        depth_target_checker=is_bad,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_good,
        pitch_velocity_target_checker=is_good,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(DepthConfig(depth=2.0))
    next_levels = dsh.update(EelDepthState(depth=0.0))
    assert next_levels is not None


def test__when_all_targets_good__should_return_none():
    dsh = DepthController(
        depth_target_checker=is_good,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_good,
        pitch_velocity_target_checker=is_good,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(DepthConfig(depth=2.0))
    next_levels = dsh.update(EelDepthState(depth=2.0))
    assert next_levels is None


def test__when_at_target_depth_but_high_velocity__should_not_return_none():
    dsh = DepthController(
        depth_target_checker=is_good,
        pitch_target_checker=is_good,
        depth_velocity_target_checker=is_bad,
        merger=merge_add,
        depth_pid=compute_to_one_pid,
        pitch_pid=compute_to_one_pid,
    )
    dsh.set_new_target(DepthConfig(depth=2.0))
    next_levels = dsh.update(EelDepthState(depth=2.0))
    assert next_levels is not None


# class TestDepthStateHandler(unittest.TestCase):
#     def setUp(self) -> None:
#         print("setup is called")
#         self.mock1 = Mock()
#         self.depth_target_checker = Mock(spec=TargetChecker)
#         self.depth_target_checker.is_at_target = Mock()
#         self.depth_target_checker.is_at_target.return_value = True
#         # self.depth_target_checker

#     def tearDown(self) -> None:
#         print("teardown is called")
#         self.mock1.reset_mock()
#         self.depth_target_checker.reset_mock()

#     # def test__bajs(self):
#     #     assert True is True

#     # def test__bajs__2(self):
#     #     assert True is True

#     # @patch.object(DepthTargetChecker, "is_at_target")
#     def test__when_target_depth_and_not_at_target__should_not_return_none(
#         self,
#     ):
#         # mock_depth_is_at_target.return_value = True
#         dsh = DepthController(depth_target_checker=self.depth_target_checker)
#         dsh.set_new_target(DepthConfig(depth=2.0))
#         next_levels = dsh.update(EelDepthState(depth=0.0))
#         assert next_levels.front > 0.0
#         assert next_levels.rear > 0.0
