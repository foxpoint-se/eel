from eel.data_logger.data_recorder import PathRecorder
from eel.data_logger.common import Coord3d, TimedCoord3d, Segment

from geopy import distance


def add_meters(coord: Coord3d, meters_to_add: float) -> Coord3d:
    bearing = 45
    new_position = distance.distance(meters=meters_to_add).destination(
        (coord["x"], coord["y"]),
        bearing=bearing,
    )
    return Coord3d(x=new_position.latitude, y=new_position.longitude, z=coord["z"])


origin = Coord3d(x=59.309395, y=17.974279, z=0)
origin_plus1 = add_meters(origin, 1.1)
origin_plus2 = add_meters(origin, 2.1)
origin_plus3 = add_meters(origin, 3.1)
origin_plus4 = add_meters(origin, 4.1)
origin_plus5 = add_meters(origin, 5.1)
origin_plus6 = add_meters(origin, 6.1)
origin_plus7 = add_meters(origin, 7.1)
origin_plus8 = add_meters(origin, 8.1)
origin_plus9 = add_meters(origin, 9.1)


def print_what_to_publish(segment: Segment) -> None:
    print(f"{segment=}")


def test__when_instantiated__should_not_have_a_position() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )

    assert instance_to_test.last_recorded_3d_position is None


def test__when_not_enough_time_passed__should_not_return_new_segment() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(new_pos=TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(new_pos=TimedCoord3d(coord=origin_plus6, created_at=1))
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 0


def test__when_enough_time_and_distance__should_be_one_finalized_segment() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=1))
    instance_to_test.step(TimedCoord3d(coord=origin_plus6, created_at=2.01))
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 1


def test__when_enough_time_and_distance__should_be_one_finalized_and_one_in_progress() -> (
    None
):
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=1))
    instance_to_test.step(TimedCoord3d(coord=origin_plus2, created_at=1.5))
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=2.01))
    actual_finalized = instance_to_test.get_finalized_segments()
    assert len(actual_finalized) == 1
    assert instance_to_test.get_segment_in_progress() is not None


def test__when_finalized__segment_should_have_correct_start_and_finish() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=1))
    instance_to_test.step(TimedCoord3d(coord=origin_plus2, created_at=1.5))
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=2.01))
    actual_finalized = instance_to_test.get_finalized_segments()
    assert actual_finalized[0]["started_at_seconds"] == 0
    assert actual_finalized[0]["ended_at_seconds"] == 1.5
    assert actual_finalized[0]["polyline"][0]["coord"] == origin
    assert actual_finalized[0]["polyline"][-1]["coord"] == origin_plus2


def test__when_two_finalized_segments__should_be_connected() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=1.9))
    instance_to_test.step(TimedCoord3d(coord=origin_plus2, created_at=2.1))
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=3.8))
    instance_to_test.step(TimedCoord3d(coord=origin_plus4, created_at=4.1))
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 2


def test__when_segment_without_movement__should_discard_that() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, seconds_threshold=2.0, on_new_segment=print_what_to_publish
    )
    # some movement
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=1.9))
    # no movement
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=2.01))
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=3.9))
    # some movement again
    instance_to_test.step(TimedCoord3d(coord=origin_plus1, created_at=4.01))
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=5.9))

    # another one, so that the previous has the chance to finalize
    instance_to_test.step(TimedCoord3d(coord=origin_plus4, created_at=6.1))

    actual_finalized = instance_to_test.get_finalized_segments()

    assert len(actual_finalized) == 2


def test__should_be_able_to_change_distance_and_time_threshold() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=2, seconds_threshold=10, on_new_segment=print_what_to_publish
    )
    instance_to_test.step(TimedCoord3d(coord=origin, created_at=0))
    instance_to_test.step(TimedCoord3d(coord=origin_plus2, created_at=9.9))

    # change thresholds
    instance_to_test.set_thresholds(seconds_threshold=2.0, meters_threshold=0.5)

    # more steps
    instance_to_test.step(TimedCoord3d(coord=origin_plus2, created_at=10.5))
    instance_to_test.step(
        TimedCoord3d(coord=add_meters(origin_plus2, 0.52), created_at=11.5)
    )

    # one more, to finalize the previous
    instance_to_test.step(TimedCoord3d(coord=origin_plus3, created_at=13))

    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 2
