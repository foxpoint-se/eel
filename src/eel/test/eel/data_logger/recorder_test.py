import math
from eel.data_logger.data_recorder import PathRecorder, get_3d_distance
from eel.data_logger.common import Coord3d, Segment, to_coord_3d, LatLon, TimedCoord3d

# kanske två noder??
# en som bara loggar path.
# förenklar saker lite där
# då kan den lyssna på position och djup och simplifiera rutter
#
# den som publicerar kan lyssna på connectivity
# om det finns connectivity
#   kolla om den inte har skickat nåt segment
#       skicka dom
#
# skicka kontinuerligt nya segment
# behöver nog bara lagra tidsstämpel senast den skickade
# när den återfår connectivity, leta upp alla segment som skapats efter senaste tidsstämpel
#   och uppdatera tidsstämpel vid varje skickning
# när den förlorar connectivity
#   skicka inget. tidsstämpel uppdateras inte

# lyssna på record True
# börja spela in då.
# man kan stänga av inspelningen


# varje step
# om den har connectivity
# prova synka
# kan sätta chunk_size_seconds = 2.0
#
# om den inte har connectivity
# ingen synkning
# kan sätta chunk_size_seconds = 10.0

# oavsett
# kalla på step(current_time_seconds)
#   denna gör vadå.. skapar segments. om tid har passerat, simplifiera den och skapa segment.
# kolla om det finns ny segment
# plocka ut den.
#   om connectivity:
#       skicka ut den
#   annars
#       lägg i lista


def print_what_to_publish(segment: Segment) -> None:
    print(f"{segment=}")


def test__should_get_distance_properly() -> None:
    actual_1 = get_3d_distance(Coord3d(x=3, y=0, z=0), Coord3d(x=0, y=4, z=0))
    assert actual_1 == 5
    actual_2 = get_3d_distance(Coord3d(x=0, y=0, z=0), Coord3d(x=1, y=1, z=1))
    expected_2 = 1.732051
    assert math.isclose(a=actual_2, b=expected_2, rel_tol=0.000001)
    actual_3 = get_3d_distance(Coord3d(x=0, y=0, z=0), Coord3d(x=0.5, y=0.5, z=0.5))
    expected_3 = 0.866025
    assert math.isclose(a=actual_3, b=expected_3, rel_tol=0.000001)


def test__when_instantiated__should_not_have_a_position() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )

    assert instance_to_test.last_recorded_3d_position is None


# TODO: remove
def test__when_no_position__should_have_moved_significantly() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    actual = instance_to_test._has_moved_significantly(Coord3d(x=1, y=1, z=1))
    assert actual is True


# TODO: remove
def test__when_moved_distance_less_than_threshold__should_NOT_have_moved_significantly() -> (
    None
):
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.last_recorded_3d_position = Coord3d(x=0, y=0, z=0)
    actual = instance_to_test._has_moved_significantly(Coord3d(x=0.5, y=0.5, z=0.5))
    assert actual is False


# TODO: remove
def test__when_moved_distance_more_than_threshold__should_have_moved_significantly() -> (
    None
):
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.last_recorded_3d_position = Coord3d(x=0, y=0, z=0)
    actual = instance_to_test._has_moved_significantly(Coord3d(x=1, y=1, z=1))
    assert actual is True


def test__when_NOT_enough_time_passed__should_NOT_return_new_segment() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.step(
        new_pos=TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=0)
    )
    instance_to_test.step(
        new_pos=TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=1)
    )
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 0


def test__should_be_one_finalized_segment() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=1, y=1, z=1), created_at=1))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=2, y=2, z=2), created_at=2.01))
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 1


def test__should_be_one_finalized_and_one_in_progress() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=1, y=1, z=1), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=2, y=2, z=2), created_at=1))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=3, y=3, z=3), created_at=1.5))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=2.01))
    actual_finalized = instance_to_test.get_finalized_segments()
    assert len(actual_finalized) == 1
    assert instance_to_test.get_segment_in_progress() is not None


def test__the_finalized_should_have_correct_start_and_finish() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=1, y=1, z=1), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=2, y=2, z=2), created_at=1))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=3, y=3, z=3), created_at=1.5))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=2.01))
    actual_finalized = instance_to_test.get_finalized_segments()
    assert actual_finalized[0]["started_at_seconds"] == 0
    assert actual_finalized[0]["ended_at_seconds"] == 1.5
    assert actual_finalized[0]["polyline"][0]["coord"] == Coord3d(x=1, y=1, z=1)
    assert actual_finalized[0]["polyline"][-1]["coord"] == Coord3d(x=3, y=3, z=3)


def test__finalized_segments_should_be_connected() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=1, y=1, z=1), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=2, y=2, z=2), created_at=1.9))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=3, y=3, z=3), created_at=2.1))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=4, y=4, z=4), created_at=3.8))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=5, y=5, z=5), created_at=4.1))
    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 2


def test__when_segment_without_movement__should_discard_that() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=1, on_publish=print_what_to_publish, seconds_threshold=2.0
    )
    # some movement
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=1, y=1, z=1), created_at=1.9))
    # no movement
    instance_to_test.step(
        TimedCoord3d(coord=Coord3d(x=1.1, y=1.1, z=1.1), created_at=2.01)
    )
    instance_to_test.step(
        TimedCoord3d(coord=Coord3d(x=1.15, y=1.15, z=1.15), created_at=3.9)
    )
    # some movement again
    instance_to_test.step(
        TimedCoord3d(coord=Coord3d(x=1.2, y=1.2, z=1.2), created_at=4.01)
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=4, y=4, z=4), created_at=5.9))

    # another one, so that the previous has the chance to finalize
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=4, y=4, z=4), created_at=6.1))

    actual_finalized = instance_to_test.get_finalized_segments()

    assert len(actual_finalized) == 2


def test__should_be_able_to_change_distance_and_time_threshold() -> None:
    instance_to_test = PathRecorder(
        meters_threshold=2, on_publish=print_what_to_publish, seconds_threshold=10
    )
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=0, y=0, z=0), created_at=0))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=2, y=2, z=2), created_at=9.9))

    # change thresholds
    instance_to_test.set_thresholds(seconds_threshold=2.0, meters_threshold=0.5)

    # more steps
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=3, y=3, z=3), created_at=10.5))
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=4, y=4, z=4), created_at=11.5))

    # one more, to finalize the previous
    instance_to_test.step(TimedCoord3d(coord=Coord3d(x=5, y=5, z=5), created_at=13))

    actual = instance_to_test.get_finalized_segments()
    assert len(actual) == 2
