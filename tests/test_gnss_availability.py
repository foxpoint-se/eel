from eel_world_sim.gnss_availability import GNSS_UNAVAILABLE_DEPTH_M, should_publish_gnss_fix


def test__when_depth_shallow__should_publish_gnss_fix() -> None:
    assert should_publish_gnss_fix(0.0)
    assert should_publish_gnss_fix(GNSS_UNAVAILABLE_DEPTH_M - 0.01)


def test__when_depth_at_or_below_cutoff__should_not_publish_gnss_fix() -> None:
    assert not should_publish_gnss_fix(GNSS_UNAVAILABLE_DEPTH_M)
    assert not should_publish_gnss_fix(1.0)
