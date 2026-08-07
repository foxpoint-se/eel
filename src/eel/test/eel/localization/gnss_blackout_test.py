from eel.localization.gnss_blackout import (
    GnssBlackoutConfig,
    clustered_fixes,
    require_valid_gnss_blackout_config,
    trust_state_after_depth_change,
)

TEST_CONFIG = GnssBlackoutConfig(
    enter_depth_m=0.35,
    exit_depth_m=0.15,
    cluster_radius_m=10.0,
    fixes_required=2,
    max_outlier_distance_m=1000.0,
    reacquire_timeout_sec=30.0,
)


def test__when_shallow__should_trust_gnss() -> None:
    assert trust_state_after_depth_change("trusting", 0.10, TEST_CONFIG) == "trusting"


def test__when_deep_enough__should_enter_dead_reckoning() -> None:
    assert trust_state_after_depth_change("trusting", 0.35, TEST_CONFIG) == "dead_reckoning"


def test__when_surfacing__should_enter_reacquiring() -> None:
    assert trust_state_after_depth_change("dead_reckoning", 0.15, TEST_CONFIG) == "reacquiring"


def test__when_resubmerging_during_reacquire__should_return_to_dead_reckoning() -> None:
    assert trust_state_after_depth_change("reacquiring", 0.35, TEST_CONFIG) == "dead_reckoning"


def test__when_two_fixes_agree__should_return_cluster_centroid() -> None:
    fixes = [{"lat": 0.0, "lon": 0.0}, {"lat": 0.0, "lon": 0.0000001}]
    accepted = clustered_fixes(fixes, TEST_CONFIG)
    assert accepted is not None
    assert accepted["lat"] == 0.0


def test__when_fixes_disagree__should_not_cluster() -> None:
    fixes = [{"lat": 0.0, "lon": 0.0}, {"lat": 1.0, "lon": 1.0}]
    assert clustered_fixes(fixes, TEST_CONFIG) is None


def test__when_fixes_required_is_zero__should_not_cluster() -> None:
    invalid_config = GnssBlackoutConfig(
        enter_depth_m=0.35,
        exit_depth_m=0.15,
        cluster_radius_m=10.0,
        fixes_required=0,
        max_outlier_distance_m=1000.0,
        reacquire_timeout_sec=30.0,
    )
    assert clustered_fixes([{"lat": 0.0, "lon": 0.0}], invalid_config) is None


def test__when_exit_depth_exceeds_enter_depth__should_raise() -> None:
    config = GnssBlackoutConfig(
        enter_depth_m=0.15,
        exit_depth_m=0.35,
        cluster_radius_m=5.0,
        fixes_required=2,
        max_outlier_distance_m=1000.0,
        reacquire_timeout_sec=30.0,
    )
    try:
        require_valid_gnss_blackout_config(config)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass


def test__when_reacquire_timeout_is_negative__should_raise() -> None:
    config = GnssBlackoutConfig(
        enter_depth_m=0.35,
        exit_depth_m=0.15,
        cluster_radius_m=5.0,
        fixes_required=2,
        max_outlier_distance_m=1000.0,
        reacquire_timeout_sec=-1.0,
    )
    try:
        require_valid_gnss_blackout_config(config)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass
