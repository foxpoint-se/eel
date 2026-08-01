from eel.navigation.navigation_pose import has_navigation_pose


def test__when_pose_is_none__should_not_have_navigation_pose() -> None:
    assert not has_navigation_pose(None)


def test__when_pose_is_set__should_have_navigation_pose() -> None:
    assert has_navigation_pose(object())
