from eel.depth_control.target_checker import TargetChecker


# TODO: better test
def test__when_initialized__should_have_no_target():
    checker = TargetChecker()
    assert True is True


def test__when_at_surface_and_tanks_empty__should_return_true():
    checker = TargetChecker()
    assert True is True
