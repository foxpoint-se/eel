from eel.depth_control.pid_merger import PidMerger


def test__when_args_empty__should_return_none():
    merger = PidMerger()
    actual = merger.merge()
    assert actual.front is None
    assert actual.rear is None
