from typing import Callable, Generic, TypeVar

T = TypeVar("T")

class Future(Generic[T]):
    def result(self) -> T: ...
    def add_done_callback(self, callback: Callable[..., None]) -> None: ...
