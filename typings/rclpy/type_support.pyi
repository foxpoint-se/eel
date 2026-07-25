from typing import Generic, TypeVar

T = TypeVar("T")

class GetResultServiceResponse(Generic[T]):
    result: T
    status: int
