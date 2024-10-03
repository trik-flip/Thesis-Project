from typing import Callable, Optional, TypeVar

T = TypeVar("T")
IT = TypeVar("IT")
RT = TypeVar("RT")

ACTION = Callable[[T], RT]
ACTION_CREATOR = Callable[[ACTION], ACTION]


def create_collection(
    collection: "dict[IT,ACTION]",
    preprocess: Optional[ACTION] = None,
    check: Optional[Callable[[IT], bool]] = None,
) -> Callable[[IT], ACTION_CREATOR]:
    def register_func(arg: IT) -> ACTION_CREATOR:
        if preprocess is not None:
            arg = preprocess(arg)
        if check is not None:
            assert check(arg)

        def inner_func(func: ACTION) -> ACTION:
            collection[arg] = func
            return func

        return inner_func

    return register_func
