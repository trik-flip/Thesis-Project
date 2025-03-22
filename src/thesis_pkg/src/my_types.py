from enum import Flag, auto


class Axis(Flag):
    """The type of direction"""

    def __len__(self) -> int:
        counter = 0
        for _value, flag in enumerate(Flag):
            if flag in self:  # type: ignore
                counter += 1
        return counter

    def get_base_axis(self):
        for _value, flag in enumerate(Flag):
            if flag in self:  # type: ignore
                yield flag

    X = auto()
    """This is the front and back direction"""
    Y = auto()
    """This is the sideway direction"""
    Z = auto()
    """This is the up and down direction"""
