import os
from typing import Iterable

__all__ = ["write_obj_file", "write_multi_part_obj_file"]


def write_obj_file(
    points: "Iterable[tuple[float,float,float]]", file_name: str = "tmp.obj"
) -> None:
    """Write an object consisting of points to a `obj` file"""
    assert ".obj" in file_name, f"Needs to be an `obj` file, got {file_name}"
    with open(file_name, "w") as f:
        f.write("o obj_0\n\n")
        for x, y, z in points:
            line = f"v {x}\t{y}\t{z}\n"
            f.write(line)


def write_multi_part_obj_file(
    points: "Iterable[Iterable[tuple[float,float,float]]]",
    file_name: str = "tmp_{i}.obj",
    folder_name: str = "multi_tmp",
) -> None:
    """
    Write an object composed from multiple convex objects
    """
    assert not os.path.exists(folder_name), "Folder already exists"
    assert "{i}" in file_name, f"Need an indexer, got `{file_name}`"
    assert ".obj" in file_name, f"Needs to be an `obj` file, got {file_name}"
    os.mkdir(folder_name)
    assert os.path.exists(folder_name), "Coulnd't create folder"
    for i, part in enumerate(points):
        complete_file_name = folder_name + "/" + file_name.format(i=i)
        assert not os.path.exists(
            complete_file_name
        ), f"{complete_file_name} already exists"
        write_obj_file(part, complete_file_name)
        assert os.path.exists(
            complete_file_name
        ), f"Couldn't create {complete_file_name}"
