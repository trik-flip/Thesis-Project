import mujoco


def get_flags() -> "list[str]":
    flags: dict[int, str] = {}
    a = [x for x in dir(mujoco.mjtVisFlag) if "mjVIS_" in x]
    for x in a:
        for i in range(len(a)):
            if eval(f"{i} == mujoco.mjtVisFlag." + x):
                flags[i] = x
    return [value for _key, value in sorted(flags.items())]
