import os
import pathlib
import subprocess


def describe() -> str:
    robot_dir = pathlib.Path(__file__).parents[1]
    fname = robot_dir / "gitid"
    if os.path.isdir(".git"):
        desc = subprocess.check_output(
            ("git", "describe", "--broken", "--always", "--tags", "--dirty"),
            encoding="utf-8",
        )
        # would use a hidden file here, but deploy skips dotfiles
        with open(fname, "w") as f:
            f.write(desc)
        return desc
    with open(fname) as f:
        return f.read()
