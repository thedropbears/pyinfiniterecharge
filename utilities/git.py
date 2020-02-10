import os
import pathlib
import subprocess


def describe() -> str:
    robot_dir = pathlib.Path(__file__).parents[1]
    # would use a hidden file here, but deploy skips dotfiles
    fname = robot_dir / "gitid"
    if os.path.isdir(".git"):
        commit_hash = subprocess.check_output(
            ("git", "describe", "--broken", "--always", "--tags"), text=True
        ).strip()
        ref = subprocess.check_output(
            ("git", "describe", "--all", "--dirty=", "--broken="), text=True
        ).strip()
        desc = f"{commit_hash} ({ref})"
        with open(fname, "w") as f:
            f.write(desc)
        return desc
    with open(fname) as f:
        return f.read()
