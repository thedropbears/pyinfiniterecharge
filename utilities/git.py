import os
import subprocess


def describe() -> str:
    if os.path.isdir(".git"):
        desc = subprocess.check_output(
            ("git", "describe", "--always", "--tags", "--dirty"), encoding="utf-8"
        )
        # would use a hidden file here, but deploy skips dotfiles
        with open("gitid", "w") as f:
            f.write(desc)
        return desc
    else:
        with open("gitid") as f:
            return f.read()
