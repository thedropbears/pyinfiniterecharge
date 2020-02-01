# pyinfiniterecharge
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)

The Drop Bears' robot code for FIRST Infinite Recharge (FRC 2020).

## Install dependencies

    pip3 install -r requirements.txt

## Code style
This codebase adheres to the code style enforced by the black autoformatter:

    black .

This is enforced by CI. To install this:

    pip3 install black

See [PEP 8](https://www.python.org/dev/peps/pep-0008/) on naming conventions.

[Docstrings should follow Google style](https://google.github.io/styleguide/pyguide.html#383-functions-and-methods).
See also [PEP 257](https://www.python.org/dev/peps/pep-0257/).

## Checks
Install the linters we're using with:

    pip3 install -r requirements-lint.txt

Ensure code passes pyflakes, a static analysis checker:

    pyflakes .

This is enforced by CI.

## Run

### Simulation (desktop)

    ./robot.py sim

### Deploy to robot

    ./robot.py deploy

This project is configured to automatically deploy to 4774's robot.
