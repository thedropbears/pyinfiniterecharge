import math

from hypothesis import given
from hypothesis.strategies import floats

from utilities.scalers import apply_deadzone, map_exponential


@given(value=floats(-1, 1), threshold=floats(0, 1, exclude_max=True))
def test_deadzone(value, threshold):
    result = apply_deadzone(value, threshold)
    assert abs(result) <= 1
    if value == 0:
        assert result == 0
    elif not math.isclose(value, 0, abs_tol=threshold):
        assert math.copysign(1, result) == math.copysign(1, value)


@given(value=floats(-1, 1), base=floats(1, exclude_min=True, allow_infinity=False))
def test_exponential(value, base):
    result = map_exponential(value, base)
    assert abs(result) <= 1
    if value == 0:
        assert result == 0
    else:
        assert math.copysign(1, result) == math.copysign(1, value)
