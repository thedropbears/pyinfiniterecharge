import math
from typing import Tuple

from hypothesis import assume, given
from hypothesis.strategies import floats, tuples

from utilities.scalers import apply_deadzone, map_exponential, scale_value


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


real_halves = floats(allow_nan=False, allow_infinity=False, width=16)


@given(
    value=real_halves,
    input_range=tuples(real_halves, real_halves).filter(lambda x: x[0] != x[1]),
    output_range=tuples(real_halves, real_halves)
    .map(sorted)
    .filter(lambda x: x[0] < x[1]),
)
def test_scale_value(
    value: float,
    input_range: Tuple[float, float],
    output_range: Tuple[float, float],
):
    input_lower, input_upper = input_range
    output_lower, output_upper = output_range
    assume(min(input_lower, input_upper) <= value <= max(input_lower, input_upper))
    result = scale_value(value, input_lower, input_upper, output_lower, output_upper)
    assert output_lower <= result <= output_upper
