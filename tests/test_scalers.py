import math

from hypothesis import assume, given
from hypothesis.strategies import floats

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


reals = floats(allow_nan=False, allow_infinity=False)


@given(
    value=reals,
    input_lower=reals,
    input_upper=reals,
    output_lower=floats(-24, 24),
    output_upper=floats(-24, 24),
    exponent=floats(0, 16, exclude_min=True),
)
def test_scale_value(
    value: float,
    input_lower: float,
    input_upper: float,
    output_lower: float,
    output_upper: float,
    exponent: float,
):
    assume(input_lower != input_upper)
    assume(min(input_lower, input_upper) <= value <= max(input_lower, input_upper))
    assume(output_lower < output_upper)
    scale_value(value, input_lower, input_upper, output_lower, output_upper, exponent)
