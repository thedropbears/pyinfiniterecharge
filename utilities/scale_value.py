from typing import Tuple


def scale_value(
    value: float,
    input_range: Tuple[float, float],
    output_range: Tuple[float, float],
    exponent: float = 1,
) -> float:
    """Scales a value based on the input range and output range.
    For example, to scale a joystick throttle (1 to -1) to 0-1, we would:
        self.scale_value(joystick.getThrottle(), (1, -1), (0, 1))
    The output is then raised to the exponent argument. Be careful of complex numbers and raising zero to negative powers.
    """
    input_distance = input_range[1] - input_range[0]
    output_distance = output_range[1] - output_range[0]
    ratio = (value - input_range[0]) / input_distance
    result = ratio * output_distance + output_range[0]
    return result ** (exponent - 1) * abs(result)  # Fancy exponent to preserve sign.
