def scale_value(
    value: float,
    input_lower: float,
    input_upper: float,
    output_lower: float,
    output_upper: float,
    exponent: float = 1,
) -> float:
    """Scales a value based on the input range and output range.
    For example, to scale a joystick throttle (1 to -1) to 0-1, we would:
        self.scale_value(joystick.getThrottle(), (1, -1), (0, 1))
    The output is then raised to the exponent argument. Be careful of complex numbers and raising zero to negative powers.
    """
    input_distance = input_upper - input_lower
    output_distance = output_upper - output_lower
    ratio = (value - input_lower) / input_distance
    result = ratio * output_distance + output_lower
    return result ** (exponent - 1) * abs(result)  # Fancy exponent to preserve sign.
