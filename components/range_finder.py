import wpilib


class RangeFinder:
    counter = wpilib.Counter

    def __init__(self):
        self._smoothed_d = 0.0

    def setup(self):
        self.counter.setSemiPeriodMode(highSemiPeriod=True)
        self.counter.setSamplesToAverage(10)

    def _get_distance(self):
        # 10 usec is 1cm, return as metres
        return self.counter.getPeriod() * 1000000 / 1000

    def get_distance(self):
        return self.distance

    def execute(self):
        # get the distance and smooth it
        d = self._get_distance()
        if d > 40:  # Max range is around 40m
            d = 40
        self.distance = d
