"""Nodo ROS falso para probar los modelos de driver sin ROS ni Gazebo."""


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _add(self, level, msg):
        self.lines.append((level, msg))

    def info(self, msg):
        self._add('info', msg)

    def warn(self, msg):
        self._add('warn', msg)

    def error(self, msg):
        self._add('error', msg)

    def of(self, level):
        return [m for lv, m in self.lines if lv == level]


class FakePublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeTime:
    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class _FakeRosClock:
    def now(self):
        return _FakeTime()


class FakeNode:
    def get_clock(self):
        return _FakeRosClock()

    def __init__(self):
        self.logger = FakeLogger()
        self.pubs = {}
        self.subs = {}
        self.timers = []

    def get_logger(self):
        return self.logger

    def create_publisher(self, _type, topic, _qos):
        return self.pubs.setdefault(topic, FakePublisher())

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs.setdefault(topic, []).append(cb)
        return cb

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return cb

    def send(self, topic, data):
        from std_msgs.msg import Float64
        for cb in self.subs[topic]:
            cb(Float64(data=float(data)))


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t
