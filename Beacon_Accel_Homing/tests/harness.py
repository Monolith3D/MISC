"""Load beacon_accel_homing with stub Klipper objects, for offline tests and replay."""
import collections
import importlib.util
import struct
import sys
import types
from pathlib import Path

PLUGIN = Path(__file__).resolve().parent.parent / "beacon_accel_homing.py"
MG_PER_COUNT_16G = 7.8125     # Beacon RevH 16 g scale
CLIP_COUNTS = 2047            # BEACON_ACCEL_BITS 12
BEACON_HZ = 1000.0            # fake Beacon clock: 1 tick per ms


class Sent:
    def __init__(self):
        self.messages = []

    def send(self, args):
        self.messages.append(list(args))


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


def _stub_mcu():
    module = types.ModuleType("mcu")

    class MCU_trsync:
        REASON_ENDSTOP_HIT = 1
        REASON_HOST_REQUEST = 2
        REASON_PAST_END_TIME = 3
        REASON_COMMS_TIMEOUT = 4

    class Trsync:
        def __init__(self, mcu, oid):
            self.mcu, self.oid = mcu, oid

        def get_mcu(self):
            return self.mcu

        def get_oid(self):
            return self.oid

    class TriggerDispatch:
        def __init__(self, mcu):
            self._mcu = mcu
            self._trsyncs = [Trsync(mcu, 7)]
            self.steppers = []
            self.result = MCU_trsync.REASON_ENDSTOP_HIT
            self.started = []

        def get_oid(self):
            return 7

        def add_stepper(self, stepper):
            if stepper.get_mcu() not in [t.mcu for t in self._trsyncs]:
                self._trsyncs.append(Trsync(stepper.get_mcu(), 7 + len(self._trsyncs)))
            self.steppers.append(stepper)

        def get_steppers(self):
            return list(self.steppers)

        def start(self, print_time):
            self.started.append(print_time)
            return "completion"

        def wait_end(self, end_time):
            pass

        def stop(self):
            return self.result

    module.MCU_trsync = MCU_trsync
    module.TriggerDispatch = TriggerDispatch
    return module


def load_plugin():
    sys.modules["mcu"] = _stub_mcu()
    spec = importlib.util.spec_from_file_location("beacon_accel_homing", PLUGIN)
    plugin = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plugin)
    return plugin


plugin = load_plugin()


class FakeReactor:
    NOW = 0.0
    NEVER = 9e9

    def __init__(self):
        self.now = 0.0
        self.timers = {}
        self.on_pause = None

    def monotonic(self):
        return self.now

    def register_timer(self, callback, when=NEVER):
        handle = object()
        self.timers[handle] = when
        return handle

    def update_timer(self, handle, when):
        self.timers[handle] = when

    def pause(self, when):
        self.now = when
        if self.on_pause is not None:
            self.on_pause()


class FakeConfig:
    error = ConfigError

    def __init__(self, printer, options, sections=("beacon",)):
        self.printer, self.options, self.sections = printer, dict(options), sections
        self.deprecated = []

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "beacon_accel_homing"

    def has_section(self, name):
        return name in self.sections

    def deprecate(self, option):
        self.deprecated.append(option)

    def get(self, name, default=None):
        return self.options.get(name, default)

    def _number(self, kind, name, default, minval=None, maxval=None):
        value = kind(self.options.get(name, default))
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise ConfigError("%s out of range" % name)
        return value

    def getfloat(self, name, default=None, minval=None, maxval=None):
        return self._number(float, name, default, minval, maxval)

    def getint(self, name, default=None, minval=None, maxval=None):
        return self._number(int, name, default, minval, maxval)


class BareToolhead:
    """A toolhead with no way to change the acceleration."""
    def __init__(self, accel=20000.0, kin=None):
        self.accel = accel
        self.accel_log = []
        self.dwells = []
        self.moves = []
        self.kin = kin

    def get_max_velocity(self):
        return 500.0, self.accel

    def dwell(self, delay):
        self.dwells.append(delay)

    def get_kinematics(self):
        return self.kin

    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))


class FakeToolhead(BareToolhead):
    """Klipper since 2024."""
    def set_max_velocities(self, velocity, accel, scv, mcr):
        if accel is not None:
            self.accel = accel
            self.accel_log.append(accel)


class KalicoToolhead(BareToolhead):
    """Kalico's toolhead: set_accel instead of set_max_velocities."""
    def set_accel(self, accel):
        self.accel = accel
        self.accel_log.append(accel)


class OldToolhead(BareToolhead):
    """Klipper before 2024: only the max_accel attribute and its recalculation."""
    @property
    def max_accel(self):
        return self.accel

    @max_accel.setter
    def max_accel(self, accel):
        self.accel = accel
        self.accel_log.append(accel)

    def _calc_junction_deviation(self):
        pass


class FakeConfigfile:
    def __init__(self):
        self.saved = {}
        self.warnings = []

    def set(self, section, option, value):
        self.saved[(section, option)] = value

    def runtime_warning(self, msg):
        self.warnings.append(msg)


class FakePrinter:
    config_error = ConfigError
    command_error = CommandError

    def __init__(self, toolhead, beacon):
        self.reactor = FakeReactor()
        self.handlers = collections.defaultdict(list)
        self.commands = {}
        self.chips = {}
        self.scripts = []
        self.objects = {
            "toolhead": toolhead, "beacon": beacon, "configfile": FakeConfigfile(),
            "mcu": types.SimpleNamespace(estimated_print_time=lambda t: t),
            "pins": types.SimpleNamespace(register_chip=self.chips.__setitem__),
            "gcode": types.SimpleNamespace(
                register_command=lambda name, fn, desc=None: self.commands.__setitem__(name, fn),
                run_script_from_command=self.scripts.append),
        }

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def load_object(self, config, name):
        return self.objects[name]

    def register_event_handler(self, event, callback):
        self.handlers[event].append(callback)

    def send_event(self, event, *args):
        for callback in self.handlers[event]:
            callback(*args)


def make_beacon(axes_map=((0, 1), (1, 1), (2, 1))):
    helper = types.SimpleNamespace(
        accel_stream_cmd=Sent(), _stream_en=0,
        _scale={"name": "16g", "id": 0, "scale": MG_PER_COUNT_16G},
        _scales={"16g": {"name": "16g", "id": 0, "scale": MG_PER_COUNT_16G}},
        _clip_values=(CLIP_COUNTS, -CLIP_COUNTS - 1),
        config=types.SimpleNamespace(axes_map=list(axes_map)),
        _handle_accel_data=lambda params: None)
    beacon_mcu = types.SimpleNamespace(
        _serial=types.SimpleNamespace(handlers={}),
        print_time_to_clock=lambda t: int(round(t * BEACON_HZ)),
        seconds_to_clock=lambda t: t * BEACON_HZ)
    beacon = types.SimpleNamespace(accel_helper=helper, _mcu=beacon_mcu, hooks=[],
                                   _clock32_to_time=lambda clock: clock / BEACON_HZ)
    beacon.compat_mcu_register_response = lambda cb, msg: beacon.hooks.append(cb)
    return beacon


def make_homing(axes_map=((0, 1), (1, 1), (2, 1)), impact_sign=-1, threshold=8.0,
                options=None, accel=20000.0, kin=None, connect=True, toolhead_cls=None):
    """A BeaconAccelHoming built through its real __init__ and connect handler."""
    toolhead = (toolhead_cls or FakeToolhead)(accel, kin)
    printer = FakePrinter(toolhead, make_beacon(axes_map))
    opts = {"threshold": threshold, "x_impact_sign": impact_sign, "y_impact_sign": impact_sign}
    opts.update(options or {})
    obj = plugin.BeaconAccelHoming(FakeConfig(printer, opts))
    obj.fake_printer = printer
    if connect:
        printer.send_event("klippy:connect")
    obj.session_scale = obj.scale
    obj.full_scale_g = obj._full_scale_g(obj.scale)
    obj.homing_active = True
    return obj


class FakeEndstop:
    def __init__(self, axis):
        self.axis = axis
        self.trigger_log = Sent()

    def trigger(self, code):
        self.trigger_log.send([code])


def arm_axis(obj, axis, arm_clock, deadline_clock):
    """Arm a homing pass on the trigger path directly, returning (attempt, trigger log)."""
    endstop = FakeEndstop(axis)
    count_g = obj.session_scale["scale"] / 1000.0
    index, map_sign = obj._raw_axis(axis)
    attempt = plugin.Attempt(endstop, 1, arm_clock / BEACON_HZ, arm_clock & 0xFFFFFFFF,
                             deadline_clock / BEACON_HZ, deadline_clock & 0xFFFFFFFF,
                             index, map_sign, obj.impact_sign[axis],
                             obj.threshold_g[axis] / count_g, count_g)
    obj.arm(attempt)
    return attempt, endstop.trigger_log


def counts(g):
    return int(round(g * 1000.0 / MG_PER_COUNT_16G))


def batch(start_clock, samples, receive_time=0.0, delta_clock=7):
    data = b"".join(struct.pack("<hhh", *sample) for sample in samples)
    return {"start_clock": start_clock & 0xFFFFFFFF, "delta_clock": delta_clock, "data": data,
            "#receive_time": receive_time}
