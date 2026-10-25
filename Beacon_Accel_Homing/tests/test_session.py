"""Offline tests for the homing session: stream, acceleration and checks."""
import collections
import types
import unittest

from harness import CommandError, ConfigError, batch, counts, make_homing, plugin

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST
Info = collections.namedtuple("Info", "speed position_endstop retract_speed retract_dist "
                                      "positive_dir second_homing_speed")
KalicoInfo = collections.namedtuple("KalicoInfo", Info._fields + ("min_home_dist",))


def info(speed=150.0, retract=5.0, second=150.0, min_home_dist=None):
    if min_home_dist is None:
        return Info(speed, 0.0, speed, retract, False, second)
    return KalicoInfo(speed, 0.0, speed, retract, False, second, min_home_dist)


class FakeRail:
    def __init__(self, endstop, homing_info=None, name="stepper_x", rng=(0.0, 300.0)):
        self.endstop, self.info, self.name, self.rng = endstop, homing_info or info(), name, rng

    def get_endstops(self):
        return [(self.endstop, self.name)]

    def get_homing_info(self):
        return self.info

    def get_range(self):
        return self.rng

    def get_name(self):
        return self.name


def fake_stepper(name, mcu):
    return types.SimpleNamespace(get_mcu=lambda: mcu, get_name=lambda: name,
                                 get_step_dist=lambda: 0.0125)


class Board:
    """A stepper MCU that records trsync_trigger commands per dedicated queue."""

    def __init__(self, name="mcu", log=None):
        self.name, self.log, self.queues = name, [] if log is None else log, []

    def alloc_command_queue(self):
        self.queues.append(object())
        return self.queues[-1]

    def lookup_command(self, fmt, cq=None):
        assert cq is self.queues[-1], "trigger must use a dedicated queue"
        return types.SimpleNamespace(send=lambda args: self.log.append((self.name, list(args))))


def with_endstops(obj, axes=("x",), mcu_obj=None):
    stepper_mcu = mcu_obj or Board()
    endstops = {}
    for axis in axes:
        endstop = obj.setup_pin("endstop", {"pin": axis, "invert": False, "pullup": False})
        endstop.add_stepper(fake_stepper("stepper_" + axis, stepper_mcu))
        endstop.setup_trigger(obj._trigger_command)
        endstops[axis] = endstop
    return endstops


def stream_flows(obj):
    """Make the stream answer as soon as the session waits for it."""
    reactor = obj.fake_printer.reactor
    reactor.on_pause = lambda: obj._inspect(batch(0, [(0, 0, counts(1.0))] * 8,
                                                  receive_time=reactor.now))


def start_session(obj, rail):
    obj.homing_active = False
    stream_flows(obj)
    obj.fake_printer.reactor.now = 10.0
    obj._handle_rails_begin(None, [rail])


class AccelLimitTest(unittest.TestCase):
    def test_session_limits_accel_and_restores_it_at_rails_end(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        start_session(obj, rail)
        self.assertEqual(obj.toolhead.accel, 5000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages, [[1, 0]])
        obj._handle_rails_end(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages[-1], [0, 0])

    def test_accel_limit_works_on_kalico_and_older_klipper(self):
        from harness import KalicoToolhead, OldToolhead
        for cls in (KalicoToolhead, OldToolhead):
            obj = make_homing(accel=20000.0, toolhead_cls=cls)
            rail = FakeRail(with_endstops(obj)["x"])
            start_session(obj, rail)
            self.assertEqual(obj.toolhead.accel, 5000.0, cls.__name__)
            obj._handle_rails_end(None, [rail])
            self.assertEqual(obj.toolhead.accel_log, [5000.0, 20000.0], cls.__name__)

    def test_toolhead_without_accel_setter_is_a_config_error(self):
        from harness import BareToolhead
        with self.assertRaisesRegex(ConfigError, "homing_accel: 0"):
            make_homing(toolhead_cls=BareToolhead)
        make_homing(toolhead_cls=BareToolhead, options={"homing_accel": 0.0})

    def test_accel_is_never_raised(self):
        obj = make_homing(accel=3000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertEqual(obj.toolhead.accel, 3000.0)
        self.assertEqual(obj.toolhead.accel_log, [])

    def test_homing_accel_zero_leaves_accel_alone(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 0})
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_failed_home_restores_accel_on_motor_off(self):
        # Klipper sends no rails_end after a failed home, G28 turns motors off.
        obj = make_homing(accel=20000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        obj.fake_printer.send_event("stepper_enable:motor_off")
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertFalse(obj.homing_active)
        self.assertEqual(obj.helper.accel_stream_cmd.messages[-1], [0, 0])

    def test_session_left_open_closes_itself(self):
        obj = make_homing(accel=20000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertNotEqual(obj._idle_check(11.0), obj.reactor.NEVER)
        self.assertEqual(obj._idle_check(10.0 + plugin.IDLE_CLEANUP + 1.0), obj.reactor.NEVER)
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_new_session_after_failure_keeps_the_original_accel(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        start_session(obj, rail)
        start_session(obj, rail)  # no cleanup in between
        obj._handle_rails_end(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_stream_that_never_starts_restores_accel_and_fails(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        obj.homing_active = False
        with self.assertRaises(CommandError):
            obj._handle_rails_begin(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertFalse(obj.homing_active)


class SessionTest(unittest.TestCase):
    def test_rails_without_beacon_endstops_are_ignored(self):
        obj = make_homing(accel=20000.0)
        with_endstops(obj)
        obj.homing_active = False
        obj._handle_rails_begin(None, [FakeRail(object(), name="stepper_z")])
        self.assertFalse(obj.homing_active)
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages, [])

    def test_two_beacon_axes_in_one_homing_call_are_rejected(self):
        obj = make_homing()
        endstops = with_endstops(obj, ("x", "y"))
        obj.homing_active = False
        with self.assertRaises(CommandError):
            obj._handle_rails_begin(None, [FakeRail(endstops["x"]), FakeRail(endstops["y"], name="stepper_y")])

    def test_baseline_dwell_before_every_pass_of_our_axis_only(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [object()]))
        self.assertEqual(obj.toolhead.dwells, [0.1])

    def test_home_start_outside_g28_fails_before_the_trsync_starts(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        obj.homing_active = False
        with self.assertRaises(CommandError):
            endstop.home_start(1.0, 0, 0, 0)
        self.assertEqual(endstop._dispatch.started, [])

    def test_deadline_covers_ramp_to_speed(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 1000})
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        attempt = obj.prepare(endstop, 100.0)
        # 330 mm at 150 mm/s, plus 0.15 s ramp at 1000 mm/s^2, plus the margin
        self.assertAlmostEqual(attempt.deadline - 100.0, 330.0 / 150.0 + 0.15 + 0.15, places=6)
        second = obj.prepare(endstop, 200.0)
        self.assertAlmostEqual(second.deadline - 200.0, 13.0 / 150.0 + 0.15 + 0.15, places=6)
        self.assertEqual(second.pass_no, 2)

    def test_deadline_uses_max_velocity_when_lower_than_homing_speed(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 1000})
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj.toolhead.get_max_velocity = lambda: (50.0, 1000.0)
        attempt = obj.prepare(endstop, 100.0)
        self.assertAlmostEqual(attempt.deadline - 100.0, 330.0 / 50.0 + 0.05 + 0.15, places=6)

    def test_full_home_reports_onset_time(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        for i in range(12):
            obj._inspect(batch(i * 8, [(0, 0, counts(1.0))] * 8))
        board = endstop.get_mcu()
        endstop.home_start(0.2, 0, 0, 0)
        obj._inspect(batch(200, [(counts(-12.0), 0, counts(1.0))] + [(0, 0, counts(1.0))] * 7))
        self.assertEqual(board.log, [("mcu", [7, HIT])])
        self.assertAlmostEqual(endstop.home_wait(1.0), 0.2)
        self.assertEqual(obj.last["x"]["result"], "impact")
        self.assertIsNone(obj.attempt)

    def test_failures_raise_instead_of_setting_an_origin(self):
        for reason in ("no_data", "no_baseline", "travel"):
            obj = make_homing()
            endstop = with_endstops(obj)["x"]
            attempt = types.SimpleNamespace(reason=reason, onset_batch=None)
            with self.assertRaises(CommandError):
                obj.finish(endstop, attempt, HOST)
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        with self.assertRaises(CommandError):
            obj.finish(endstop, types.SimpleNamespace(reason=None, onset_batch=None),
                       plugin.mcu.MCU_trsync.REASON_COMMS_TIMEOUT)
        self.assertEqual(obj.finish(endstop, types.SimpleNamespace(reason=None, onset_batch=None),
                                    plugin.mcu.MCU_trsync.REASON_PAST_END_TIME), 0.0)


class TriggerMcuTest(unittest.TestCase):
    def test_mcu_comes_from_the_first_stepper(self):
        obj = make_homing()
        board = Board()
        endstop = with_endstops(obj, mcu_obj=board)["x"]
        self.assertIs(endstop.get_mcu(), board)

    def test_trigger_reaches_every_stepper_mcu_on_its_own_queue(self):
        sent = []
        obj = make_homing(connect=False)
        main, extra = Board("mcu", sent), Board("extra", sent)
        endstop = obj.setup_pin("endstop", {"pin": "x", "invert": False, "pullup": False})
        endstop.add_stepper(fake_stepper("stepper_x", main))
        endstop.add_stepper(fake_stepper("stepper_y", extra))
        obj.fake_printer.send_event("klippy:connect")
        endstop.trigger(HIT)
        self.assertEqual(sent, [("mcu", [7, HIT]), ("extra", [8, HIT])])

    def test_old_mcu_option_still_loads(self):
        obj = make_homing(options={"mcu": "mcu"})
        self.assertTrue(obj.endstops is not None)


class ConnectChecksTest(unittest.TestCase):
    def test_missing_accelerometer_is_a_config_error_at_startup(self):
        obj = make_homing(connect=False)
        obj.fake_printer.objects["beacon"].accel_helper = None
        with self.assertRaises(ConfigError):
            obj.fake_printer.send_event("klippy:connect")

    def test_missing_beacon_internals_are_named(self):
        obj = make_homing(connect=False)
        del obj.fake_printer.objects["beacon"].accel_helper.accel_stream_cmd
        with self.assertRaisesRegex(ConfigError, "accel_stream_cmd"):
            obj.fake_printer.send_event("klippy:connect")

    def test_threshold_above_sensor_range_is_rejected(self):
        with self.assertRaises(ConfigError):
            make_homing(threshold=14.0)

    def test_missing_beacon_section_is_a_config_error(self):
        from harness import FakeConfig, FakePrinter, FakeToolhead, make_beacon
        printer = FakePrinter(FakeToolhead(), make_beacon())
        with self.assertRaises(ConfigError):
            plugin.BeaconAccelHoming(FakeConfig(printer, {}, sections=()))

    def test_rail_warnings_show_at_startup(self):
        obj = make_homing(connect=False)
        endstop = with_endstops(obj)["x"]
        rail = FakeRail(endstop, info(speed=30.0, retract=0.0))
        obj.fake_printer.objects["toolhead"].kin = types.SimpleNamespace(rails=[rail, FakeRail(object())])
        obj.fake_printer.send_event("klippy:connect")
        warnings = obj.fake_printer.objects["configfile"].warnings
        self.assertEqual(len(warnings), 1)
        self.assertIn("30 mm/s", warnings[0])

    def test_homing_warnings(self):
        self.assertEqual(plugin.homing_warnings("stepper_x", info()), [])
        # One touch per home is the design, so retract 0 is not a warning.
        self.assertEqual(plugin.homing_warnings("stepper_x", info(retract=0.0)), [])
        # Kalico min_home_dist gives the second touch for starts at the stop.
        self.assertEqual(plugin.homing_warnings("stepper_x", info(retract=0.0, min_home_dist=5.0)), [])
        self.assertEqual(len(plugin.homing_warnings("stepper_x", info(second=20.0))), 2)
        self.assertEqual(len(plugin.homing_warnings("stepper_x", info(speed=20.0, second=20.0))), 1)


class TravelCheckTest(unittest.TestCase):
    def _home(self, obj, endstop, pass_no, travel_mm):
        kin = types.SimpleNamespace(
            get_steppers=lambda: [types.SimpleNamespace(get_name=lambda: "stepper_x")],
            calc_position=lambda moved: [moved["stepper_x"], 0.0, 0.0])
        obj.toolhead.kin = kin
        stepper = types.SimpleNamespace(get_step_dist=lambda: 0.0125)
        trig = -int(round(travel_mm / 0.0125))
        sp = types.SimpleNamespace(stepper_name="stepper_x", stepper=stepper, start_pos=0,
                                   trig_pos=trig, halt_pos=trig - 30)
        record = {"axis": "x", "pass": pass_no, "result": "impact", "onset_g": -12.0,
                  "impact_peak_g": 16.0, "noise_pos_g": 3.0, "noise_neg_g": 3.0,
                  "trigger_after_arm_s": 0.05}
        attempt = types.SimpleNamespace(endstop=endstop, axis="x", pass_no=pass_no, reason="impact",
                                        record=record)
        obj.pending_move = attempt
        obj._handle_move_end(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop],
                                                   stepper_positions=[sp]))
        return attempt.record

    def test_second_touch_at_the_stop_passes(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        record = self._home(obj, endstop, 2, 5.2)
        self.assertAlmostEqual(record["travel_mm"], 5.2)

    def test_early_second_touch_stops_homing(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 1.0)

    def test_late_second_touch_stops_homing(self):
        # An early first trigger left the second touch 10 mm
        # of commanded travel, and the end of that move was taken as the stop.
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=0.0, min_home_dist=5.0)))
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 9.881)

    def test_kalico_rehome_uses_min_home_dist(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=0.0, min_home_dist=5.0)))
        self._home(obj, endstop, 2, 4.8)
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 2.0)

class StatusTest(unittest.TestCase):
    def test_status_is_a_copy(self):
        obj = make_homing()
        obj.last["x"] = {"result": "impact", "impact_peak_g": 10.0}
        status = obj.get_status(0.0)
        obj.last["x"]["impact_peak_g"] = 12.0
        self.assertEqual(status["last"]["x"]["impact_peak_g"], 10.0)
        self.assertEqual(set(status), {"threshold_g", "impact_sign", "homing_accel", "back_off_dist",
                                       "homing", "last"})


if __name__ == "__main__":
    unittest.main()


class EnableLine:
    def __init__(self, log, name, enabled):
        self.log, self.name, self.enabled = log, name, enabled

    def is_motor_enabled(self):
        return self.enabled

    def motor_enable(self, print_time):
        self.log.append((self.name, print_time))
        self.enabled = True


class MotorEnableTest(unittest.TestCase):
    """Motors that were off are switched on and settle before the resting reading."""

    def _home_x(self, enabled, active=("stepper_x", "stepper_y")):
        obj = make_homing()
        log = []
        lines = {name: EnableLine(log, name, on) for name, on in enabled.items()}
        steppers = [types.SimpleNamespace(get_name=lambda n=name: n,
                                          is_active_axis=lambda a, n=name: a == "x" and n in active)
                    for name in enabled]
        toolhead = obj.toolhead
        toolhead.kin = types.SimpleNamespace(get_steppers=lambda: steppers,
                                             get_status=lambda t: {"homed_axes": ""})
        toolhead.flush_step_generation = lambda: None
        toolhead.get_last_move_time = lambda: 42.0
        toolhead.get_position = lambda: [150.0, 298.0, 40.0, 0.0]
        obj.fake_printer.objects["stepper_enable"] = types.SimpleNamespace(
            lookup_enable=lines.__getitem__)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        return log, toolhead

    def test_motors_off_are_switched_on_before_the_move(self):
        # CoreXY: both motors drive X. Z is left alone.
        log, toolhead = self._home_x({"stepper_x": False, "stepper_y": False, "stepper_z": False})
        self.assertEqual(log, [("stepper_x", 42.0), ("stepper_y", 42.0)])
        self.assertIn(plugin.ENABLE_SETTLE, toolhead.dwells)

    def test_motors_already_on_add_no_delay(self):
        log, toolhead = self._home_x({"stepper_x": True, "stepper_y": True})
        self.assertEqual(log, [])
        self.assertNotIn(plugin.ENABLE_SETTLE, toolhead.dwells)

    def test_without_stepper_enable_nothing_happens(self):
        obj = make_homing()
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertNotIn(plugin.ENABLE_SETTLE, obj.toolhead.dwells)


class BackOffTest(unittest.TestCase):
    """An axis left pushed into its stop is moved off it before the next axis homes."""

    def _home_y(self, x_position, homed="xyz", x_info=None, options=None):
        obj = make_homing(options=options)
        endstops = with_endstops(obj, axes=("x", "y"))
        toolhead = obj.toolhead
        toolhead.position = [x_position, 150.0, 10.0, 0.0]
        toolhead.get_position = lambda: list(toolhead.position)
        toolhead.kin = types.SimpleNamespace(get_status=lambda t: {"homed_axes": homed})
        obj.rails_by_axis["x"] = FakeRail(endstops["x"], x_info or info())
        start_session(obj, FakeRail(endstops["y"], info(), name="stepper_y"))
        return toolhead

    def test_x_left_at_the_stop_backs_off_before_y_homes(self):
        toolhead = self._home_y(-0.4)
        self.assertEqual(toolhead.moves, [([5.0, None, None], 150.0)])
        self.assertIn(0.1, toolhead.dwells)

    def test_x_already_clear_of_the_stop_stays(self):
        self.assertEqual(self._home_y(50.0).moves, [])

    def test_back_off_can_be_switched_off(self):
        self.assertEqual(self._home_y(-0.4, options={"back_off_dist": 0.0}).moves, [])

    def test_unhomed_x_is_never_moved(self):
        self.assertEqual(self._home_y(-0.4, homed="").moves, [])

    def test_axis_homing_to_its_max_backs_off_downwards(self):
        x_info = Info(150.0, 300.0, 150.0, 0.0, True, 150.0)
        toolhead = self._home_y(300.3, x_info=x_info)
        self.assertEqual(toolhead.moves, [([295.0, None, None], 150.0)])
