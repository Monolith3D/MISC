"""Offline tests for single-touch homing: pressed starts and the travel-direction guard."""
import types
import unittest

from harness import arm_axis, batch, counts, make_homing, plugin

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
STEP = 0.0125
REST = (0, 0, counts(1.0))


class RampStepper:
    """Stepper whose step history follows a ramp from rest at `start` (fake seconds)."""

    def __init__(self, start=0.2, accel=5000.0, speed=150.0):
        self.start, self.accel, self.speed = start, accel, speed

    def get_step_dist(self):
        return STEP

    def get_past_mcu_position(self, t):
        dt = max(0.0, t - self.start)
        ramp = self.speed / self.accel
        dist = 0.5 * self.accel * dt * dt if dt < ramp else (
            0.5 * self.speed * ramp + self.speed * (dt - ramp))
        return int(dist / STEP)


def armed(impact_sign=0, stepper_start=0.2):
    """Baseline fed, pass armed at 0.2 s (fake clock in ms), steps ramping at 5000 mm/s^2."""
    obj = make_homing(impact_sign=impact_sign)
    for i in range(12):
        obj._inspect(batch(i * 8, [REST] * 8))
    attempt, trigger = arm_axis(obj, "x", 200, 900000)
    attempt.steppers = [RampStepper(start=stepper_start)]
    return obj, attempt, trigger


def feed(obj, attempt, start, x_g, batches, noise=1):
    """Batches of 8 samples (1 ms apart in the fake clock) with a steady X reading
    plus one count of sensor noise (noise=0 gives a frozen stream)."""
    for i in range(batches):
        obj._inspect(batch(start + i * 8, [(counts(x_g) + noise * (j % 2), 0, counts(1.0))
                                           for j in range(8)]))
        if not attempt.done:
            obj._check_pressed(attempt)
    return start + batches * 8


class PressedStartTest(unittest.TestCase):
    def test_steps_running_without_motion_stop_as_pressed(self):
        obj, attempt, trigger = armed()
        feed(obj, attempt, 200, 0.0, 6)
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(attempt.reason, "pressed")
        obj.disarm()
        # The origin is where the move started: the trigger time is the arm time.
        self.assertAlmostEqual(obj.trigger_time(attempt), attempt.arm_time)
        self.assertEqual(attempt.record["result"], "pressed")

    def test_frozen_readings_fail_instead_of_pressed(self):
        # Identical readings while the steps run mean a dead sensor, not a pressed
        # start. Homing must fail rather than set the origin at the start.
        obj, attempt, trigger = armed()
        feed(obj, attempt, 200, 0.0, 6, noise=0)
        self.assertEqual(attempt.reason, "frozen")
        self.assertNotEqual(trigger.messages, [[HIT]])
        obj.disarm()
        with self.assertRaises(Exception):
            obj.finish(attempt.endstop, attempt, HIT)

    def test_missing_samples_mid_move_never_count_as_pressed(self):
        # A gap leaves the integrated speed too low, so a moving toolhead would
        # look pressed and the origin would be set where the move started.
        obj, attempt, trigger = armed()
        feed(obj, attempt, 200, 0.0, 1)
        feed(obj, attempt, 232, 0.0, 6)
        self.assertTrue(attempt.gap)
        self.assertEqual(trigger.messages, [])
        self.assertNotEqual(attempt.reason, "pressed")

    def test_missing_samples_at_the_start_never_count_as_pressed(self):
        obj, attempt, trigger = armed()
        feed(obj, attempt, 230, 0.0, 6)
        self.assertTrue(attempt.gap)
        self.assertEqual(trigger.messages, [])

    def test_toolhead_moving_with_the_steps_is_never_pressed(self):
        obj, attempt, trigger = armed()
        # 0.5 g is about 4900 mm/s^2, the same as the commanded ramp.
        feed(obj, attempt, 200, 0.5, 30)
        self.assertEqual(trigger.messages, [])
        self.assertTrue(attempt.moving)

    def test_toolhead_lagging_the_steps_by_10_ms_is_not_pressed(self):
        obj, attempt, trigger = armed(stepper_start=0.19)
        feed(obj, attempt, 200, 0.5, 30)
        self.assertEqual(trigger.messages, [])

    def test_no_decision_while_the_commanded_speed_is_low(self):
        obj, attempt, trigger = armed()
        feed(obj, attempt, 200, 0.0, 2)  # 16 ms: commanded 15 ms earlier is still 0
        self.assertEqual(trigger.messages, [])
        self.assertIsNone(obj._motion_state(attempt))

    def test_impact_after_free_travel_keeps_the_impact_time(self):
        # Start close to the stop: the toolhead travels with the steps, then hits.
        obj, attempt, trigger = armed()
        end = feed(obj, attempt, 200, 0.5, 4)
        obj._inspect(batch(end, [(counts(-14.0), 0, counts(1.0))] + [REST] * 7))
        self.assertEqual(attempt.reason, "impact")
        obj.disarm()
        self.assertAlmostEqual(obj.trigger_time(attempt), end / 1000.0)

    def test_impact_without_any_motion_counts_as_pressed(self):
        # Steps ran for 30 ms, the toolhead never moved, then a sharp reading.
        obj, attempt, trigger = armed()
        attempt.pressed_checks = -99  # keep the watchdog check from firing first
        end = feed(obj, attempt, 200, 0.0, 4)
        obj._inspect(batch(end, [(counts(-14.0), 0, counts(1.0))] + [REST] * 7))
        obj.disarm()
        self.assertAlmostEqual(obj.trigger_time(attempt), attempt.arm_time)
        self.assertEqual(attempt.record["result"], "pressed")

    def test_motion_in_another_sensor_axis_counts_as_moving(self):
        # A wrong axes map must never make a moving toolhead look pressed.
        obj, attempt, trigger = armed()
        for i in range(30):
            obj._inspect(batch(200 + i * 8, [(0, 0, counts(1.5))] * 8))
            obj._check_pressed(attempt)
        self.assertEqual(trigger.messages, [])


class KeepOriginTest(unittest.TestCase):
    def _session(self, homed, position):
        from test_session import FakeRail, info, start_session, with_endstops
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        rail = FakeRail(endstop, info(retract=0.0))
        toolhead = obj.toolhead
        toolhead.position = [position, 150.0, 10.0, 0.0]
        toolhead.get_position = lambda: list(toolhead.position)
        toolhead.set_position = lambda pos: setattr(toolhead, "position", list(pos))
        toolhead.kin = types.SimpleNamespace(
            get_status=lambda t: {"homed_axes": "xyz" if homed else ""})
        start_session(obj, rail)
        attempt = types.SimpleNamespace(reason="pressed", axis="x", halt_delta=-2.0, record={})
        obj.last_pass = attempt
        toolhead.position = [-2.0, 150.0, 10.0, 0.0]  # what Klipper sets after a pressed home
        obj._handle_rails_end(None, [rail])
        return toolhead.position, attempt.record

    def test_homed_axis_at_the_stop_keeps_its_origin(self):
        # Previous home left the motors 0.3 mm past the stop. Without the kept
        # origin every G28 in a row would move the origin by the belt stretch.
        position, record = self._session(True, -0.3)
        self.assertAlmostEqual(position[0], -2.3)
        self.assertTrue(record["kept_origin"])

    def test_unhomed_axis_takes_the_start_as_origin(self):
        position, record = self._session(False, 0.0)
        self.assertAlmostEqual(position[0], -2.0)
        self.assertNotIn("kept_origin", record)

    def test_homed_axis_far_from_the_stop_is_not_kept(self):
        position, record = self._session(True, 40.0)
        self.assertAlmostEqual(position[0], -2.0)


class TravelDirectionTest(unittest.TestCase):
    """With no configured sign, travel fixes which way an impact must point."""

    def test_jolt_in_the_travel_direction_is_ignored_once_moving(self):
        obj, attempt, trigger = armed()
        end = feed(obj, attempt, 200, 1.0, 2)  # about 150 mm/s in +X
        self.assertEqual(attempt.auto_direction, -1)
        obj._inspect(batch(end, [(counts(9.0), 0, counts(1.0))] + [REST] * 7))
        self.assertEqual(trigger.messages, [])
        obj._inspect(batch(end + 8, [(counts(-9.0), 0, counts(1.0))] + [REST] * 7))
        self.assertEqual(trigger.messages, [[HIT]])

    def test_both_directions_before_the_toolhead_travels(self):
        obj, attempt, trigger = armed()
        obj._inspect(batch(200, [(counts(9.0), 0, counts(1.0))] + [REST] * 7))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(attempt.auto_direction, 0)

    def test_configured_sign_wins(self):
        obj, attempt, trigger = armed(impact_sign=1)
        end = feed(obj, attempt, 200, 1.0, 2)
        self.assertEqual(attempt.auto_direction, 0)
        obj._inspect(batch(end, [(counts(9.0), 0, counts(1.0))] + [REST] * 7))
        self.assertEqual(trigger.messages, [[HIT]])


if __name__ == "__main__":
    unittest.main()
