"""Offline tests for the beacon_accel_homing trigger path (serial thread side)."""
import unittest
from unittest import mock

from harness import arm_axis, batch, counts, make_homing, plugin

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST
ONE_G_Z = (0, 0, counts(1.0))


def still(n=8):
    return [ONE_G_Z] * n


def feed_baseline(obj, start=0):
    for i in range(12):
        obj._inspect(batch(start + i * 8, still()))


def x_sample(g):
    return (counts(g), 0, counts(1.0))


class DirectionalTriggerTest(unittest.TestCase):
    def test_impact_in_configured_direction_fires_endstop_hit(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, still()))
        hit = still()
        hit[3] = x_sample(-12.0)
        obj._inspect(batch(208, hit))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(attempt.onset_batch[2], 3)
        self.assertEqual(attempt.reason, "impact")

    def test_opposite_direction_never_triggers(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(14.0)] * 8))
        obj._inspect(batch(208, still()))
        self.assertEqual(trigger.messages, [])
        self.assertAlmostEqual(attempt.noise_pos * attempt.count_g, 14.0, delta=0.05)

    def test_other_axes_are_ignored(self):
        obj = make_homing()
        feed_baseline(obj)
        _attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [(0, counts(-15.0), counts(-14.0))] * 8))
        self.assertEqual(trigger.messages, [])

    def test_y_uses_its_own_axis_and_axes_map_sign(self):
        obj = make_homing(axes_map=((0, 1), (1, -1), (2, 1)))
        feed_baseline(obj)
        _attempt, trigger = arm_axis(obj, "y", 200, 9000)
        obj._inspect(batch(200, [(0, counts(12.0), counts(1.0))] * 8))
        self.assertEqual(trigger.messages, [[HIT]])

    def test_clipped_sample_triggers_with_sign_of_previous_sample(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(-3.0), (0x7FFF, 0, counts(1.0))] + still(6)))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertLess(attempt.onset_raw, 0)
        self.assertEqual(attempt.onset_batch[2], 1)

    def test_clip_against_the_configured_direction_never_triggers(self):
        # A positive clip is still a clip the wrong way for impact_sign -1.
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(3.0)] + [(0x7FFF, 0, counts(1.0))] * 7))
        # The next batch starts with a clip too. It follows the decoded clip
        # before it, not the raw 0x7FFF marker, so it stays positive.
        obj._inspect(batch(208, [(0x7FFF, 0, counts(1.0))] + still(7)))
        self.assertEqual(trigger.messages, [])
        hit = still()
        hit[2] = x_sample(-12.0)
        obj._inspect(batch(216, hit))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(attempt.onset_batch[2], 2)

    def test_ignored_clips_count_as_full_scale_in_the_measured_speed(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(3.0)] + [(0x7FFF, 0, counts(1.0))] * 7))
        clip = obj.clip_values[0]
        self.assertAlmostEqual(attempt.sums[0], counts(3.0) + 7 * clip - 8 * attempt.bases[0])

    def test_noise_peak_excludes_trigger_batch(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        quiet = still()
        quiet[1] = x_sample(-4.0)
        obj._inspect(batch(200, quiet))
        obj._inspect(batch(208, still()))
        hit = still()
        hit[5] = x_sample(-15.0)
        obj._inspect(batch(216, hit))
        self.assertAlmostEqual(attempt.noise_neg * attempt.count_g, 4.0, delta=0.05)

    def test_rising_edge_in_the_batch_before_the_impact_is_not_noise(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        edge = still()
        edge[7] = x_sample(-7.0)
        obj._inspect(batch(200, edge))
        obj._inspect(batch(208, [x_sample(-15.0)] + still(7)))
        self.assertEqual(attempt.noise_neg, 0.0)


class EitherDirectionTest(unittest.TestCase):
    """impact_sign 0 (the default) watches both directions of the homing axis."""

    def test_triggers_on_positive_and_negative_impacts(self):
        for g in (10.0, -10.0):
            obj = make_homing(impact_sign=0)
            feed_baseline(obj)
            attempt, trigger = arm_axis(obj, "x", 200, 9000)
            obj._inspect(batch(200, [x_sample(g)] + still(7)))
            self.assertEqual(trigger.messages, [[HIT]])
            self.assertEqual(attempt.onset_raw > 0, g > 0)

    def test_noise_below_threshold_in_both_directions_does_not_trigger(self):
        obj = make_homing(impact_sign=0)
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(7.0), x_sample(-7.0)] * 4))
        obj._inspect(batch(208, still()))
        self.assertEqual(trigger.messages, [])
        self.assertAlmostEqual(attempt.noise_pos * attempt.count_g, 7.0, delta=0.05)
        self.assertAlmostEqual(attempt.noise_neg * attempt.count_g, 7.0, delta=0.05)

    def test_default_impact_sign_is_either_direction(self):
        obj = make_homing(options={"x_impact_sign": 0, "y_impact_sign": 0})
        self.assertEqual(obj.impact_sign, {"x": 0, "y": 0})

    def test_checked_noise_skips_travel_direction_once_moving(self):
        # Once the toolhead travels, a jolt in the travel direction is not
        # compared with the threshold, so it is not counted as noise.
        obj = make_homing(impact_sign=0)
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        attempt.auto_direction = -1
        obj._inspect(batch(200, [x_sample(7.0), x_sample(-3.0)] * 4))
        obj._inspect(batch(208, still()))
        obj._inspect(batch(216, still()))
        self.assertEqual(trigger.messages, [])
        self.assertAlmostEqual(attempt.noise_pos * attempt.count_g, 7.0, delta=0.05)
        self.assertAlmostEqual(attempt.noise_checked * attempt.count_g, 3.0, delta=0.05)


class PerAxisThresholdTest(unittest.TestCase):
    def test_x_and_y_thresholds_are_independent(self):
        obj = make_homing(options={"x_threshold": 6.0, "y_threshold": 11.0})
        self.assertEqual(obj.threshold_g, {"x": 6.0, "y": 11.0})

    def test_threshold_sets_both_axes_unless_overridden(self):
        obj = make_homing(threshold=9.0, options={"y_threshold": 12.0})
        self.assertEqual(obj.threshold_g, {"x": 9.0, "y": 12.0})

    def test_each_axis_triggers_at_its_own_threshold(self):
        options = {"x_threshold": 6.0, "y_threshold": 11.0}
        obj = make_homing(impact_sign=0, options=options)
        feed_baseline(obj)
        _attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(-7.0)] + still(7)))
        self.assertEqual(trigger.messages, [[HIT]])
        for g, fired in ((-9.0, []), (-12.0, [[HIT]])):
            obj = make_homing(impact_sign=0, options=options)
            feed_baseline(obj)
            _attempt, trigger = arm_axis(obj, "y", 200, 9000)
            obj._inspect(batch(200, [(0, counts(g), counts(1.0))] + still(7)))
            self.assertEqual(trigger.messages, fired)


class SafetyTest(unittest.TestCase):
    def test_travel_deadline_stops_move_across_clock_wrap(self):
        obj = make_homing()
        base = 0xFFFFFF00
        feed_baseline(obj, start=base - 200)
        attempt, trigger = arm_axis(obj, "x", base, base + 300)
        obj._inspect(batch(base + 100, still()))
        self.assertEqual(trigger.messages, [])
        obj._inspect(batch(base + 300, still()))
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "travel")

    def test_missing_baseline_stops_instead_of_homing_blind(self):
        obj = make_homing()
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, still()))
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "no_baseline")

    def test_watchdog_stops_move_when_stream_dies(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        attempt.last_data = 1.0
        obj._watchdog(1.02)
        self.assertEqual(trigger.messages, [])
        obj._watchdog(1.1)
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "no_data")

    def test_watchdog_waits_for_move_start_before_judging_stream(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 5000, 90000)  # move starts at t=5.0
        attempt.last_data = 0.0
        obj._watchdog(1.0)
        obj._watchdog(4.9)
        self.assertEqual(trigger.messages, [])
        obj._watchdog(5.2)
        self.assertEqual(trigger.messages, [[HOST]])

    def test_watchdog_travel_stop_without_data_path(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 1000)  # deadline at t=1.0
        attempt.last_data = 1.2
        obj._watchdog(1.2)
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "travel")

    def test_disarmed_attempt_can_never_stop_the_next_move(self):
        obj = make_homing()
        feed_baseline(obj)
        stale, stale_trigger = arm_axis(obj, "x", 200, 9000)
        obj.disarm()
        fresh, fresh_trigger = arm_axis(obj, "x", 400, 9000)
        obj._fire(stale, "impact", HIT)
        self.assertEqual(stale_trigger.messages, [])
        self.assertEqual(fresh_trigger.messages, [])
        self.assertFalse(fresh.done)

    def test_short_batch_ignored(self):
        obj = make_homing()
        obj._inspect({"start_clock": 0, "delta_clock": 0, "data": b"\x00" * 6})
        self.assertEqual(len(obj.recent), 0)

    def test_big_endian_fallback_reads_the_same_values(self):
        data = batch(0, [x_sample(-5.0), x_sample(3.0)])["data"]
        fast = list(plugin.axis_values(data, 0))
        with mock.patch.object(plugin, "NATIVE_LE", False):
            self.assertEqual(plugin.axis_values(data, 0), fast)


class ReportingTest(unittest.TestCase):
    def test_impact_peak_follows_samples_after_trigger_and_after_disarm(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, [x_sample(-9.0)] + still(7)))
        obj._inspect(batch(208, [x_sample(-12.0)] + still(7)))
        record = attempt  # peak lives on the attempt until disarm
        obj.disarm()
        obj._inspect(batch(212, [x_sample(14.0)] + still(7)))
        obj._inspect(batch(400, [x_sample(15.5)] + still(7)))  # outside the impact window
        self.assertAlmostEqual(record.record["impact_peak_g"], 14.0, delta=0.05)
        self.assertEqual(record.record["impact_sign"], -1)

    def test_velocity_from_travel_has_the_travel_sign(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        # 0.5 g for 10 batches of 8 ms (fake clock), then stop.
        for i in range(10):
            obj._inspect(batch(200 + i * 8, [x_sample(0.5)] * 8, delta_clock=7))
        obj.disarm()
        self.assertGreater(attempt.record["velocity_mm_s"], 0)


if __name__ == "__main__":
    unittest.main()


class StreamGuardTest(unittest.TestCase):
    def test_other_client_switching_the_range_fails_the_home(self):
        obj = make_homing()
        feed_baseline(obj)
        obj.stream_alone = True
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj._inspect(batch(200, still()))
        self.assertEqual(trigger.messages, [])
        obj.helper._stream_en = 1
        obj.helper._scale = dict(obj.scale, id=obj.scale["id"] + 1)
        obj._inspect(batch(208, still()))
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "scale")

    def test_other_client_at_the_same_range_is_fine(self):
        obj = make_homing()
        feed_baseline(obj)
        obj.stream_alone = True
        _attempt, trigger = arm_axis(obj, "x", 200, 9000)
        obj.helper._stream_en = 1
        obj.helper._scale = obj.scale
        obj._inspect(batch(200, still()))
        self.assertEqual(trigger.messages, [])


class FakeGcmd:
    class Error(Exception):
        pass

    def __init__(self, **params):
        self.params, self.replies = params, []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_float(self, name, default=None, minval=None, maxval=None):
        value = self.params.get(name)
        return default if value is None else float(value)

    get_int = get_float

    def error(self, msg):
        return self.Error(msg)

    def respond_info(self, msg):
        self.replies.append(msg)


class SetCommandTest(unittest.TestCase):
    def test_threshold_above_the_sensor_range_is_refused(self):
        obj = make_homing()
        with self.assertRaises(FakeGcmd.Error):
            obj.cmd_SET(FakeGcmd(AXIS="X", THRESHOLD=14.0))
        self.assertEqual(obj.threshold_g["x"], 8.0)
        obj.cmd_SET(FakeGcmd(AXIS="X", THRESHOLD=10.0))
        self.assertEqual(obj.threshold_g["x"], 10.0)
