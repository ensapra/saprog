"""Cartesian joystick mapping and command lifecycle without a controller or robot."""

# ruff: noqa: E402 -- configure Qt and the desktop import path first.

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from documents.snapshot_units import AX_TICK_RADIANS
from limit_fixtures import prime_limits
from PySide6.QtCore import QEvent, QObject, QPoint, QSettings, Signal
from PySide6.QtWidgets import QApplication, QScrollArea, QSplitter
from tabs.leg_test.joystick import STATE_TOPIC, CartesianJoystick
from tabs.leg_test.urdf_model import DESCRIPTION, LEGS, SingleLegModel
from tabs.leg_test.urdf_tab import SingleLegTestTab

_APP = QApplication.instance() or QApplication([])


def dispose(obj):
    obj.deleteLater()
    _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)


class FakeManager(QObject):
    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    command_result = Signal(str, bool, str)
    position_published = Signal(str, object)
    torque_release_requested = Signal()

    def __init__(self):
        super().__init__()
        self.request_topic = Mock(return_value="joystick-feedback")
        self.refresh_servo_limits = Mock(return_value="limits-read")
        self.release = Mock()
        self.cancel_commands = Mock()
        self.publish_joint_command = Mock(side_effect=(f"move-{i}" for i in range(100)))
        prime_limits(self)


class FakeMonitor(QObject):
    devices_changed = Signal(object)
    selected_changed = Signal(object)
    inputs_changed = Signal(object)
    axes_changed = Signal(object)
    status_changed = Signal(str)

    def __init__(self):
        super().__init__()
        self.closed = False

    def start(self):
        self.refresh()

    def refresh(self):
        self.devices_changed.emit([dict(id=42, name="Test pad", guid="test-pad")])

    def select(self, instance):
        self.selected_changed.emit(dict(id=42, name="Test pad", guid="test-pad", axis_count=4,
                                        controls=[("button:0", "Button 1"), ("button:1", "Button 2")])
                                   if instance is not None else None)
        self.axes_changed.emit((0., 0., 0., 0.) if instance is not None else ())

    def close(self):
        self.closed = True
        self.select(None)


class CartesianJoystickTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.now = 100.
        self.engine = CartesianJoystick(self.manager, clock=lambda: self.now)
        self.model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", bounds=self.engine.limits.bounds)
        self.seed = {m.servo: .55 if i else 0. for i, m in enumerate(self.model.mappings)}
        self.engine.update_input((0., 0., 0., 0.), set())
        self.addCleanup(dispose, self.manager)
        self.addCleanup(dispose, self.engine)
        self.addCleanup(self.engine.stop)

    def step(self, axes=(0., 0., 0., 0.), pressed=(), *, feedback=False):
        self.now += .05
        self.engine.update_input(axes, pressed)
        if feedback:
            self.report(self.engine.pose)
        self.engine.tick()

    def report(self, pose):
        self.manager.message_received.emit(STATE_TOPIC, dict(name=list(pose), position=list(pose.values())))

    def arm_live(self):
        self.engine.start(self.model, self.seed, hardware=True)
        self.report(self.seed)
        self.step(pressed={"button:0"})
        self.assertTrue(self.engine.neutral)

    def test_stick_moves_cartesian_XZ_and_holds_Y_for_every_leg(self):
        for leg in LEGS:
            with self.subTest(leg=leg):
                model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", leg, self.engine.limits.bounds)
                seed = {m.servo: .55 if i else 0. for i, m in enumerate(model.mappings)}
                self.engine.start(model, seed)
                self.step()
                origin = self.engine.target.copy()
                self.step((0., -1., 0., 0.))
                np.testing.assert_allclose(self.engine.target-origin, [0, 0, .0005], atol=1e-12)
                self.step((1., 0., 0., 0.))
                np.testing.assert_allclose(self.engine.target-origin, [.0005, 0, .0005], atol=1e-12)
                np.testing.assert_allclose(model.foot_position(self.engine.pose), self.engine.target, atol=1e-5)
                before = self.engine.target.copy()
                self.step((.1, -.1, 0., 0.))
                np.testing.assert_array_equal(self.engine.target, before)
                self.engine.stop()
        self.manager.publish_joint_command.assert_not_called()
        self.manager.request_topic.assert_not_called()

    def test_axis_assignment_reversal_speed_and_diagonal_limit(self):
        self.engine.start(self.model, self.seed, axes=(2, 3), horizontal=0, reverse=(True, True))
        self.step()
        before = self.engine.target.copy()
        self.step((1., 1., 1., -1.))
        delta = self.engine.target-before
        self.assertLess(delta[0], 0)
        self.assertLess(delta[2], 0)
        self.assertEqual(delta[1], 0)
        self.assertAlmostEqual(np.linalg.norm(delta), .0005)

    def test_stick_centering_and_held_input_survive_delayed_ticks(self):
        self.engine.update_input((1., 0.), set())
        self.engine.start(self.model, self.seed)
        before = self.engine.target.copy()
        self.step((1., 0.))
        np.testing.assert_array_equal(self.engine.target, before)
        self.step((0., 0.))
        self.step((1., 0.))
        self.assertGreater(np.linalg.norm(self.engine.target-before), 0)
        self.now += .3
        self.engine.tick()
        self.assertTrue(self.engine.active)
        self.assertGreater(self.engine.target[0], before[0]+.0005)
        self.engine.update_input((), set())
        self.assertFalse(self.engine.active)

    def test_each_motion_tick_reads_held_stick_without_axis_change_events(self):
        reader = Mock(return_value=((0., 0.), set()))
        self.engine.read_input = reader
        self.engine.start(self.model, self.seed)
        self.now += .5
        self.engine.tick()
        before = self.engine.target.copy()
        reader.return_value = ((0., -1.), set())
        for _ in range(5):
            self.now += .5
            self.engine.tick()
        self.assertTrue(self.engine.active)
        self.assertAlmostEqual(self.engine.target[2]-before[2], .005)
        self.assertEqual(reader.call_count, 7)
        reader.return_value = ((), set())
        self.engine.tick()
        self.assertFalse(self.engine.active)

    def test_servo_limit_blocks_outward_jog_and_reverse_moves_back_immediately(self):
        self.model.bounds = dict(self.model.bounds)
        point, jacobian = self.model.foot_kinematics(self.seed)
        delta = np.linalg.solve(jacobian, [0., 0., .0005])
        index = int(np.argmax(abs(delta)))
        servo = self.model.mappings[index].servo
        low, high = self.model.bounds[servo]
        limit = self.seed[servo] + delta[index]*.3
        self.model.bounds[servo] = (low, limit) if delta[index] > 0 else (limit, high)
        self.engine.start(self.model, self.seed)
        self.step()
        for _ in range(15):
            self.step((0., -1.))
            self.assertTrue(self.model.in_limits(self.engine.pose))
        self.assertTrue(self.engine.active)
        self.assertAlmostEqual(self.engine.pose[servo], limit, delta=2e-5)
        self.assertGreater(self.engine.target[2], point[2])
        self.assertLess(self.engine.target[2]-point[2], .0005)
        blocked = self.engine.target.copy()
        self.step((0., 1.))
        self.assertLess(self.engine.target[2], blocked[2])
        np.testing.assert_allclose(self.model.foot_position(self.engine.pose)[:2], point[:2], atol=2e-6)

    def test_invalid_or_duplicate_axes_and_missing_limits_are_rejected(self):
        for axes in ((0, 0), (0, 8), (None, 1)):
            with self.subTest(axes=axes), self.assertRaises(ValueError):
                self.engine.start(self.model, self.seed, axes=axes)
        self.engine.limits.invalidate("disconnected")
        with self.assertRaises(ValueError):
            self.engine.start(self.model, self.seed)

    def test_unreachable_target_holds_last_valid_target_and_can_move_back(self):
        self.engine.start(self.model, self.seed)
        self.step()
        before = self.engine.target.copy()
        with patch.object(self.model, "solve", side_effect=ValueError("unreachable")):
            self.step((1., 0.))
        np.testing.assert_array_equal(self.engine.target, before)
        self.assertTrue(self.engine.active)
        self.step((-1., 0.))
        self.assertLess(self.engine.target[0], before[0])

    def test_live_waits_for_measured_pose_enable_and_neutral(self):
        self.engine.start(self.model, self.seed, hardware=True)
        self.step((1., 0.), {"button:0"})
        self.manager.publish_joint_command.assert_not_called()
        measured = dict(self.seed, servo_12=.65)
        self.report(measured)
        self.step((1., 0.), {"button:0"})
        self.manager.publish_joint_command.assert_not_called()
        np.testing.assert_allclose(self.engine.target, self.model.foot_position(measured))
        self.step((0., 0.), {"button:0"})
        self.step((1., 0.), {"button:0"})
        self.manager.publish_joint_command.assert_called_once()
        joints, positions, speed, torque = self.manager.publish_joint_command.call_args.args
        self.assertEqual(set(joints), {"servo_11", "servo_12", "servo_13"})
        self.assertEqual(len(speed), len(joints))
        self.assertTrue(all(1 <= value <= 100 for value in speed))
        self.assertEqual(torque, 300)
        np.testing.assert_allclose(self.model.foot_position(dict(zip(joints, positions))),
                                   self.engine.target, atol=1e-5)

    def test_live_centers_and_locks_x1_for_every_leg(self):
        for leg in LEGS:
            with self.subTest(leg=leg):
                self.manager.publish_joint_command.reset_mock()
                model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", leg, self.engine.limits.bounds)
                measured = {m.servo: .55 if i else .2 for i, m in enumerate(model.mappings)}
                x1 = model.mappings[0].servo
                self.engine.update_input((0., 0.), set())
                self.engine.start(model, measured, hardware=True)
                self.report(measured)
                self.step()
                self.step((1., 0.), {"button:0"})
                self.manager.publish_joint_command.assert_not_called()
                self.step(pressed={"button:0"})
                self.manager.publish_joint_command.assert_called_once_with([x1], [0.], 100, 300)
                pending = self.engine.pending
                self.step(pressed={"button:0"})
                self.manager.publish_joint_command.assert_called_once()
                self.manager.command_result.emit(pending, True, "published")
                self.step(pressed={"button:0"})
                self.assertFalse(self.engine.x1_centered)
                measured[x1] = 0.
                self.report(measured)
                self.step(pressed={"button:0"})
                self.assertTrue(self.engine.x1_centered)
                origin = model.foot_position(measured)
                for axes in ((1., -1.), (-1., 1.)):
                    self.step(axes, {"button:0"})
                    names, positions, _speed, _torque = self.manager.publish_joint_command.call_args.args
                    commanded = dict(zip(names, positions))
                    self.assertEqual(commanded[x1], 0.)
                    self.assertEqual(set(commanded), set(measured))
                    self.assertTrue(model.in_limits(commanded))
                    np.testing.assert_allclose(model.foot_position(commanded)[1], origin[1], atol=1e-6)
                    self.manager.command_result.emit(self.engine.pending, True, "published")
                    self.report(commanded)
                self.engine.stop()

    def test_live_rejects_zero_x1_outside_hardware_limits(self):
        self.model.bounds = dict(self.model.bounds)
        self.model.bounds[self.model.mappings[0].servo] = (.1, 1.)
        with self.assertRaisesRegex(ValueError, "0 rad.*EEPROM limits"):
            self.engine.start(self.model, self.seed, hardware=True)
        self.manager.publish_joint_command.assert_not_called()

    def test_releasing_controller_button_cancels_x1_centering(self):
        measured = dict(self.seed, servo_11=.2)
        self.engine.start(self.model, self.seed, hardware=True)
        self.report(measured)
        self.step(pressed={"button:0"})
        pending = self.engine.pending
        self.assertIsNotNone(pending)
        self.step()
        self.manager.cancel_commands.assert_called_once_with([pending])
        self.assertIsNone(self.engine.pending)
        self.assertFalse(self.engine.x1_commanded)

    def test_live_has_one_pending_command_and_release_cancels_it(self):
        self.arm_live()
        self.step((1., 0.), {"button:0"})
        pending = self.engine.pending
        for _ in range(4):
            self.step((1., 0.), {"button:0"})
        self.manager.publish_joint_command.assert_called_once()
        self.step((1., 0.))
        self.manager.cancel_commands.assert_called_once_with([pending])
        self.assertIsNone(self.engine.pending)
        self.step((1., 0.), {"button:0"})
        self.manager.publish_joint_command.assert_called_once()
        self.step(pressed={"button:0"})
        self.step((1., 0.), {"button:0"})
        self.assertEqual(self.manager.publish_joint_command.call_count, 2)

    def test_timeout_failure_disconnect_torque_release_and_limits_stop_live(self):
        for cause in ("timeout", "failure", "controller", "axis_count", "nan", "feedback", "torque", "limits", "other"):
            with self.subTest(cause=cause):
                self.engine.update_input((0., 0.), set())
                self.arm_live()
                self.step((1., 0.), {"button:0"})
                pending = self.engine.pending
                if cause == "timeout":
                    self.now += 1.1
                    self.engine.update_input((1., 0.), {"button:0"})
                    self.engine.tick()
                elif cause == "failure":
                    self.manager.command_result.emit(pending, False, "failed")
                elif cause == "controller":
                    self.engine.update_input((), set())
                elif cause == "axis_count":
                    self.engine.update_input((0.,), set())
                elif cause == "nan":
                    self.engine.update_input((float("nan"), 0.), set())
                elif cause == "feedback":
                    self.now += 2.1
                    self.engine.update_input((1., 0.), {"button:0"})
                    self.engine.tick()
                elif cause == "torque":
                    self.manager.torque_release_requested.emit()
                elif cause == "limits":
                    self.engine.limits.changed.emit(self.engine.limits.bounds)
                else:
                    self.manager.position_published.emit("another-control", 0)
                self.assertFalse(self.engine.active)
                self.assertIsNone(self.engine.pending)
                self.manager.release.assert_called_with("joystick-feedback")

    def test_acknowledgement_retains_bounded_motion_time_while_waiting(self):
        self.arm_live()
        self.step((1., 0.), {"button:0"})
        first = self.engine.pending
        for _ in range(4):
            self.step((1., 0.), {"button:0"})
        self.manager.position_published.emit(first, 0)
        self.assertTrue(self.engine.active)
        self.manager.command_result.emit(first, True, "published")
        before = self.engine.target.copy()
        self.step((1., 0.), {"button:0"}, feedback=True)
        self.assertEqual(self.manager.publish_joint_command.call_count, 2)
        self.assertAlmostEqual(np.linalg.norm(self.engine.target-before), .0025, delta=1e-6)

    def test_live_sub_tick_motion_accumulates_until_quantized_servos_move(self):
        def quantize(pose):
            return {s: float(np.round(q/AX_TICK_RADIANS)*AX_TICK_RADIANS) for s, q in pose.items()}

        self.seed = quantize(self.seed)
        self.arm_live()
        measured = dict(self.seed)
        initial = self.model.foot_position(measured)
        for _ in range(60):
            self.report(measured)
            self.step((0., -.25), {"button:0"})
            if self.engine.pending:
                measured = quantize(self.engine.pose)
                self.manager.command_result.emit(self.engine.pending, True, "published")
        actual = self.model.foot_position(measured)
        self.assertTrue(self.engine.active)
        self.assertGreater(actual[2]-initial[2], .002)
        np.testing.assert_allclose(self.engine.target-initial, [0, 0, .01*(.25-.15)/.85*3], atol=1e-10)

    def test_live_joint_speeds_reduce_stop_time_between_waypoints(self):
        self.arm_live()
        unit = .111*2*np.pi/60
        previous = dict(self.seed)
        idle, ceiling_idle = 0., 0.
        for _ in range(20):
            self.report(previous)
            self.step((0., -1.), {"button:0"})
            names, positions, speeds, _torque = self.manager.publish_joint_command.call_args.args
            self.assertEqual(positions[0], 0.)
            self.assertEqual(speeds[0], 1)
            self.assertTrue(all(1 <= speed <= 100 for speed in speeds))
            # An ideal speed-limited actuator reaches a nearby target and then
            # sits idle. Compare that dwell to the former fixed AX 100 speed.
            travel = [abs(position-previous[name]) for name, position in zip(names, positions)]
            idle += max(0., .05-max(d/(v*unit) for d, v in zip(travel, speeds)))
            ceiling_idle += max(0., .05-max(travel)/(100*unit))
            previous = dict(zip(names, positions))
            self.manager.command_result.emit(self.engine.pending, True, "published")
        self.assertLess(idle, ceiling_idle*.5)
        np.testing.assert_allclose(self.model.foot_position(previous)-self.model.foot_position(self.seed),
                                   [0, 0, .01], atol=1e-6)

    def test_live_tracking_wait_is_bounded_and_reversal_uses_measured_pose(self):
        self.arm_live()
        measured = dict(self.seed)
        for _ in range(60):
            self.report(measured)
            self.step((0., -1.), {"button:0"})
            if self.engine.pending:
                self.manager.command_result.emit(self.engine.pending, True, "published")
        count = self.manager.publish_joint_command.call_count
        self.assertLess(count, 60)
        for _ in range(5):
            self.report(measured)
            self.step((0., -1.), {"button:0"})
        self.assertEqual(self.manager.publish_joint_command.call_count, count)
        self.step((0., 1.), {"button:0"})
        self.assertEqual(self.manager.publish_joint_command.call_count, count+1)
        np.testing.assert_allclose(self.engine.target-self.model.foot_position(measured),
                                   [0, 0, -.0005], atol=1e-12)


class JoystickPanelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.settings = QSettings(str(Path(self.temp.name)/"joystick.ini"), QSettings.Format.IniFormat)
        self.manager, self.monitor = FakeManager(), FakeMonitor()
        self.tab = SingleLegTestTab(self.manager, joystick_monitor=self.monitor,
                                    joystick_settings=self.settings)
        self.addCleanup(dispose, self.manager)
        self.addCleanup(dispose, self.tab)
        self.addCleanup(self.tab.close)

    def test_setup_is_optional_and_default_axes_can_preview(self):
        panel = self.tab.joystick
        self.assertTrue(panel.setup.isHidden())
        panel.detect()
        self.assertEqual(panel.horizontal_axis.currentData(), 0)
        self.assertEqual(panel.vertical_axis.currentData(), 1)
        self.tab.start_joystick(False)
        self.assertTrue(panel.engine.active)
        panel.setup_toggle.setChecked(True)
        self.assertFalse(panel.setup.isHidden())
        panel.deadzone.setValue(20)
        self.assertFalse(panel.engine.active)

    def test_axis_settings_restore_per_device_and_changing_them_stops_preview(self):
        panel = self.tab.joystick
        self.settings.setValue("profiles/test-pad", '{"horizontal":1}')
        panel.detect()
        panel.horizontal_axis.setCurrentIndex(2)
        panel.vertical_axis.setCurrentIndex(3)
        panel.reverse_horizontal.setChecked(True)
        self.tab.start_joystick(False)
        self.assertTrue(panel.engine.active)
        self.assertEqual(panel.engine.horizontal, 0)
        panel.speed.setValue(5)
        self.assertFalse(panel.engine.active)
        self.monitor.select(None)
        self.monitor.select(42)
        self.assertEqual(panel.horizontal_axis.currentData(), 2)
        self.assertEqual(panel.vertical_axis.currentData(), 3)
        self.assertTrue(panel.reverse_horizontal.isChecked())
        self.assertEqual(panel.speed.value(), 5)
        self.manager.publish_joint_command.assert_not_called()

    def test_preview_updates_mesh_and_hiding_tab_stops_control(self):
        panel = self.tab.joystick
        panel.detect()
        self.tab.start_joystick(False)
        engine = panel.engine
        engine.clock = lambda: 100.
        engine.last_tick = 99.95
        engine.update_input((0., 0.), set())
        engine.tick()
        initial = self.tab.view._foot.copy()
        engine.last_tick = 99.95
        engine.update_input((0., -1.), set())
        engine.tick()
        self.assertGreater(self.tab.view._foot[2], initial[2])
        self.tab.show()
        self.tab.hide()
        self.assertFalse(engine.active)
        self.assertFalse(self.monitor.closed)
        self.tab.close()
        self.assertTrue(self.monitor.closed)
        self.manager.publish_joint_command.assert_not_called()

    def test_controller_panel_is_to_the_right_of_the_view(self):
        self.tab.resize(1450, 1000)
        self.tab.show()
        _APP.processEvents()
        splitter = self.tab.findChild(QSplitter)
        self.assertEqual(splitter.count(), 3)
        self.assertIsInstance(splitter.widget(2), QScrollArea)
        self.assertIs(splitter.widget(2).widget(), self.tab.joystick)
        self.assertGreater(self.tab.joystick.mapTo(self.tab, QPoint()).x(),
                           self.tab.view.mapTo(self.tab, QPoint()).x()+self.tab.view.width())


if __name__ == "__main__":
    unittest.main()
