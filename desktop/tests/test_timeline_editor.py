"""Offline document and Qt integration checks; no ROS installation is required."""

import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtTest import QTest
from limit_fixtures import prime_limits
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
)

from desktop.tab_workspace import WorkspaceMainWindow
from desktop.tabs.timeline import TimeLineTab
from desktop.tabs.timeline.backend import TimelineDocument, motion_settings


class TimelineFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.snapshots = self.root / "snapshots"
        self.sequences = self.root / "sequences"
        self.snapshots.mkdir()
        self.sequences.mkdir()
        self.pose = {
            "schema_version": 1, "name": "pose",
            "positions": {"servo_11": 0.1, "servo_12": -0.2, "servo_31": 0.3},
        }
        self.snapshot = self.snapshots / "pose.json"
        self.snapshot.write_text(json.dumps(self.pose), encoding="utf-8")
        self.document = TimelineDocument(self.snapshots)
        self.target = self.sequences / "timeline.json"


class DocumentTest(TimelineFixture):
    def test_snapshot_targets_are_clamped_when_bounds_are_available(self):
        bounds = {
            "servo_11": (-0.2, 0.6),
            "servo_12": (-0.5, 0.5),
            "servo_31": (-0.5, 0.5),
        }
        self.pose["positions"]["servo_11"] = 2.0
        self.snapshot.write_text(json.dumps(self.pose), encoding="utf-8")
        document = TimelineDocument(self.snapshots, bounds=bounds)
        document.add_snapshot(self.snapshot)
        self.assertEqual(document.snapshot(0)["positions"]["servo_11"], 0.6)

    def test_snapshot_directory_expands_home_on_creation_and_new(self):
        with patch.dict(os.environ, {"HOME": str(self.root), "USERPROFILE": str(self.root)}):
            document = TimelineDocument("~/snapshots")
            self.assertEqual(document.snapshot_directory, self.snapshots.resolve())
            document.add_snapshot(self.snapshot)
            self.assertEqual(document.steps[0]["snapshot"], "pose.json")
            document.new("~/snapshots")
            self.assertEqual(document.snapshot_directory, self.snapshots.resolve())
            document.add_snapshot(self.snapshot)
            self.assertEqual(document.snapshot(0)["positions"], self.pose["positions"])

    def test_round_trip_preserves_references_motion_wait_and_lifecycle(self):
        document = self.document
        document.add_snapshot(self.snapshot)
        document.update_duration(0, "seconds", 0.125)
        document.set_control_mode(0, "Custom Control")
        document.update_motion(0, "Custom Control", "servo_11", "speed", 241)
        document.add_wait(1, 0.75)
        document.set_lifecycle("off", True)
        document.set_smooth_playback(True)
        self.assertTrue(document.dirty)
        expected = copy.deepcopy(document.steps)
        document.save(self.target)
        saved = json.loads(self.target.read_text(encoding="utf-8"))
        self.assertEqual(saved["schema_version"], 2)
        self.assertEqual(saved["steps"][0]["snapshot"], "pose.json")
        self.assertFalse(document.dirty)
        restored = TimelineDocument(self.snapshots)
        restored.load(self.target)
        self.assertEqual(restored.steps, expected)
        self.assertEqual((restored.on_end, restored.loop), ("off", True))
        self.assertTrue(restored.smooth_playback)
        self.assertEqual(restored.path, self.target.resolve())
        self.assertFalse(restored.dirty)

    def test_smooth_setting_marks_document_dirty_and_rejects_invalid_value(self):
        self.document.add_snapshot(self.snapshot)
        self.document.save(self.target)
        self.document.set_smooth_playback(True)
        self.assertTrue(self.document.dirty)
        self.document.save()
        data = json.loads(self.target.read_text(encoding="utf-8"))
        data["smooth_playback"] = "true"
        self.target.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.document.load(self.target)
        self.assertTrue(self.document.smooth_playback)

    def test_invalid_load_keeps_existing_state_and_reference_base(self):
        self.document.add_snapshot(self.snapshot)
        original = copy.deepcopy(self.document.steps)
        self.target.write_text('{"kind": "broken"}', encoding="utf-8")
        with self.assertRaises(ValueError):
            self.document.load(self.target)
        self.assertEqual(self.document.steps, original)
        self.assertTrue(self.document.dirty)
        self.assertIsNone(self.document.path)
        self.assertEqual(self.document.snapshot_directory, self.snapshots.resolve())

    def test_legacy_embedded_pose_migrates_without_changing_source(self):
        legacy = {
            "kind": "saprog_timeline", "schema_version": 1,
            "steps": [{"snapshot": self.pose, "motion": {"speed": 300, "torque": 500}}],
        }
        self.target.write_text(json.dumps(legacy), encoding="utf-8")
        self.document.load(self.target)
        self.assertEqual(self.document.steps[0]["snapshot"], "pose.json")
        self.assertEqual(json.loads(self.target.read_text())["schema_version"], 1)
        self.document.save()
        self.assertEqual(json.loads(self.target.read_text())["schema_version"], 2)

    def test_missing_reference_cannot_overwrite_saved_file(self):
        self.document.add_snapshot(self.snapshot)
        self.document.save(self.target)
        original = self.target.read_bytes()
        self.document.update_duration(0, "seconds", 2)
        self.snapshot.unlink()
        with self.assertRaises(OSError):
            self.document.save()
        self.assertEqual(self.target.read_bytes(), original)
        self.assertTrue(self.document.dirty)

    def test_failed_atomic_replace_keeps_file_and_dirty_state(self):
        self.document.add_wait(0, 0.5)
        self.document.save(self.target)
        original = self.target.read_bytes()
        self.document.update_wait(0, 2.25)
        with patch("documents.snapshot_files.os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                self.document.save()
        self.assertEqual(self.target.read_bytes(), original)
        self.assertTrue(self.document.dirty)
        self.assertEqual(list(self.sequences.glob("*.tmp")), [])

    def test_failed_save_with_absolute_references_keeps_document_and_file(self):
        self.document.add_snapshot(self.snapshot)
        self.document.save(self.target)
        self.document.update_duration(0, "seconds", 2)
        original_steps = copy.deepcopy(self.document.steps)
        original_file = self.target.read_bytes()
        with patch("documents.snapshot_files.os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                self.document.save(absolute_references=True)
        self.assertEqual(self.document.steps, original_steps)
        self.assertEqual(self.document.snapshot_directory, self.snapshots.resolve())
        self.assertEqual(self.document.path, self.target.resolve())
        self.assertTrue(self.document.dirty)
        self.assertEqual(self.target.read_bytes(), original_file)

    def test_reordering_and_duplicate_are_whole_independent_steps(self):
        self.document.add_snapshot(self.snapshot)
        self.document.add_wait(1, 0.25)
        duplicate = self.document.presenter.duplicate(0)
        self.document.update_duration(duplicate, "seconds", 3)
        self.assertNotIn("duration", self.document.steps[0])
        self.assertEqual(self.document.presenter.move(duplicate, 1), 2)
        self.assertEqual(self.document.steps[1]["kind"], "wait")
        self.document.presenter.remove(0)
        self.assertEqual(len(self.document.steps), 2)
        self.assertEqual(self.document.steps[1]["duration"]["seconds"], 3)

    def test_switching_motion_modes_preserves_then_averages_values(self):
        self.document.add_snapshot(self.snapshot)
        self.document.set_control_mode(0, "Custom Control")
        self.document.update_motion(0, "Custom Control", "servo_11", "speed", 101)
        self.document.update_motion(0, "Custom Control", "servo_12", "speed", 201)
        self.document.set_control_mode(0, "Leg Control")
        resolved = motion_settings(self.document.steps[0], self.pose["positions"])
        self.assertEqual(resolved["servo_11"]["speed"], 151)
        self.assertEqual(resolved["servo_12"]["speed"], 151)
        self.assertEqual(resolved["servo_31"]["speed"], 1023)
        self.document.set_control_mode(0, "Global Control")
        self.assertEqual(self.document.steps[0]["motion"]["speed"], 442)
        self.assertEqual(self.document.steps[0]["overrides"], {})

    def test_invalid_values_and_snapshots_do_not_mutate_document(self):
        self.document.add_wait(0)
        original = copy.deepcopy(self.document.steps)
        for seconds in (-1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                self.document.update_wait(0, seconds)
        self.snapshot.write_text('{"schema_version": 99}', encoding="utf-8")
        with self.assertRaises(ValueError):
            self.document.add_snapshot(self.snapshot)
        self.assertEqual(self.document.steps, original)


class TimelineUiTest(TimelineFixture):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        super().setUp()
        self.widget = TimeLineTab(
            snapshot_directory=self.snapshots, sequence_directory=self.sequences
        )
        prime_limits(self.widget.playback.limits)
        self.addCleanup(self.widget.deleteLater)

    def click(self, text):
        button = next(b for b in self.widget.findChildren(QPushButton) if b.text() == text)
        button.click()

    def test_source_load_edit_save_and_reload_through_controls(self):
        widget = self.widget
        self.assertTrue(widget.load_source(1))
        self.assertEqual(len(widget.document.steps), 1)
        widget.motion_controls["All servos"]["speed"].setValue(321)
        widget.duration_picker.setCurrentIndex(1)
        widget.duration_seconds.setValue(0.625)
        self.click("Add wait")
        widget.wait_seconds.setValue(1.75)
        self.assertIn("1.75", widget.steps_list.currentItem().text())
        widget.end_action.setCurrentIndex(1)
        widget.loop_toggle.setChecked(True)
        with patch("desktop.tabs.timeline.QFileDialog.getSaveFileName", return_value=(str(self.target), "")):
            next(b for b in widget.findChildren(QPushButton) if b.accessibleName() == "Save timeline").click()
        self.assertFalse(widget.document.dirty)
        self.assertEqual(widget.source_controls[0][0].currentData(), str(self.target))
        expected = copy.deepcopy(widget.document.steps)
        self.assertTrue(widget.new_timeline())
        self.assertEqual(widget.document.steps, [])
        self.assertTrue(widget.load_source(0))
        self.assertEqual(widget.document.steps, expected)
        self.assertEqual(widget.motion_controls["All servos"]["speed"].value(), 321)
        self.assertTrue(widget.loop_toggle.isChecked())
        self.assertEqual(widget.end_action.currentIndex(), 1)
        self.assertFalse(widget.document.dirty)

    def test_motion_mode_and_whole_step_buttons(self):
        self.widget.load_source(1)
        self.widget.mode_picker.setCurrentText("Custom Control")
        self.widget.motion_controls["servo_12"]["torque"].setValue(410)
        self.click("Duplicate")
        self.assertEqual(len(self.widget.document.steps), 2)
        self.click("Up")
        self.assertEqual(self.widget._selected()[0], 0)
        self.click("Remove")
        self.assertEqual(len(self.widget.document.steps), 1)
        self.assertEqual(self.widget.document.steps[0]["overrides"]["servo_12"]["torque"], 410)

    def test_unsaved_cancel_and_save_before_new(self):
        self.widget.add_wait()
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Cancel):
            self.assertFalse(self.widget.new_timeline())
            self.assertEqual(len(self.widget.document.steps), 1)
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Save):
            with patch("desktop.tabs.timeline.QFileDialog.getSaveFileName", return_value=("", "")):
                self.assertFalse(self.widget.new_timeline())
            with patch("desktop.tabs.timeline.QFileDialog.getSaveFileName", return_value=(str(self.target), "")):
                self.assertTrue(self.widget.new_timeline())
        self.assertTrue(self.target.is_file())
        self.assertEqual(self.widget.document.steps, [])

    def test_failed_ui_load_and_save_keep_edited_state(self):
        self.widget.add_wait()
        self.target.write_text("{invalid", encoding="utf-8")
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Discard):
            self.assertFalse(self.widget.load_timeline(self.target))
        self.assertEqual(len(self.widget.document.steps), 1)
        self.assertTrue(self.widget.document.dirty)
        self.assertIn("Could not load", self.widget.status_label.text())
        original = self.target.read_bytes()
        with patch("documents.snapshot_files.os.replace", side_effect=OSError("disk error")):
            self.assertFalse(self.widget.save_timeline(self.target))
        self.assertEqual(self.target.read_bytes(), original)
        self.assertTrue(self.widget.document.dirty)
        self.assertIn("Could not save", self.widget.status_label.text())

    def test_browser_changes_do_not_rebase_open_snapshot_references(self):
        self.widget.load_source(1)
        other = self.root / "other"
        other.mkdir()
        other_pose = copy.deepcopy(self.pose)
        other_pose["positions"]["servo_11"] = 100
        (other / "pose.json").write_text(json.dumps(other_pose), encoding="utf-8")
        self.widget.source_controls[1][1].setText(str(other))
        self.widget.refresh_sources()
        self.widget.load_source(1)
        self.assertEqual(self.widget.document.snapshot(0)["positions"]["servo_11"], 0.1)
        self.assertAlmostEqual(
            self.widget.document.snapshot(1)["positions"]["servo_11"],
            self.widget.playback.bounds["servo_11"][1],
        )
        self.assertTrue(self.widget.save_timeline(self.target))
        self.document.load(self.target)
        self.assertEqual(self.document.snapshot(0)["positions"]["servo_11"], 0.1)
        self.assertEqual(self.document.snapshot(1)["positions"]["servo_11"], 100)
        self.assertTrue(self.widget.load_timeline(self.target))
        self.assertEqual(self.widget.document.snapshot(0)["positions"]["servo_11"], 0.1)
        self.assertAlmostEqual(
            self.widget.document.snapshot(1)["positions"]["servo_11"],
            self.widget.playback.bounds["servo_11"][1],
        )
        self.assertTrue(Path(self.widget.document.steps[0]["snapshot"]).is_absolute())
        self.assertTrue(Path(self.widget.document.steps[1]["snapshot"]).is_absolute())

    def test_snapshot_joint_changes_show_error_without_losing_document(self):
        self.widget.load_source(1)
        self.widget.mode_picker.setCurrentText("Custom Control")
        self.widget.motion_controls["servo_12"]["speed"].setValue(241)
        self.assertTrue(self.widget.save_timeline(self.target))
        original_steps = copy.deepcopy(self.widget.document.steps)
        original_file = self.target.read_bytes()
        self.widget.steps_list.setCurrentRow(1)
        changed_pose = copy.deepcopy(self.pose)
        del changed_pose["positions"]["servo_12"]
        self.snapshot.write_text(json.dumps(changed_pose), encoding="utf-8")
        with patch.object(sys, "excepthook") as exception_handler:
            self.widget.steps_list.setCurrentRow(0)
        exception_handler.assert_not_called()
        content = self.widget.editor_scroll.widget()
        self.assertTrue(any(
            "Unknown servo in transition overrides" in label.text()
            for label in content.findChildren(QLabel)
        ))
        self.assertEqual(self.widget.document.steps, original_steps)
        self.assertFalse(self.widget.document.dirty)
        self.assertFalse(self.widget.save_timeline())
        self.assertEqual(self.target.read_bytes(), original_file)
        self.snapshot.write_text(json.dumps(self.pose), encoding="utf-8")
        self.widget.steps_list.setCurrentRow(1)
        self.widget.steps_list.setCurrentRow(0)
        self.assertEqual(self.widget.motion_controls["servo_12"]["speed"].value(), 241)

    def test_home_snapshot_folder_browses_and_loads_relative_references(self):
        self.document.add_snapshot(self.snapshot)
        self.document.save(self.target)
        with patch.dict(os.environ, {"HOME": str(self.root), "USERPROFILE": str(self.root)}):
            self.widget.source_controls[1][1].setText("~/snapshots")
            self.widget.refresh_sources()
            self.assertEqual(self.widget.source_controls[1][0].count(), 1)
            self.assertTrue(self.widget.load_timeline(self.target))
            self.assertEqual(self.widget.document.snapshot_directory, self.snapshots.resolve())
            self.assertEqual(self.widget.document.snapshot(0)["positions"], self.pose["positions"])

    def test_pending_duration_prompts_on_close_and_save_preserves_it(self):
        window = QMainWindow()
        window.setCentralWidget(self.widget)
        self.addCleanup(window.deleteLater)
        for kind in ("wait", "transition"):
            with self.subTest(kind=kind):
                self.widget.document.new()
                self.widget._redraw()
                window.show()
                self.app.processEvents()
                if kind == "wait":
                    self.widget.add_wait()
                    spin = self.widget.wait_seconds
                else:
                    self.widget.load_source(1)
                    self.widget.duration_picker.setCurrentIndex(1)
                    spin = self.widget.duration_seconds
                self.assertTrue(self.widget.save_timeline(self.target))
                spin.setFocus()
                spin.selectAll()
                QTest.keyClicks(spin, "3")
                self.assertFalse(self.widget.document.dirty)
                with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Cancel) as prompt:
                    self.assertFalse(window.close())
                    prompt.assert_called_once()
                self.assertTrue(self.widget.document.dirty)
                self.assertTrue(window.isVisible())
                with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Save):
                    self.assertTrue(window.close())
                saved = json.loads(self.target.read_text(encoding="utf-8"))["steps"][0]
                duration = saved if kind == "wait" else saved["duration"]
                self.assertEqual(duration["seconds"], 3)
                self.assertFalse(self.widget.document.dirty)

    def test_direct_save_commits_pending_duration(self):
        self.widget.add_wait()
        self.assertTrue(self.widget.save_timeline(self.target))
        spin = self.widget.wait_seconds
        spin.selectAll()
        QTest.keyClicks(spin, "3")
        self.assertFalse(self.widget.document.dirty)
        self.assertTrue(self.widget.save_timeline())
        saved = json.loads(self.target.read_text(encoding="utf-8"))
        self.assertEqual(saved["steps"][0]["seconds"], 3)
        self.assertFalse(self.widget.document.dirty)

    def test_loading_and_selection_do_not_modify_existing_overrides_or_precision(self):
        self.document.add_snapshot(self.snapshot)
        self.document.set_control_mode(0, "Custom Control")
        self.document.update_motion(0, "Custom Control", "servo_11", "speed", 241)
        self.document.update_duration(0, "seconds", 0.123456789)
        self.document.save(self.target)
        expected = copy.deepcopy(self.document.steps)
        self.assertTrue(self.widget.load_timeline(self.target))
        self.widget.steps_list.setCurrentRow(1)
        self.widget.steps_list.setCurrentRow(0)
        self.assertEqual(self.widget.document.steps, expected)
        self.assertFalse(self.widget.document.dirty)
        self.assertTrue(self.widget.save_timeline())
        self.document.load(self.target)
        self.assertEqual(self.document.steps, expected)

    def test_new_save_adds_json_extension_for_source_browser(self):
        self.widget.add_wait()
        destination = self.sequences / "new_timeline"
        with patch("desktop.tabs.timeline.QFileDialog.getSaveFileName", return_value=(str(destination), "")):
            self.assertTrue(self.widget.save_timeline())
        expected = destination.with_suffix(".json")
        self.assertEqual(self.widget.document.path, expected)
        self.assertEqual(self.widget.source_controls[0][0].currentData(), str(expected))

    def test_detached_tab_guards_workspace_close_without_prompt_on_restore(self):
        window = WorkspaceMainWindow()
        window.add_tab(self.widget, "Timeline")
        window.show()
        self.app.processEvents()
        self.widget.add_wait()
        window.detach_tab(0)
        self.app.processEvents()
        detached = self.widget.window()
        self.assertIsNot(detached, window)
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Cancel) as prompt:
            self.assertFalse(window.close())
            self.assertEqual(prompt.call_count, 1)
            prompt.reset_mock()
            self.assertTrue(detached.close())
            self.app.processEvents()
            prompt.assert_not_called()
        self.assertIs(self.widget.window(), window)
        self.assertTrue(self.widget.document.dirty)
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Discard):
            self.assertTrue(window.close())
        window.deleteLater()

    def test_tab_entry_refresh_and_main_window_unsaved_prompt(self):
        window = QMainWindow()
        tabs = QTabWidget()
        window.setCentralWidget(tabs)
        tabs.addTab(QTabWidget(), "Other")
        tabs.addTab(self.widget, "Timeline")
        window.show()
        self.app.processEvents()
        extra = self.snapshots / "extra.json"
        extra.write_text(json.dumps(self.pose), encoding="utf-8")
        tabs.setCurrentWidget(self.widget)
        self.app.processEvents()
        self.assertEqual(self.widget.source_controls[1][0].count(), 2)
        self.widget.add_wait()
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Cancel):
            self.assertFalse(window.close())
        self.assertTrue(window.isVisible())
        with patch("desktop.tabs.timeline.QMessageBox.question", return_value=QMessageBox.StandardButton.Discard):
            self.assertTrue(window.close())
        window.deleteLater()


if __name__ == "__main__":
    unittest.main()
