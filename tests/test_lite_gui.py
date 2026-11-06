"""Exercise Lite's Tk integration without hardware or persistent user state."""
import gc
import importlib.util
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

from test_lite_serial_manager import FakePort

SOURCE = Path(__file__).resolve().parents[1] / 'software'
spec = importlib.util.spec_from_file_location('opendrive_lite_gui', SOURCE / 'opendrive_gui.py')
gui = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = gui
spec.loader.exec_module(gui)


class LiteGuiTests(unittest.TestCase):
    def setUp(self):
        self.ports = []
        self.open_gate = None
        self.open_started = threading.Event()
        self.patches = [
            patch.object(gui.OpenDriveApp, '_load_state'),
            patch.object(gui.OpenDriveApp, '_save_state'),
            patch('serial_manager.serial.Serial', side_effect=self.open_port),
            patch.object(gui.messagebox, 'showwarning'),
            patch.object(gui.messagebox, 'showinfo'),
            patch.object(gui.messagebox, 'showerror'),
        ]
        for item in self.patches:
            item.start()
        self.app = gui.OpenDriveApp()
        self.callback_errors = []
        self.app.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.app.root.withdraw()
        self.app._port_var.set('FAKE')

    def open_port(self, **kwargs):
        self.open_started.set()
        if self.open_gate:
            self.open_gate.wait(2)
        port = FakePort(**kwargs)
        self.ports.append(port)
        return port

    def pump_until(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            self.app.root.update()
            if time.monotonic() > deadline:
                self.fail('Timed out waiting for Tk/serial notification')
            time.sleep(.002)
        self.app.root.update()

    def connect(self):
        self.app._on_connect()
        self.pump_until(lambda: self.app._connect_btn.cget('text') == 'Disconnect')
        return self.ports[-1]

    def tearDown(self):
        if self.open_gate:
            self.open_gate.set()
        for port in self.ports:
            port.release.set()
        self.app._on_close()
        self.pump_until(lambda: not self.app.serial_manager.is_busy)
        # Complete the scheduled shutdown before removing patches.
        try:
            self.app._finish_close()
        except gui.tk.TclError:
            pass
        for item in reversed(self.patches):
            item.stop()
        self.assertEqual(self.callback_errors, [])
        # Collect destroyed Tk objects on their owner thread before the next
        # serial worker can trigger cyclic garbage collection.
        self.app = None
        gc.collect()

    def test_opening_port_keeps_tk_responsive(self):
        self.open_gate = threading.Event()
        start = time.monotonic()
        self.app._on_connect()
        self.assertLess(time.monotonic() - start, .1)
        self.assertTrue(self.open_started.wait(.5))
        ticks = []
        self.app.root.after(0, lambda: ticks.append(True))
        self.pump_until(lambda: bool(ticks))
        self.assertEqual(self.app._connect_btn.cget('text'), 'Connecting...')
        self.open_gate.set()
        self.pump_until(lambda: self.app._connect_btn.cget('text') == 'Disconnect')

    def test_position_notifications_reach_tk(self):
        port = self.connect()
        port.inject(gui.SerialProtocol.create_command('POS', 1, '12.345'))
        self.pump_until(lambda: self.app._get_widgets(1)['absolute_var'].get() == '12.345')
        self.assertEqual(self.app.motor_states[0].absolute, 12.345)

    def test_stalled_run_leaves_controls_responsive(self):
        port = self.connect()
        state = self.app.motor_states[0]
        state.is_selected = True
        widgets = self.app._get_widgets(1)
        widgets['speed_var'].set('0.02')
        widgets['target_var'].set('10')
        port.block_command = 'RUN'
        self.app._on_run()
        self.assertTrue(port.entered.wait(.5))
        ticks = []
        self.app.root.after(0, lambda: ticks.append(True))
        self.pump_until(lambda: bool(ticks))
        start = time.monotonic()
        self.app._on_pause()
        self.assertLess(time.monotonic() - start, .1)
        self.assertFalse(state.is_running)

    def test_firmware_warning_identifies_update_and_correct_sketch(self):
        for reply in [None, (gui.Config.FIRMWARE_PRODUCT, 'full', gui.Config.FIRMWARE_VERSION)]:
            with self.subTest(reply=reply), patch.object(FakePort, 'firmware_reply', reply), \
                    patch.object(gui.Config, 'FIRMWARE_CHECK_TIMEOUT', .12), \
                    patch.object(gui.messagebox, 'showwarning') as warning:
                self.app._on_connect()
                self.pump_until(lambda: warning.called)
                self.pump_until(lambda: not self.app.serial_manager.is_busy)
                title, message = warning.call_args.args
                self.assertEqual(title, 'Firmware Update Required')
                self.assertIn(gui.Config.FIRMWARE_SKETCH, message)
                self.assertIn('Movement is disabled', message)
                self.assertFalse(self.app.serial_manager.is_connected)
                self.assertEqual(self.app._connect_btn.cget('text'), 'Connect')
                self.assertEqual(warning.call_count, 1)

    def test_failed_open_restores_connect_button(self):
        with patch('serial_manager.serial.Serial', side_effect=OSError('simulated unplug')):
            self.app._on_connect()
            self.pump_until(lambda: self.app._connect_btn.cget('text') == 'Connect')
        self.assertFalse(self.app.serial_manager.is_connected)


if __name__ == '__main__':
    unittest.main()
