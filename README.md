# OpenDrive Lite

A minimal OpenDrive GUI and matching console firmware for controlling up to six microdrives. Use Lite for basic motor control, position tracking, notes, and CSV export. It does not include the primary codebase's MRI visualization, SpikeGLX integrations, and other advanced features.

The [primary OpenDrive codebase](https://github.com/jazlab/opendrive) remains the recommended starting point for the full system. Lite is a separate, self-contained alternative; use its GUI with the firmware in this repository. See the [project website](https://jazlab.github.io/opendrive/) for more information.

## Install the GUI

Use a desktop Python installation with Tkinter (Python 3.12 is a suitable starting point). `python -m tkinter` should open a small test window. If Tkinter is missing, install the Tk package appropriate for your Python distribution; it is not installed by pip.

Download or clone this repository, then install and launch from its `software/` directory:

```bash
git clone https://github.com/jazlab/opendrive_lite.git
cd opendrive_lite/software
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python opendrive_gui.py
```

On Windows Command Prompt, replace the activation command with `.venv\Scripts\activate`. Use `python3` if that is your Python command. Keep this folder writable: the GUI automatically saves positions, settings, and notes to `software/opendrive_state.json` beside the script.

## Install the console firmware

1. Install the Arduino IDE. In Preferences/Settings, add `https://www.pjrc.com/teensy/package_teensy_index.json` to **Additional boards manager URLs**.
2. In Boards Manager, install the Teensy board support package.
3. Connect the console's Teensy 4.0 by USB. Select **Teensy 4.0** and its port in the IDE.
4. Open `firmware/opendrive_firmware/opendrive_firmware.ino`, keeping it inside the folder of the same name.
5. Disconnect the GUI from the serial port, then upload the sketch. If needed, press the Teensy's programming button as described in the console manual.

The firmware targets the documented OpenDrive console wiring and motor mechanics. It uses Arduino/Teensy core facilities and requires no additional Arduino library in this sketch.

On connection, Lite verifies the firmware's version and Lite variant before
enabling movement. Full-app firmware, incompatible versions, and older firmware
that cannot report a version trigger **Firmware Update Required**, with the
correct sketch path. Upload this directory's matching firmware and reconnect;
previously installed firmware needs this update to pass the check.

## Basic operation

1. With console power disconnected, connect the drives. Connect USB to the computer, then power the assembled console as described in the manual.
2. Launch the GUI, choose the Teensy port, and click **Connect**. Typical names are `COMx` on Windows and `/dev/cu.usbmodem…` on macOS. Reopening the port menu refreshes it.
3. Use **+ / −** to show the connected motor rows (up to six). Row numbers correspond to the console's numbered motor channels.
4. Check the carriage ruler. Double-click **Absolute**, enter its current position in millimeters, and press Enter. Confirm this value before every session; the system has no position encoder.
5. Select a motor's checkbox, enter **Speed** in mm/s and **Target** in absolute mm, then click **Run**. The default limits are 0–50 mm and 0.001–0.2 mm/s. Confirm the intended direction and travel with a short bench movement first.
6. **Pause** stops selected running motors. **Zero** sets selected motors' relative-position reference; it does not move the drive or change the absolute position. **Export** pauses movement and saves the current table to CSV. Notes and state are also saved automatically.

Read the [OpenDrive documentation](https://docs.google.com/document/d/1CwdIq8lv9Gtcey1GMihnH131U_tzeRZlZ0cmyOJbvFY/edit), especially the GUI Manual, Console Manual, and safety instructions for more details. Do not connect/disconnect drives while powered, let a drive hit a travel stop, or leave a running system unattended. Disconnect console power to stop movement in an emergency. Test the complete system on the bench before experimental use.

## Contact

For questions, contact [opendrive@mit.edu](mailto:opendrive@mit.edu).

## Files and license

- `software/opendrive_gui.py`: Python/Tkinter GUI; `pyserial` is its external Python dependency. Keep the accompanying Python files in the same folder.
- `software/serial_manager.py`: background serial worker, bounded command queue, connection lifecycle, and heartbeat monitoring. Device I/O never runs on the GUI thread; reconnect waits for the previous worker to close its port.
- `software/control_protocol.py`: configuration constants and serial packet formatting.
- `firmware/opendrive_firmware/opendrive_firmware.ino`: matching Teensy 4.0 firmware.
- [MIT License](LICENSE): covers the Lite GUI and firmware.

## Development checks

From the repository root, with Lite's Python dependencies installed:

```sh
python -m unittest discover -s tests -v
```

The tests use simulated serial ports and temporary GUI windows, without connecting
to hardware or reading/writing saved application state. Firmware heartbeat tests
compile the actual Lite sketch against simulated Arduino I/O and require GCC,
Clang, or MSVC; they are skipped if no desktop C++ compiler is available.

Firmware requires a valid heartbeat before accepting movement and checks its
expiry before every stepping pass. After a timeout, a fresh heartbeat permits a
new explicit Run command; it never resumes the interrupted movement by itself.
The GUI stops renewing heartbeats if its event loop stalls, and reconnecting
starts a new session rather than restarting old communication threads.
