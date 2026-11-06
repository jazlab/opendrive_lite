#!/usr/bin/env python3
"""
OpenDrive GUI - Control interface for OpenDrive microdrive system.

This GUI allows users to control up to 6 stepper motor-driven microdrives
for precise positioning of neural probes.

Author: OpenDrive Project
License: MIT
"""

import csv
import json
import logging
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional, Callable

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# Configure logging
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# =============================================================================
# Configuration Constants
# =============================================================================

class Config:
    """Global configuration parameters for the OpenDrive system."""

    # Serial communication
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 0.1

    # Motor limits
    MAX_MOTORS: int = 6
    MIN_ABSOLUTE_POS: float = 0.0      # mm
    MAX_ABSOLUTE_POS: float = 50.0     # mm
    MIN_SPEED: float = 0.001           # mm/s (inclusive)
    MAX_SPEED: float = 0.2             # mm/s (inclusive)
    MIN_TARGET_POS: float = 0.0        # mm
    MAX_TARGET_POS: float = 50.0       # mm

    # Timing intervals (seconds)
    HEARTBEAT_INTERVAL: float = 0.5
    HEARTBEAT_TIMEOUT: float = 1.0
    POSITION_UPDATE_INTERVAL: float = 0.2
    AUTO_SAVE_INTERVAL: float = 5.0
    SERIAL_READ_INTERVAL: float = 0.02

    # Data persistence
    DATA_FILE: str = "opendrive_state.json"

    # GUI Layout
    WINDOW_TITLE: str = "OpenDrive Lite"
    DECIMAL_PLACES: int = 3
    ENTRY_WIDTH: int = 12
    COLOR_BG: str = "#f0f0f0"
    COLOR_FG: str = "#000000"
    COLOR_ENTRY_BG: str = "#ffffff"
    COLOR_READONLY_BG: str = "#d9d9d9"
    COLOR_ACTIVE_BG: str = "#e5e5e5"
    COLOR_SELECT_BG: str = "#4a90e2"

    # Column Weights and Min Sizes for Grid Layout
    # Col 0: Buttons/ID | Col 1: Speed | Col 2: Target | Col 3: Abs | Col 4: Rel | Col 5: Notes
    COL_MIN_WIDTHS: tuple = (120, 100, 100, 100, 100, 150)
    ROW_PAD_Y: int = 2
    HEADER_PAD_Y: tuple = (5, 10)


# =============================================================================
# Serial Communication Protocol
# =============================================================================

class SerialProtocol:
    """
    Serial communication protocol between GUI and Teensy console.
    
    All commands use XOR checksum for data integrity verification.
    Format: <command>,<arg1>,<arg2>,...*<checksum>
    """

    @staticmethod
    def calculate_checksum(data: str) -> str:
        """Calculate XOR checksum of data string and return as 2-digit hex."""
        checksum = 0
        for char in data:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    @staticmethod
    def create_command(cmd: str, *args) -> str:
        """Create a command string with checksum."""
        if args:
            data = f"{cmd},{','.join(str(a) for a in args)}"
        else:
            data = cmd
        checksum = SerialProtocol.calculate_checksum(data)
        return f"{data}*{checksum}\n"

    @staticmethod
    def parse_response(response: str) -> Optional[tuple]:
        """
        Parse a response string and verify checksum.
        
        Returns:
            Tuple of (command, [args]) if valid, None if invalid.
        """
        response = response.strip()
        if not response or '*' not in response:
            return None

        try:
            data, received_checksum = response.rsplit('*', 1)
            expected_checksum = SerialProtocol.calculate_checksum(data)

            if received_checksum != expected_checksum:
                return None

            parts = data.split(',')
            return (parts[0], parts[1:])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def cmd_run(motor_id: int, speed: float, target: float, absolute: float) -> str:
        """Create RUN command for motor movement."""
        return SerialProtocol.create_command(
            "RUN", motor_id, f"{speed:.6f}", f"{target:.3f}", f"{absolute:.3f}"
        )

    @staticmethod
    def cmd_pause(motor_id: int) -> str:
        """Create PAUSE command for a single motor."""
        return SerialProtocol.create_command("PAUSE", motor_id)

    @staticmethod
    def cmd_pause_all() -> str:
        """Create PAUSE_ALL command to stop all motors."""
        return SerialProtocol.create_command("PAUSE_ALL")

    @staticmethod
    def cmd_heartbeat() -> str:
        """Create HEARTBEAT command for connection monitoring."""
        return SerialProtocol.create_command("HEARTBEAT")

    @staticmethod
    def cmd_update_target(motor_id: int, target: float) -> str:
        """Create TARGET command to update motor target position."""
        return SerialProtocol.create_command("TARGET", motor_id, f"{target:.3f}")


# =============================================================================
# Motor State Data Class
# =============================================================================

@dataclass
class MotorState:
    """
    State of a single motor/drive.
    
    Attributes:
        motor_id: Unique identifier (1-indexed)
        speed: Movement speed in mm/s
        target: Target position in mm
        absolute: Current absolute position in mm
        relative: Position relative to zero point in mm
        relative_zero_point: Reference point for relative position
        notes: User notes for this motor
        is_running: Whether motor is currently moving
        is_selected: Whether motor is selected in GUI
    """
    motor_id: int
    speed: float = 0.0
    target: float = 0.0
    absolute: float = 0.0
    relative: float = 0.0
    relative_zero_point: float = 0.0
    notes: str = ""
    is_running: bool = False
    is_selected: bool = False

    def to_dict(self) -> dict:
        """Convert state to dictionary for serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> 'MotorState':
        """Create MotorState from dictionary."""
        return cls(**data)


# =============================================================================
# Serial Manager
# =============================================================================

class SerialManager:
    """
    Manages serial communication with the Teensy console.
    
    Handles connection management, heartbeat monitoring, and position updates
    in background threads.
    """

    def __init__(self, on_position_update: Callable, on_connection_lost: Callable):
        """
        Initialize SerialManager.
        
        Args:
            on_position_update: Callback(motor_id, position) for position updates
            on_connection_lost: Callback() when connection is lost
        """
        self._serial_port: Optional[serial.Serial] = None
        self._port_name: str = ""
        self._is_connected: bool = False

        self._on_position_update = on_position_update
        self._on_connection_lost = on_connection_lost

        self._read_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._running: bool = False

        self._last_heartbeat_received: float = 0.0
        self._lock = threading.Lock()
        self._connection_lost_notified: bool = False
        self._heartbeat_suspended: bool = False

    @property
    def is_connected(self) -> bool:
        """Return current connection status."""
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool):
        """Set connection status."""
        self._is_connected = value

    @property
    def heartbeat_suspended(self) -> bool:
        """Return whether heartbeat checking is suspended."""
        return self._heartbeat_suspended

    @heartbeat_suspended.setter
    def heartbeat_suspended(self, value: bool):
        """Set heartbeat suspension status."""
        self._heartbeat_suspended = value

    @property
    def last_heartbeat_received(self) -> float:
        """Return timestamp of last heartbeat."""
        return self._last_heartbeat_received

    @last_heartbeat_received.setter
    def last_heartbeat_received(self, value: float):
        """Set timestamp of last heartbeat."""
        self._last_heartbeat_received = value

    @staticmethod
    def list_ports() -> list:
        """Return list of available serial port names."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port_name: str) -> bool:
        """
        Connect to the specified serial port.
        
        Args:
            port_name: Name of the serial port to connect to.
            
        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.disconnect()

            self._serial_port = serial.Serial(
                port=port_name,
                baudrate=Config.BAUD_RATE,
                timeout=Config.SERIAL_TIMEOUT
            )

            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()

            self._port_name = port_name
            self._is_connected = True
            self._running = True
            self._last_heartbeat_received = time.time()
            self._connection_lost_notified = False

            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._heartbeat_thread.start()

            return True

        except serial.SerialException as e:
            logger.error("Connection error: %s", e)
            self._is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the serial port and stop background threads."""
        self._running = False
        self._is_connected = False

        if self._serial_port and self._serial_port.is_open:
            try:
                self._serial_port.close()
            except serial.SerialException:
                pass

        self._serial_port = None
        self._port_name = ""

    def send(self, command: str) -> bool:
        """
        Send a command to the console.
        
        Args:
            command: Command string to send.
            
        Returns:
            True if send successful, False otherwise.
        """
        if not self._is_connected or not self._serial_port:
            return False

        try:
            with self._lock:
                self._serial_port.write(command.encode('utf-8'))
                self._serial_port.flush()
            return True
        except serial.SerialException as e:
            if self._running:
                logger.error("Send error: %s", e)
            return False

    def _read_loop(self):
        """Background thread loop for reading serial data."""
        buffer = ""
        while self._running:
            try:
                if (self._serial_port and 
                    self._serial_port.is_open and 
                    self._serial_port.in_waiting > 0):
                    
                    chunk = self._serial_port.read(
                        self._serial_port.in_waiting
                    ).decode('utf-8', errors='ignore')
                    buffer += chunk

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._handle_response(line)

            except (OSError, serial.SerialException) as e:
                if self._running:
                    logger.error("Device disconnected: %s", e)
                    self._running = False
                    self._is_connected = False
                    if not self._connection_lost_notified:
                        self._connection_lost_notified = True
                        self._on_connection_lost()
                break

            except Exception as e:
                if self._running:
                    logger.error("Read error: %s", e)

            time.sleep(Config.SERIAL_READ_INTERVAL)

    def _heartbeat_loop(self):
        """Background thread loop for heartbeat monitoring."""
        while self._running:
            if not self._heartbeat_suspended:
                self.send(SerialProtocol.cmd_heartbeat())

                elapsed = time.time() - self._last_heartbeat_received
                if elapsed > Config.HEARTBEAT_TIMEOUT:
                    if self._is_connected and not self._connection_lost_notified:
                        self._is_connected = False
                        self._connection_lost_notified = True
                        self._on_connection_lost()

            time.sleep(Config.HEARTBEAT_INTERVAL)

    def _handle_response(self, response: str):
        """Process a response from the console."""
        parsed = SerialProtocol.parse_response(response)
        if not parsed:
            return

        cmd, args = parsed

        if cmd == "HEARTBEAT_ACK":
            self._last_heartbeat_received = time.time()

        elif cmd == "POS" and len(args) >= 2:
            try:
                motor_id = int(args[0])
                position = float(args[1])
                self._on_position_update(motor_id, position)
            except (ValueError, IndexError):
                pass


# =============================================================================
# Notes Window
# =============================================================================

class NotesWindow(tk.Toplevel):
    """Popup window for editing motor notes."""

    def __init__(self, parent: tk.Tk, motor_id: int, current_notes: str, 
                 on_save: Callable[[int, str], None]):
        """
        Initialize NotesWindow.
        
        Args:
            parent: Parent Tk window
            motor_id: ID of motor being edited
            current_notes: Current notes content
            on_save: Callback(motor_id, notes) when window closes
        """
        super().__init__(parent)

        self._motor_id = motor_id
        self._on_save = on_save

        self.title(f"Notes for Motor {motor_id}")
        self.geometry("300x200")
        self.resizable(False, False)

        # Position window to the right of parent
        parent_x = parent.winfo_x()
        parent_y = parent.winfo_y()
        parent_width = parent.winfo_width()
        self.geometry(f"+{parent_x + parent_width + 10}+{parent_y}")

        self._text = tk.Text(self, wrap=tk.WORD, font=('TkDefaultFont', 10))
        self._text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self._text.insert('1.0', current_notes)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        """Handle window close event."""
        notes = self._text.get('1.0', tk.END).strip()
        self._on_save(self._motor_id, notes)
        self.destroy()


# =============================================================================
# Main Application
# =============================================================================

class OpenDriveApp:
    """Main OpenDrive GUI application."""

    def __init__(self):
        """Initialize the OpenDrive application."""
        self._root = tk.Tk()
        self._root.title(Config.WINDOW_TITLE)
        self._configure_light_theme()

        # State
        self._motor_states: list = []
        self._motor_widgets: list = []
        self._notes_windows: dict = {}

        # Serial manager
        self._serial_manager = SerialManager(
            on_position_update=self._on_position_update,
            on_connection_lost=self._on_connection_lost
        )

        # GUI variables (initialized in _build_ui)
        self._port_var: Optional[tk.StringVar] = None
        self._port_combo: Optional[ttk.Combobox] = None
        self._connect_btn: Optional[tk.Button] = None
        self._motors_container: Optional[tk.Frame] = None

        # Load saved state
        self._load_state()

        # Build UI
        self._build_ui()

        # Ensure at least one motor
        if len(self._motor_states) == 0:
            self._add_motor()
        else:
            self._rebuild_motor_rows()

        self._style_widget_tree()

        # Disable resizing after UI is built
        self._root.update_idletasks()
        self._root.resizable(False, False)

        # Start auto-save timer
        self._schedule_auto_save()

        # Handle window close
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    @property
    def root(self) -> tk.Tk:
        """Return the root Tk window."""
        return self._root

    @property
    def serial_manager(self) -> SerialManager:
        """Return the serial manager instance."""
        return self._serial_manager

    @property
    def motor_states(self) -> list:
        """Return list of motor states."""
        return self._motor_states

    def run(self):
        """Start the main event loop."""
        self._root.mainloop()

    def _configure_light_theme(self):
        """Force readable colors on older macOS Tk builds."""
        for option, value in (
            ("*Background", Config.COLOR_BG),
            ("*Foreground", Config.COLOR_FG),
            ("*Entry.Background", Config.COLOR_ENTRY_BG),
            ("*Entry.Foreground", Config.COLOR_FG),
            ("*Text.Background", Config.COLOR_ENTRY_BG),
            ("*Text.Foreground", Config.COLOR_FG),
            ("*Button.Background", Config.COLOR_ENTRY_BG),
            ("*Button.Foreground", Config.COLOR_FG),
        ):
            self._root.option_add(option, value)

        self._root.tk_setPalette(
            background=Config.COLOR_BG,
            foreground=Config.COLOR_FG,
            activeBackground=Config.COLOR_ACTIVE_BG,
            activeForeground=Config.COLOR_FG,
            highlightColor=Config.COLOR_SELECT_BG,
            selectBackground=Config.COLOR_SELECT_BG,
            selectForeground=Config.COLOR_ENTRY_BG,
        )
        self._root.configure(background=Config.COLOR_BG)

        style = ttk.Style(self._root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(
            "TCombobox",
            fieldbackground=Config.COLOR_ENTRY_BG,
            background=Config.COLOR_ENTRY_BG,
            foreground=Config.COLOR_FG,
            selectbackground=Config.COLOR_SELECT_BG,
            selectforeground=Config.COLOR_ENTRY_BG,
        )

    def _style_widget_tree(self, widget=None):
        """Apply explicit colors to Tk widgets that may ignore the app palette."""
        widget = widget or self._root

        try:
            if isinstance(widget, (tk.Tk, tk.Toplevel, tk.Frame)):
                widget.configure(background=Config.COLOR_BG)
            elif isinstance(widget, tk.Label):
                widget.configure(background=Config.COLOR_BG, foreground=Config.COLOR_FG)
            elif isinstance(widget, tk.Checkbutton):
                widget.configure(
                    background=Config.COLOR_BG,
                    foreground=Config.COLOR_FG,
                    activebackground=Config.COLOR_ACTIVE_BG,
                    activeforeground=Config.COLOR_FG,
                    selectcolor=Config.COLOR_ENTRY_BG,
                )
            elif isinstance(widget, tk.Button):
                widget.configure(
                    background=Config.COLOR_ENTRY_BG,
                    foreground=Config.COLOR_FG,
                    activebackground=Config.COLOR_ACTIVE_BG,
                    activeforeground=Config.COLOR_FG,
                    highlightbackground=Config.COLOR_BG,
                )
            elif isinstance(widget, tk.Entry):
                readonly_bg = widget.cget("readonlybackground")
                widget.configure(
                    background=Config.COLOR_ENTRY_BG,
                    foreground=Config.COLOR_FG,
                    insertbackground=Config.COLOR_FG,
                    readonlybackground=readonly_bg or Config.COLOR_READONLY_BG,
                    disabledforeground=Config.COLOR_FG,
                )
            elif isinstance(widget, tk.Text):
                widget.configure(
                    background=Config.COLOR_ENTRY_BG,
                    foreground=Config.COLOR_FG,
                    insertbackground=Config.COLOR_FG,
                )
        except tk.TclError:
            pass

        for child in widget.winfo_children():
            self._style_widget_tree(child)

    # -------------------------------------------------------------------------
    # UI Construction
    # -------------------------------------------------------------------------

    def _build_ui(self):
        """Build the main UI."""
        # Main container
        main_frame = tk.Frame(self._root, padx=15, pady=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Row 0: Port selection
        self._build_port_selection(main_frame)

        # Row 1: Main buttons (centered)
        self._build_control_buttons(main_frame)

        # Container for Headers and Motor Rows (Grid Layout)
        self._motors_container = tk.Frame(main_frame)
        self._motors_container.pack(fill=tk.BOTH, expand=True)

        # Define Grid Columns
        for i, min_w in enumerate(Config.COL_MIN_WIDTHS):
            self._motors_container.grid_columnconfigure(i, weight=1, minsize=min_w)

        # Header Row
        self._build_header_row()

    def _build_port_selection(self, parent: tk.Frame):
        """Build the port selection UI section."""
        port_frame = tk.Frame(parent)
        port_frame.pack(fill=tk.X, pady=(0, 10))

        tk.Label(port_frame, text="Port:").pack(side=tk.LEFT)

        self._port_var = tk.StringVar()
        self._port_combo = ttk.Combobox(
            port_frame, textvariable=self._port_var,
            width=40, state='readonly'
        )
        self._port_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self._port_combo.bind('<Button-1>', self._refresh_ports)

        self._connect_btn = tk.Button(
            port_frame, text="Connect", command=self._on_connect
        )
        self._connect_btn.pack(side=tk.RIGHT)

    def _build_control_buttons(self, parent: tk.Frame):
        """Build the main control buttons."""
        button_frame = tk.Frame(parent)
        button_frame.pack(pady=(0, 15))

        buttons = [
            ("Run", self._on_run),
            ("Pause", self._on_pause),
            ("Zero", self._on_zero),
            ("Export", self._on_export),
            ("Clear", self._on_clear),
        ]

        for text, command in buttons:
            tk.Button(
                button_frame, text=text, command=command, width=8
            ).pack(side=tk.LEFT, padx=2)

    def _build_header_row(self):
        """Build the header row with column labels."""
        # Col 0: Buttons (+/-)
        adjust_frame = tk.Frame(self._motors_container)
        adjust_frame.grid(row=0, column=0, sticky='w', padx=(0, 5), pady=Config.HEADER_PAD_Y)
        
        tk.Button(
            adjust_frame, text="+", command=self._add_motor,
            font=('TkDefaultFont', 9), width=2
        ).pack(side=tk.LEFT, padx=1)
        
        tk.Button(
            adjust_frame, text="-", command=self._remove_motor,
            font=('TkDefaultFont', 9), width=2
        ).pack(side=tk.LEFT, padx=1)

        # Column labels
        labels = [
            (1, "Speed (mm/s):"),
            (2, "Target (mm):"),
            (3, "Absolute (mm):"),
            (4, "Relative (mm):"),
            (5, "Notes:"),
        ]

        for col, text in labels:
            tk.Label(
                self._motors_container, text=text, anchor='w'
            ).grid(row=0, column=col, sticky='w', padx=5, pady=Config.HEADER_PAD_Y)

    def _create_motor_row(self, state: MotorState, row_index: int) -> dict:
        """
        Create widgets for a motor row.
        
        Args:
            state: MotorState for this row
            row_index: Grid row index (1-based, 0 is header)
            
        Returns:
            Dictionary of widget references.
        """
        widgets = {}

        # Col 0: Motor ID Label + Checkbox
        id_frame = tk.Frame(self._motors_container)
        id_frame.grid(row=row_index, column=0, sticky='w', padx=(0, 5), pady=Config.ROW_PAD_Y)

        tk.Label(id_frame, text=f"Motor {state.motor_id}:", width=8, anchor='w').pack(side=tk.LEFT)

        widgets['selected_var'] = tk.BooleanVar(value=state.is_selected)
        widgets['checkbox'] = tk.Checkbutton(
            id_frame,
            variable=widgets['selected_var'],
            command=lambda mid=state.motor_id: self._on_selection_change(mid)
        )
        widgets['checkbox'].pack(side=tk.LEFT)

        # Col 1: Speed entry
        widgets['speed_var'] = tk.StringVar(value=f"{state.speed}" if state.speed else "")
        widgets['speed_entry'] = tk.Entry(
            self._motors_container, textvariable=widgets['speed_var'],
            width=Config.ENTRY_WIDTH
        )
        widgets['speed_entry'].grid(row=row_index, column=1, sticky='ew', padx=5, pady=Config.ROW_PAD_Y)
        widgets['speed_entry'].bind('<KeyRelease>', lambda e, mid=state.motor_id: self._on_speed_change(mid))
        widgets['speed_entry'].bind('<FocusOut>', lambda e, mid=state.motor_id: self._on_speed_change(mid))

        # Col 2: Target entry
        widgets['target_var'] = tk.StringVar(value=f"{state.target}" if state.target else "")
        widgets['target_entry'] = tk.Entry(
            self._motors_container, textvariable=widgets['target_var'],
            width=Config.ENTRY_WIDTH
        )
        widgets['target_entry'].grid(row=row_index, column=2, sticky='ew', padx=5, pady=Config.ROW_PAD_Y)
        widgets['target_entry'].bind('<KeyRelease>', lambda e, mid=state.motor_id: self._on_target_change(mid))
        widgets['target_entry'].bind('<FocusOut>', lambda e, mid=state.motor_id: self._on_target_change(mid))

        # Col 3: Absolute position
        widgets['absolute_var'] = tk.StringVar(value=f"{state.absolute:.3f}")
        widgets['absolute_entry'] = tk.Entry(
            self._motors_container, textvariable=widgets['absolute_var'],
            width=Config.ENTRY_WIDTH, state='readonly',
            readonlybackground='lightgray'
        )
        widgets['absolute_entry'].grid(row=row_index, column=3, sticky='ew', padx=5, pady=Config.ROW_PAD_Y)
        widgets['absolute_entry'].bind('<Double-Button-1>', lambda e, mid=state.motor_id: self._enable_absolute_edit(mid))

        # Col 4: Relative position
        widgets['relative_var'] = tk.StringVar(value=f"{state.relative:.3f}")
        widgets['relative_entry'] = tk.Entry(
            self._motors_container, textvariable=widgets['relative_var'],
            width=Config.ENTRY_WIDTH, state='readonly',
            readonlybackground='lightgray'
        )
        widgets['relative_entry'].grid(row=row_index, column=4, sticky='ew', padx=5, pady=Config.ROW_PAD_Y)
        widgets['relative_entry'].bind('<Double-Button-1>', lambda e, mid=state.motor_id: self._enable_relative_edit(mid))

        # Col 5: Notes
        notes_preview = state.notes[:15] + "..." if len(state.notes) > 15 else state.notes
        widgets['notes_var'] = tk.StringVar(value=notes_preview)
        widgets['notes_entry'] = tk.Entry(
            self._motors_container, textvariable=widgets['notes_var'],
            width=Config.ENTRY_WIDTH, state='readonly',
            readonlybackground='white', cursor='hand2'
        )
        widgets['notes_entry'].grid(row=row_index, column=5, sticky='ew', padx=5, pady=Config.ROW_PAD_Y)
        widgets['notes_entry'].bind('<Double-Button-1>', lambda e, mid=state.motor_id: self._on_notes_click(mid))

        # Store references to widgets for cleanup
        widgets['all_widgets'] = [
            id_frame, widgets['speed_entry'], widgets['target_entry'],
            widgets['absolute_entry'], widgets['relative_entry'], widgets['notes_entry']
        ]

        return widgets

    def _rebuild_motor_rows(self):
        """Rebuild all motor row widgets."""
        # Clear existing motor widgets from grid
        for w_dict in self._motor_widgets:
            for widget in w_dict.get('all_widgets', []):
                widget.destroy()
        self._motor_widgets.clear()

        # Recreate rows starting from index 1 (0 is header)
        for i, state in enumerate(self._motor_states):
            widgets = self._create_motor_row(state, row_index=i + 1)
            self._motor_widgets.append(widgets)

        self._style_widget_tree(self._motors_container)
        self._root.update_idletasks()

    # -------------------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------------------

    def _get_widgets(self, motor_id: int) -> Optional[dict]:
        """Get widget dictionary for motor by ID."""
        idx = motor_id - 1
        if 0 <= idx < len(self._motor_widgets):
            return self._motor_widgets[idx]
        return None

    def _get_state(self, motor_id: int) -> Optional[MotorState]:
        """Get MotorState for motor by ID."""
        idx = motor_id - 1
        if 0 <= idx < len(self._motor_states):
            return self._motor_states[idx]
        return None

    def _safe_popup(self, popup_func: Callable, title: str, message: str = None, **kwargs):
        """
        Show a popup dialog with automatic motor pause and heartbeat suspension.
        
        Suspends heartbeat checking and pauses all running motors before showing 
        the dialog to prevent connection timeout during blocking dialogs.
        """
        paused_any = False

        # Suspend heartbeat checking during the blocking dialog
        self._serial_manager.heartbeat_suspended = True

        try:
            # Pause all running motors
            if self._serial_manager.is_connected:
                for state in self._motor_states:
                    if state.is_running:
                        self._serial_manager.send(SerialProtocol.cmd_pause(state.motor_id))
                        state.is_running = False
                        paused_any = True

            # Add safety note to message if motors were paused
            if paused_any and message:
                message += "\n\n(Safety: Running motors have been automatically paused)"

            # Show the popup
            if message:
                return popup_func(title, message, **kwargs)
            else:
                return popup_func(title=title, **kwargs)
        
        finally:
            # Resume heartbeat checking and reset the timer
            self._serial_manager.heartbeat_suspended = False
            if self._serial_manager.is_connected:
                self._serial_manager.last_heartbeat_received = time.time()

    # -------------------------------------------------------------------------
    # Event Handlers - Selection and Input
    # -------------------------------------------------------------------------

    def _on_selection_change(self, motor_id: int):
        """Handle motor selection checkbox change."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if state and widgets:
            state.is_selected = widgets['selected_var'].get()

    def _on_speed_change(self, motor_id: int):
        """Handle speed input change."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if state and widgets:
            try:
                state.speed = float(widgets['speed_var'].get())
            except ValueError:
                pass

    def _on_target_change(self, motor_id: int):
        """Handle target input change."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if state and widgets:
            try:
                new_target = float(widgets['target_var'].get())
                state.target = new_target
            except ValueError:
                pass

    def _enable_absolute_edit(self, motor_id: int):
        """Enable editing of absolute position field."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if widgets and state:
            # If motor is running, pause it before editing
            if state.is_running:
                if self._serial_manager.is_connected:
                    self._serial_manager.send(SerialProtocol.cmd_pause(motor_id))
                state.is_running = False

            entry = widgets['absolute_entry']
            entry.config(state='normal', background='white')
            entry.focus_set()
            entry.select_range(0, tk.END)
            entry.bind('<Return>', lambda e, mid=motor_id: self._on_absolute_confirm(mid))
            entry.bind('<FocusOut>', lambda e, mid=motor_id: self._on_absolute_confirm(mid))

    def _on_absolute_confirm(self, motor_id: int):
        """Handle confirmation of absolute position edit."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if not state or not widgets:
            return

        entry = widgets['absolute_entry']
        if entry['state'] == 'readonly':
            return

        try:
            value = float(widgets['absolute_var'].get())
            if Config.MIN_ABSOLUTE_POS <= value <= Config.MAX_ABSOLUTE_POS:
                state.absolute = value
                # Reset relative position to 0 and set new zero point
                state.relative = 0.0
                state.relative_zero_point = value
                widgets['relative_var'].set("0.000")
            else:
                raise ValueError("Position out of range")
        except ValueError:
            self._safe_popup(
                messagebox.showwarning, "Invalid Input",
                f"Absolute position must be between {Config.MIN_ABSOLUTE_POS}mm and "
                f"{Config.MAX_ABSOLUTE_POS}mm. See Motor {motor_id}."
            )
            widgets['absolute_var'].set(f"{state.absolute:.3f}")

        entry.config(state='readonly', readonlybackground='lightgray')

    def _enable_relative_edit(self, motor_id: int):
        """Enable editing of relative position field."""
        widgets = self._get_widgets(motor_id)
        if widgets:
            entry = widgets['relative_entry']
            entry.config(state='normal', background='white')
            entry.focus_set()
            entry.select_range(0, tk.END)
            entry.bind('<Return>', lambda e, mid=motor_id: self._on_relative_confirm(mid))
            entry.bind('<FocusOut>', lambda e, mid=motor_id: self._on_relative_confirm(mid))

    def _on_relative_confirm(self, motor_id: int):
        """Handle confirmation of relative position edit."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if not state or not widgets:
            return

        entry = widgets['relative_entry']
        if entry['state'] == 'readonly':
            return

        try:
            value = float(widgets['relative_var'].get())
            state.relative = value
            state.relative_zero_point = state.absolute - value
        except ValueError:
            widgets['relative_var'].set(f"{state.relative:.3f}")

        entry.config(state='readonly', readonlybackground='lightgray')

    def _on_notes_click(self, motor_id: int):
        """Handle notes field click to open notes window."""
        if motor_id in self._notes_windows:
            try:
                self._notes_windows[motor_id].destroy()
            except tk.TclError:
                pass

        state = self._get_state(motor_id)
        if state:
            window = NotesWindow(self._root, motor_id, state.notes, self._on_notes_save)
            self._notes_windows[motor_id] = window

    def _on_notes_save(self, motor_id: int, notes: str):
        """Handle notes save from notes window."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if state and widgets:
            state.notes = notes
            preview = notes[:15] + "..." if len(notes) > 15 else notes
            widgets['notes_var'].set(preview)

        if motor_id in self._notes_windows:
            del self._notes_windows[motor_id]

    # -------------------------------------------------------------------------
    # Port Management
    # -------------------------------------------------------------------------

    def _refresh_ports(self, event=None):
        """Refresh the list of available serial ports."""
        ports = SerialManager.list_ports()
        self._port_combo['values'] = ports

    def _on_connect(self):
        """Handle connect/disconnect button click."""
        port = self._port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Please select a port.")
            return

        if self._serial_manager.is_connected:
            # Pause all running motors before disconnecting
            for state in self._motor_states:
                if state.is_running:
                    self._serial_manager.send(SerialProtocol.cmd_pause(state.motor_id))
                    state.is_running = False
            self._serial_manager.disconnect()
            self._connect_btn.config(text="Connect")
        else:
            if self._serial_manager.connect(port):
                self._connect_btn.config(text="Disconnect")
                messagebox.showinfo("Connected", f"Connected to {port}")
            else:
                messagebox.showerror("Connection Failed", f"Could not connect to {port}")

    def _on_connection_lost(self):
        """Handle connection lost callback from serial manager."""
        for state in self._motor_states:
            state.is_running = False
        self._root.after(0, self._show_connection_lost_dialog)

    def _show_connection_lost_dialog(self):
        """Show connection lost dialog on main thread."""
        self._connect_btn.config(text="Connect")
        messagebox.showwarning(
            "Connection Lost",
            "Connection to console was lost. Any running motors have been paused. "
            "Check physical connection and attempt to reconnect."
        )

    # -------------------------------------------------------------------------
    # Motor Management
    # -------------------------------------------------------------------------

    def _add_motor(self):
        """Add a new motor row."""
        if len(self._motor_states) >= Config.MAX_MOTORS:
            self._safe_popup(
                messagebox.showinfo, "Maximum Motors",
                f"Cannot add more than {Config.MAX_MOTORS} motor(s)."
            )
            return

        motor_id = len(self._motor_states) + 1
        state = MotorState(motor_id=motor_id)
        self._motor_states.append(state)

        self._rebuild_motor_rows()

    def _remove_motor(self):
        """Remove the last motor row."""
        if len(self._motor_states) <= 1:
            return

        last_state = self._motor_states[-1]
        if last_state.is_running:
            self._serial_manager.send(SerialProtocol.cmd_pause(last_state.motor_id))

        self._motor_states.pop()
        self._rebuild_motor_rows()

    # -------------------------------------------------------------------------
    # Control Actions
    # -------------------------------------------------------------------------

    def _on_run(self):
        """Handle Run button click."""
        if not self._serial_manager.is_connected:
            self._safe_popup(messagebox.showwarning, "Not Connected", "Console not connected.")
            return

        selected = [s for s in self._motor_states if s.is_selected]
        if not selected:
            self._safe_popup(messagebox.showinfo, "No Selection", "Select at least one motor.")
            return

        # Check if any selected motor's absolute position is being edited
        for state in selected:
            widgets = self._get_widgets(state.motor_id)
            if widgets and widgets['absolute_entry']['state'] != 'readonly':
                self._safe_popup(
                    messagebox.showwarning, "Edit In Progress",
                    f"Motor {state.motor_id} absolute position is being edited. "
                    f"Press enter to confirm edit before running."
                )
                return

        # Validation pass
        for state in selected:
            widgets = self._get_widgets(state.motor_id)
            if not widgets:
                continue

            # Validate Speed Input
            try:
                s_val = float(widgets['speed_var'].get())
                state.speed = s_val
            except ValueError:
                self._safe_popup(
                    messagebox.showwarning, "Invalid Input",
                    f"Invalid speed input for Motor {state.motor_id}."
                )
                return

            # Validate Target Input
            try:
                t_val = float(widgets['target_var'].get())
                state.target = t_val
            except ValueError:
                self._safe_popup(
                    messagebox.showwarning, "Invalid Input",
                    f"Invalid target input for Motor {state.motor_id}."
                )
                return

            # Range checks
            if not (Config.MIN_SPEED <= state.speed <= Config.MAX_SPEED):
                self._safe_popup(
                    messagebox.showwarning, "Invalid Input",
                    f"Speed must be between {Config.MIN_SPEED}mm/s and "
                    f"{Config.MAX_SPEED}mm/s. See Motor {state.motor_id}."
                )
                return

            if not (Config.MIN_TARGET_POS <= state.target <= Config.MAX_TARGET_POS):
                self._safe_popup(
                    messagebox.showwarning, "Invalid Input",
                    f"Target must be between {Config.MIN_TARGET_POS}mm and "
                    f"{Config.MAX_TARGET_POS}mm. See Motor {state.motor_id}."
                )
                return

            if not (Config.MIN_ABSOLUTE_POS <= state.absolute <= Config.MAX_ABSOLUTE_POS):
                self._safe_popup(
                    messagebox.showwarning, "Invalid Input",
                    f"Absolute position must be between {Config.MIN_ABSOLUTE_POS}mm and "
                    f"{Config.MAX_ABSOLUTE_POS}mm. See Motor {state.motor_id}."
                )
                return

        # Execution pass
        for state in selected:
            cmd = SerialProtocol.cmd_run(
                state.motor_id, state.speed, state.target, state.absolute
            )
            self._serial_manager.send(cmd)
            state.is_running = True

    def _on_pause(self):
        """Handle Pause button click."""
        selected = [s for s in self._motor_states if s.is_selected and s.is_running]
        for state in selected:
            self._serial_manager.send(SerialProtocol.cmd_pause(state.motor_id))
            state.is_running = False

    def _on_zero(self):
        """Zero relative position for selected motors."""
        selected = [s for s in self._motor_states if s.is_selected]

        for state in selected:
            state.relative_zero_point = state.absolute
            state.relative = 0.0

            widgets = self._get_widgets(state.motor_id)
            if widgets:
                widgets['relative_var'].set("0.000")

    def _on_export(self):
        """Export data to CSV file."""
        # Pause any running motors before opening file dialog
        if self._serial_manager.is_connected:
            for state in self._motor_states:
                if state.is_running:
                    self._serial_manager.send(SerialProtocol.cmd_pause(state.motor_id))
                    state.is_running = False

        # Suspend heartbeat checking during the blocking file dialog
        self._serial_manager.heartbeat_suspended = True

        try:
            filepath = filedialog.asksaveasfilename(
                parent=self._root,
                title="Export Data",
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                initialfile=f"opendrive_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )

            if not filepath:
                return

            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Date and Time", "Motor", "Speed (mm/s)", "Target (mm)",
                    "Absolute (mm)", "Relative (mm)", "Notes"
                ])

                timestamp = datetime.now().strftime("%m/%d/%y %H:%M")
                for state in self._motor_states:
                    writer.writerow([
                        timestamp,
                        f"Motor {state.motor_id}",
                        f"{state.speed:.3f}",
                        f"{state.target:.3f}",
                        f"{state.absolute:.3f}",
                        f"{state.relative:.3f}",
                        state.notes
                    ])

            messagebox.showinfo("Export Complete", f"Data exported to:\n{filepath}")

        except IOError as e:
            messagebox.showerror("Export Error", f"Failed to export: {e}")

        finally:
            # Resume heartbeat checking and reset the timer
            self._serial_manager.heartbeat_suspended = False
            if self._serial_manager.is_connected:
                self._serial_manager.last_heartbeat_received = time.time()

    def _on_clear(self):
        """Clear selected motors - pause if running, then wipe values."""
        selected = [s for s in self._motor_states if s.is_selected]

        if not selected:
            return

        # Pause selected motors that are running
        for state in selected:
            if state.is_running:
                self._serial_manager.send(SerialProtocol.cmd_pause(state.motor_id))

        # Clear values for selected motors
        for state in selected:
            state.speed = 0.0
            state.target = 0.0
            state.absolute = 0.0
            state.relative = 0.0
            state.relative_zero_point = 0.0
            state.notes = ""
            state.is_running = False

            widgets = self._get_widgets(state.motor_id)
            if widgets:
                widgets['speed_var'].set("")
                widgets['target_var'].set("")
                widgets['absolute_var'].set("0.000")
                widgets['relative_var'].set("0.000")
                widgets['notes_var'].set("")

        self._root.update_idletasks()

    # -------------------------------------------------------------------------
    # Position Updates
    # -------------------------------------------------------------------------

    def _on_position_update(self, motor_id: int, absolute: float):
        """Handle position update from serial manager."""
        state = self._get_state(motor_id)
        if state:
            state.absolute = absolute
            state.relative = absolute - state.relative_zero_point

        self._root.after(0, lambda: self._update_position_display(motor_id, absolute))

    def _update_position_display(self, motor_id: int, absolute: float):
        """Update position display widgets on main thread."""
        state = self._get_state(motor_id)
        widgets = self._get_widgets(motor_id)
        if state and widgets:
            if widgets['absolute_entry']['state'] == 'readonly':
                widgets['absolute_var'].set(f"{state.absolute:.3f}")
            if widgets['relative_entry']['state'] == 'readonly':
                widgets['relative_var'].set(f"{state.relative:.3f}")

    # -------------------------------------------------------------------------
    # State Persistence
    # -------------------------------------------------------------------------

    def _get_data_path(self) -> Path:
        """Get path to state data file."""
        return Path(__file__).parent / Config.DATA_FILE

    def _save_state(self):
        """Save current state to JSON file."""
        try:
            data = {
                'motor_states': [s.to_dict() for s in self._motor_states],
                'port': self._port_var.get() if self._port_var else ""
            }
            with open(self._get_data_path(), 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except IOError as e:
            logger.error("Failed to save state: %s", e)

    def _load_state(self):
        """Load state from JSON file."""
        try:
            path = self._get_data_path()
            if path.exists():
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                self._motor_states = [
                    MotorState.from_dict(s) for s in data.get('motor_states', [])
                ]

                for state in self._motor_states:
                    state.is_running = False
        except (IOError, json.JSONDecodeError) as e:
            logger.error("Failed to load state: %s", e)
            self._motor_states = []

    def _schedule_auto_save(self):
        """Schedule periodic auto-save of state."""
        self._save_state()
        self._root.after(int(Config.AUTO_SAVE_INTERVAL * 1000), self._schedule_auto_save)

    # -------------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------------

    def _on_close(self):
        """Handle window close event."""
        if self._serial_manager.is_connected:
            self._serial_manager.send(SerialProtocol.cmd_pause_all())

        self._serial_manager.disconnect()
        self._save_state()

        for window in self._notes_windows.values():
            try:
                window.destroy()
            except tk.TclError:
                pass

        self._root.destroy()


# =============================================================================
# Entry Point
# =============================================================================

def main():
    """Main entry point for OpenDrive GUI."""
    app = OpenDriveApp()
    app.run()


if __name__ == "__main__":
    main()
