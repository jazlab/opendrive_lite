"""Single-owner serial I/O; callbacks are dispatched only by the GUI poller."""

from collections import deque
from dataclasses import dataclass, field
import logging
import math
import threading
import time
from typing import Callable, Optional

import serial
import serial.tools.list_ports

from control_protocol import Config, SerialProtocol

logger = logging.getLogger(__name__)


@dataclass
class _Session:
    port_name: str
    lock: object = field(default_factory=threading.Lock)
    stop: threading.Event = field(default_factory=threading.Event)
    wake: threading.Event = field(default_factory=threading.Event)
    commands: deque = field(default_factory=deque)
    pauses: set = field(default_factory=set)
    positions: dict = field(default_factory=dict)
    events: deque = field(default_factory=deque)
    last_gui_tick: float = field(default_factory=time.monotonic)
    connected: bool = False
    failure: str = ""
    failure_kind: str = "connection"
    thread: Optional[threading.Thread] = None


class SerialManager:
    """Queue commands without device I/O on the caller's thread.

    connect() means an attempt was accepted; send() means queued, not acknowledged.
    Call poll_events() from Tk. Each worker owns one port, including open/close.
    """

    def __init__(self, on_position_update: Callable, on_connection_lost: Callable,
                 on_connected: Optional[Callable] = None):
        self._on_position_update = on_position_update
        self._on_connection_lost = on_connection_lost
        self._on_connected = on_connected
        self._session: Optional[_Session] = None
        self._owner_thread = threading.get_ident()

    @property
    def is_connected(self) -> bool:
        session = self._session
        if session is None:
            return False
        with session.lock:
            return session.connected and not session.stop.is_set()

    @property
    def is_busy(self) -> bool:
        """True until the previous worker has finished closing its port."""
        session = self._session
        return bool(session and session.thread and session.thread.is_alive())

    @property
    def failure_reason(self) -> str:
        session = self._session
        if session is None:
            return ""
        with session.lock:
            return session.failure

    @property
    def failure_kind(self) -> str:
        session = self._session
        if session is None:
            return "connection"
        with session.lock:
            return session.failure_kind

    @staticmethod
    def _firmware_warning(detected: str) -> str:
        return (
            f"Expected {Config.FIRMWARE_PRODUCT} {Config.FIRMWARE_VARIANT} firmware "
            f"{Config.FIRMWARE_VERSION}.\n{detected}\n\n"
            f"Upload {Config.FIRMWARE_SKETCH} from this version of the repository, "
            "then reconnect. Movement is disabled until compatible firmware is verified."
        )

    @staticmethod
    def list_ports() -> list:
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port_name: str) -> bool:
        """Open asynchronously; never overlap connection workers."""
        self._assert_owner()
        if self.is_busy:
            return False
        session = _Session(port_name)
        self._session = session
        session.thread = threading.Thread(target=self._io_loop, args=(session,),
                                          daemon=True, name="OpenDrive serial")
        session.thread.start()
        return True

    def disconnect(self):
        """Request stop and close without waiting on a device or joining in Tk."""
        session = self._session
        if session:
            with session.lock:
                session.connected = False
                session.commands.clear()
                session.positions.clear()
                session.events.clear()
                session.stop.set()
            session.wake.set()

    def send(self, command: str) -> bool:
        """Queue a command; pauses supersede queued movement for their motors."""
        parsed = SerialProtocol.parse_response(command)
        if not parsed:
            return False
        cmd, args = parsed
        if cmd not in {"RUN", "TARGET", "PAUSE", "PAUSE_ALL"}:
            return False
        motor_id = None
        if cmd in {"RUN", "TARGET", "PAUSE"}:
            try:
                motor_id = int(args[0])
            except (ValueError, IndexError):
                return False
            if not 1 <= motor_id <= Config.MAX_MOTORS:
                return False
        session = self._session
        if session is None:
            return False
        with session.lock:
            if not session.connected or session.stop.is_set():
                return False
            if time.monotonic() - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT:
                self._fail_locked(session, "Control window stopped responding")
                return False
            if cmd in {"PAUSE", "PAUSE_ALL"}:
                session.commands = deque(
                    item for item in session.commands
                    if item[0] not in {"RUN", "TARGET"}
                    or (cmd == "PAUSE" and item[1] != motor_id)
                )
                if cmd == "PAUSE_ALL":
                    session.pauses.clear()
                    session.pauses.add(None)
                elif None not in session.pauses:
                    session.pauses.add(motor_id)
            elif len(session.commands) >= Config.SERIAL_COMMAND_QUEUE_LIMIT:
                self._fail_locked(session, "Serial command queue overflow")
                return False
            else:
                session.commands.append((cmd, motor_id, command))
        session.wake.set()
        return True

    def poll_events(self):
        """Renew GUI liveness and deliver a bounded snapshot on the Tk thread."""
        self._assert_owner()
        session = self._session
        if session is None:
            return
        with session.lock:
            # Never renew an expired lease, even if the worker was delayed too.
            if (session.connected and not session.stop.is_set()
                    and time.monotonic() - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT):
                self._fail_locked(session, "Control window stopped responding")
            session.last_gui_tick = time.monotonic()
            events = list(session.events)
            session.events.clear()
            positions = list(session.positions.items())
            session.positions.clear()
        for event in events:
            if self._session is not session:
                return
            if event == "connected" and self.is_connected and self._on_connected:
                self._on_connected()
            elif event == "lost":
                self._on_connection_lost()
        for motor_id, position in positions:
            if self._session is not session:
                return
            self._on_position_update(motor_id, position)

    def _assert_owner(self):
        if threading.get_ident() != self._owner_thread:
            raise RuntimeError("Serial lifecycle and event polling belong to the GUI thread")

    @staticmethod
    def _fail_locked(session: _Session, reason: str, kind: str = "connection"):
        if session.stop.is_set():
            return
        session.failure = reason
        session.failure_kind = kind
        session.connected = False
        session.commands.clear()
        session.positions.clear()
        session.events.clear()
        session.events.append("lost")
        session.stop.set()
        session.wake.set()
        # No logging or device I/O while holding this lock: GUI calls acquire it.

    @staticmethod
    def _write(port, command: str):
        payload = command.encode("utf-8")
        if port.write(payload) != len(payload):
            raise serial.SerialTimeoutException("Incomplete serial write")
        # flush() can wait indefinitely for the driver. Do not use it.

    def _io_loop(self, session: _Session):
        port = None
        try:
            port = serial.Serial(port=session.port_name, baudrate=Config.BAUD_RATE,
                                 timeout=0, write_timeout=Config.SERIAL_WRITE_TIMEOUT)
            port.reset_input_buffer()
            port.reset_output_buffer()
            self._write(port, SerialProtocol.cmd_pause_all())
            now = time.monotonic()
            with session.lock:
                if session.stop.is_set():
                    return
                if now - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT:
                    self._fail_locked(session, "Control window stopped responding")
                    return
            # Opening a port does not grant movement permission. Verify the
            # firmware and receive a heartbeat ACK before notifying the GUI.
            handshake_started = now
            firmware_verified = False
            last_ack = None
            next_firmware_query = now
            next_heartbeat = now
            buffer = b""
            pending_positions = {}
            while not session.stop.is_set():
                session.wake.clear()
                with session.lock:
                    if time.monotonic() - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT:
                        self._fail_locked(session, "Control window stopped responding")
                if session.stop.is_set():
                    break
                chunk = port.read(min(port.in_waiting, Config.SERIAL_RECEIVE_LIMIT))
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    parsed = SerialProtocol.parse_response(line.decode("ascii", errors="replace"))
                    if not parsed:
                        continue
                    cmd, args = parsed
                    if cmd == "FIRMWARE":
                        expected = [Config.FIRMWARE_PRODUCT, Config.FIRMWARE_VARIANT,
                                    Config.FIRMWARE_VERSION]
                        if args != expected:
                            with session.lock:
                                self._fail_locked(session, self._firmware_warning(
                                    "Detected: " + ", ".join(args)[:160]), "firmware")
                            break
                        firmware_verified = True
                    elif cmd == "HEARTBEAT_ACK":
                        last_ack = time.monotonic()
                    elif cmd == "POS" and len(args) >= 2:
                        try:
                            motor_id, position = int(args[0]), float(args[1])
                        except ValueError:
                            continue
                        if (1 <= motor_id <= Config.MAX_MOTORS and math.isfinite(position)
                                and Config.MIN_ABSOLUTE_POS <= position <= Config.MAX_ABSOLUTE_POS):
                            pending_positions[motor_id] = position
                if len(buffer) > Config.SERIAL_RECEIVE_LIMIT:
                    raise serial.SerialException("Oversized serial response")
                now = time.monotonic()
                with session.lock:
                    if (not firmware_verified
                            and now - handshake_started > Config.FIRMWARE_CHECK_TIMEOUT):
                        self._fail_locked(session, self._firmware_warning(
                            "The controller did not report a firmware version. It may have "
                            "older firmware installed, or the selected port may be incorrect."),
                            "firmware")
                    elif ((last_ack is not None and now - last_ack > Config.HEARTBEAT_TIMEOUT)
                          or (firmware_verified and last_ack is None
                              and now - handshake_started > Config.HEARTBEAT_TIMEOUT)):
                        self._fail_locked(session, "Controller heartbeat timed out")
                    if now - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT:
                        self._fail_locked(session, "Control window stopped responding")
                    if session.stop.is_set():
                        break
                    if firmware_verified and last_ack is not None and not session.connected:
                        session.connected = True
                        session.events.append("connected")
                    if session.connected:
                        session.positions.update(pending_positions)
                        pending_positions.clear()
                    command = None
                    if session.pauses:
                        motor_id = session.pauses.pop()
                        command = (SerialProtocol.cmd_pause_all() if motor_id is None
                                   else SerialProtocol.cmd_pause(motor_id))
                    elif session.commands:
                        command = session.commands.popleft()[2]
                if command:
                    self._write(port, command)
                # Recheck after a potentially slow write before renewing motion permission.
                now = time.monotonic()
                with session.lock:
                    if now - session.last_gui_tick > Config.GUI_WATCHDOG_TIMEOUT:
                        self._fail_locked(session, "Control window stopped responding")
                    alive = not session.stop.is_set()
                if alive and not firmware_verified and now >= next_firmware_query:
                    self._write(port, SerialProtocol.cmd_firmware_info())
                    next_firmware_query = time.monotonic() + Config.FIRMWARE_QUERY_INTERVAL
                if alive and now >= next_heartbeat:
                    self._write(port, SerialProtocol.cmd_heartbeat())
                    next_heartbeat = time.monotonic() + Config.HEARTBEAT_INTERVAL
                session.wake.wait(Config.SERIAL_READ_INTERVAL)
        except Exception as exc:
            with session.lock:
                self._fail_locked(session, str(exc))
        finally:
            with session.lock:
                session.connected = False
            if port is not None:
                try:
                    self._write(port, SerialProtocol.cmd_pause_all())
                except Exception:
                    logger.warning("Final stop failed; controller heartbeat timeout is the fallback")
                finally:
                    try:
                        port.close()
                    except Exception:
                        logger.exception("Failed to close serial port")
