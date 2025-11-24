# teensy_link.py
"""
Teensy serial transport (line-based).

Overview:
    A tiny, reliable wrapper around pySerial that lets you send a single
    line command to a Teensy (or any MCU that speaks "one line in, several
    lines out") and read back its textual response.

    Typical sketch side:
        - MCU waits for '\n'-terminated commands over Serial
        - Parses one command line (e.g., "M 30 40")
        - Prints 0..N lines of human-readable output
        - Stops printing (becomes "quiet") until next command

Responsibilities:
    - Auto-detect a likely Teensy serial port across macOS/Linux/Windows
    - Open the port with a modest timeout
    - Send exactly one newline-terminated command per call
    - Read lines until either:
        (a) a "quiet gap" passes after at least one line was received, or
        (b) an overall timeout passes with nothing received
    - Return the list of decoded lines (stripped of CR/LF)

Non-Responsibilities:
    - No binary protocol, checksums, or flow control beyond pySerial
    - No concurrency; one command at a time
    - No retries of the *command* semantics (only a single reopen on write error)

Usage:
    >>> from teensy_link import TeensyLink
    >>> link = TeensyLink()  # will try to auto-detect the port
    >>> lines = link.send_command("M 30 40")  # send a line, read response
    >>> print("\\n".join(lines))
    >>> link.close()

Dependencies:
    - pySerial: `pip install pyserial`

Notes on timeouts:
    - serial.Serial(..., timeout=1.0) bounds readline() to ~1s per wait
    - overall_timeout: cap if *no* lines ever arrive (silence from device)
    - quiet_gap: when at least one line has been read, stop after this much
      time has passed without any new bytes. This detects "end of message"
      for line-spammy sketches without a formal terminator.

Port hints:
    - macOS:   /dev/cu.usbmodem*, /dev/cu.usbserial*
    - Linux:   /dev/ttyACM*, /dev/ttyUSB*
    - Windows: COM3, COM4, ...

"""

import time
from typing import List, Optional
import serial
from serial.tools import list_ports

# Default UART speed for Teensy sketches unless your firmware specifies otherwise.
DEFAULT_BAUD = 115200


def auto_detect_port() -> Optional[str]:
    """
    Heuristically choose a likely Teensy serial device.

    The function scans available serial ports and prefers descriptors containing
    common Teensy/USB-serial markers. If no obvious match is found, it falls
    back to the first enumerated port (if any).

    Returns:
        Optional[str]: Device path (e.g., "COM5" or "/dev/cu.usbmodemXXXX"),
        or None if no ports are present at all.

    Tip:
        If this returns None or guesses the wrong device on systems with
        multiple adapters, construct TeensyLink with an explicit `port=`.
    """
    ports = list(list_ports.comports())
    for p in ports:
        # Normalize to lowercase for substring checks.
        desc = f"{p.device} {p.description}".lower()
        # Common identifiers across platforms and adapters.
        if any(k in desc for k in ("teensy", "usbmodem", "usbserial", "ttyacm", "ttyusb")):
            return p.device
    # Fallback: if we saw any ports, just take the first one.
    return ports[0].device if ports else None


class TeensyLink:
    """
    Minimal, reliable line-based link to a Teensy (or similar MCU).

    Pattern:
        send_command("M 30 40") -> ["OK", "J1=...","J2=..."]

    Args:
        port (Optional[str]): Explicit serial device. If None, `auto_detect_port()`
            is used to guess a suitable port.
        baud (int): UART baud rate. Must match the Teensy sketch (default: 115200).

    Raises:
        RuntimeError: If no serial port can be found when `port` is not provided.

    Attributes:
        port (str): The resolved serial device path.
        baud (int): The configured baud rate.
        ser (serial.Serial): pySerial handle for the open connection.
    """

    def __init__(self, port: Optional[str] = None, baud: int = DEFAULT_BAUD):
        self.port = port or auto_detect_port()
        if not self.port:
            # Fail fast so the caller can prompt the user for a specific device.
            raise RuntimeError("No serial port found. Plug Teensy or pass port= explicitly.")
        self.baud = baud
        self._open()

    def _open(self):
        """
        Open the serial port and prime the buffers.

        Implementation details:
            - timeout=1.0 bounds blocking reads (readline) to ~1 second.
            - A short sleep (~400 ms) allows certain OS/driver stacks to settle
              after (re)opening before first I/O, reducing occasional first-write drops.
            - Input/output buffers are cleared to avoid reading stale bytes
              from a previous session.
        """
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=1.0)
        time.sleep(0.4)  # Let the USB CDC/ACM stack settle on some platforms.
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        """
        Close the serial port (best-effort).

        Any exception during close is swallowed because most callers treat
        `close()` as a cleanup hint; failure here is usually non-fatal.
        """
        try:
            self.ser.close()
        except Exception:
            pass

    def send_command(self, line: str, overall_timeout: float = 6.0, quiet_gap: float = 0.3) -> List[str]:
        """
        Send one newline-terminated command and collect response lines.

        The function writes `line + "\\n"` to the serial port, then keeps calling
        `readline()` and appending decoded strings to a list. It stops when:
            (a) at least one line has arrived *and* there has been `quiet_gap`
                seconds of silence (no new bytes), or
            (b) no lines have arrived at all and `overall_timeout` has elapsed.

        Args:
            line (str): The command to send (without trailing newline).
            overall_timeout (float): Maximum seconds to wait if nothing is received.
            quiet_gap (float): Seconds of silence after first line that indicate
                "end of message".

        Returns:
            List[str]: All lines printed by the sketch (CR/LF stripped).

        Robustness:
            If a SerialException occurs during write/flush (e.g., transient disconnect),
            the port is reopened once and the payload is retried. No attempt is made
            to deduplicate commands on the firmware side; if your device might have
            acted on the first write before error, consider idempotent commands
            or sequence numbers in your protocol.
        """
        # Prepare single-line payload; MCU usually expects LF as terminator.
        payload = (line.strip() + "\n").encode("utf-8")
        try:
            self.ser.write(payload)
            self.ser.flush()
        except serial.SerialException:
            # If the port was briefly lost, try to reopen once and resend.
            self._open()
            self.ser.write(payload)
            self.ser.flush()

        lines: List[str] = []
        # Track the time of the most recent received line to detect "quiet".
        t_last = time.time()
        while True:
            # readline() blocks up to timeout (1.0s) set in _open().
            chunk = self.ser.readline()
            if chunk:
                # Decode defensively: replace decoding errors rather than raising.
                s = chunk.decode("utf-8", errors="replace").rstrip("\r\n")
                lines.append(s)
                t_last = time.time()  # Reset the quiet timer on every received line.
            else:
                now = time.time()
                if lines and (now - t_last) >= quiet_gap:
                    # We received something earlier, and we've now observed enough
                    # silence to assume the device is done talking.
                    break
                if not lines and (now - t_last) >= overall_timeout:
                    # Nothing has arrived at all within the overall window; give up.
                    break
        return lines
    

    def send_line_noreply(self, line: str) -> None:
            """
            Write one newline-terminated command without waiting for any reply.

            Used for high-rate control commands like 'V v1 v2' where the firmware
            intentionally prints nothing, to avoid blocking the vision loop.
            """
            payload = (line.strip() + "\n").encode("utf-8")
            try:
                self.ser.write(payload)
                self.ser.flush()
            except serial.SerialException:
                # If port dropped, reopen once and resend.
                self._open()
                self.ser.write(payload)
                self.ser.flush()

