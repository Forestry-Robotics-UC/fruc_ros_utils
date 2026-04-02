"""USB diagnostics helper utilities.

This module offers a small standalone script to inspect connected USB devices and
estimate aggregate traffic using ``usbmon`` output.
"""

from __future__ import annotations

import subprocess
import time
from typing import Iterable, Optional

import usb.core
import usb.util


_USB_CLASS_NAMES = {
    0x00: "Defined at Interface Level",
    0x01: "Audio",
    0x02: "Communications and CDC Control",
    0x03: "Human Interface Device",
    0x05: "Physical",
    0x06: "Image",
    0x07: "Printer",
    0x08: "Mass Storage",
    0x09: "Hub",
    0x0A: "CDC-Data",
    0x0B: "Smart Card",
    0x0D: "Content Security",
    0x0E: "Video",
    0x0F: "Personal Healthcare",
    0x10: "Audio/Video Devices",
    0x11: "Billboard",
    0x12: "USB Type-C Bridge",
    0xDC: "Diagnostic Device",
    0xE0: "Wireless Controller",
    0xEF: "Miscellaneous",
    0xFE: "Application Specific",
    0xFF: "Vendor Specific",
}


def get_usb_devices() -> list[usb.core.Device]:
    """Return all currently connected USB devices."""
    devices = usb.core.find(find_all=True)
    return list(devices) if devices is not None else []


def capture_usb_traffic(duration: int = 10) -> Optional[float]:
    """Capture USB traffic in KB/s using ``usbmon``.

    Parameters
    ----------
    duration:
        Capture duration in seconds.

    Returns
    -------
    float | None
        Traffic in KB/s on success, or ``None`` if parsing/capture failed.
    """
    cmd = ["/usr/bin/usbmon", "-t", str(duration), "-s", "512", "-i", "1"]
    try:
        completed = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        print(f"Error executing usbmon: {exc}")
        return None

    if completed.returncode != 0:
        stderr = (completed.stderr or "").strip()
        print(f"usbmon failed with code {completed.returncode}: {stderr}")
        return None

    marker = "total_traffic:"
    stdout = completed.stdout or ""
    if marker not in stdout:
        print("usbmon output did not include 'total_traffic:'")
        return None

    try:
        tail = stdout.split(marker, maxsplit=1)[1].strip().split()
        total_bytes = int(tail[0])
    except (IndexError, ValueError) as exc:
        print(f"Failed to parse usbmon total_traffic field: {exc}")
        return None

    return total_bytes / 1024.0


def get_device_descriptor(device: usb.core.Device) -> Optional[bytes]:
    """Return the active configuration descriptor payload for a USB device."""
    try:
        return device.get_active_configuration().extra
    except usb.core.USBError as exc:
        print(f"Error getting device descriptor: {exc}")
        return None


def _usb_class_name(class_id: int) -> str:
    """Resolve a USB class id to a human-readable name."""
    return _USB_CLASS_NAMES.get(class_id, f"Unknown (0x{class_id:02X})")


def _print_device_report(devices: Iterable[usb.core.Device]) -> None:
    """Print a short report for each USB device."""
    for device in devices:
        try:
            descriptor = get_device_descriptor(device)
            class_name = _usb_class_name(int(getattr(device, "bDeviceClass", 0x00)))
            if descriptor:
                print(f"Device Class: {class_name}")
            else:
                print(f"Device Class: {class_name} (descriptor unavailable)")

            traffic = capture_usb_traffic()
            if traffic is not None:
                print(f"Traffic: {traffic:.2f} KB/s")
            else:
                print("Traffic: Unable to capture")

        except usb.core.USBError as exc:
            print(f"USBError: {exc}")
        except Exception as exc:
            print(f"Error: {exc}")

        print("-" * 50)
        time.sleep(1.0)


def main() -> int:
    """Entry point for standalone execution."""
    devices = get_usb_devices()
    _print_device_report(devices)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
