"""Module for nettables."""

import threading
import time
from typing import Any, Callable, Dict

from ntcore import NetworkTableEntry, NetworkTableInstance


class SmartNT:
    """SmartNT class."""
    def __init__(
        self, root_table: str = "/", verbose: bool = False, poll_period: float = 0.02
    ):
        """Execute __init__."""
        self.nt = NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(root_table.strip("/"))
        self._entries: Dict[str, NetworkTableEntry] = {}
        self._properties: Dict[str, Dict[str, Callable]] = {}
        self.verbose = verbose
        self.poll_period = poll_period
        self._running = False

    def _get_entry(self, key: str) -> NetworkTableEntry:
        """Execute _get_entry."""
        key = str(key)
        if key not in self._entries:
            path_parts = key.strip("/").split("/")
            table = self.table
            for part in path_parts[:-1]:
                table = table.getSubTable(part)
            entry = table.getEntry(path_parts[-1])
            self._entries[key] = entry
            if self.verbose:
                print(f"[SmartNT] Created entry: /{'/'.join(path_parts)}")
        return self._entries[key]

    def set_struct_array(self, key: str, value: list, type):
        """Execute set_struct_array."""
        publisher = self.nt.getStructArrayTopic(f"{key}", type)
        publisher.publish(value)

    def put(self, key: str, value: Any):
        """Execute put."""
        entry = self._get_entry(key)
        if isinstance(value, (float, int)):
            entry.setDouble(float(value))
        elif isinstance(value, bool):
            entry.setBoolean(value)
        elif isinstance(value, str):
            entry.setString(value)
        else:
            raise TypeError(f"Unsupported value type for key '{key}': {type(value)}")
        if self.verbose:
            print(f"[SmartNT] Set {key} = {value} (type: {type(value).__name__})")

    def get(self, key: str, default: Any = None) -> Any:
        """Execute get."""
        entry = self._get_entry(key)
        if isinstance(default, (float, int)):
            return entry.getDouble(default)
        elif isinstance(default, bool):
            return entry.getBoolean(default)
        elif isinstance(default, str):
            return entry.getString(default)
        else:
            raise TypeError(
                f"Unsupported default type for key '{key}': {type(default)}"
            )

    def add_double_property(
        self, key: str, getter: Callable[[], float], setter: Callable[[float], None]
    ):
        """Execute add_double_property."""
        self._properties[key] = {"getter": getter, "setter": setter, "type": "double"}
        self._get_entry(key)

    def add_boolean_property(
        self, key: str, getter: Callable[[], bool], setter: Callable[[bool], None]
    ):
        """Execute add_boolean_property."""
        self._properties[key] = {"getter": getter, "setter": setter, "type": "boolean"}
        self._get_entry(key)

    def add_string_property(
        self, key: str, getter: Callable[[], str], setter: Callable[[str], None]
    ):
        """Execute add_string_property."""
        self._properties[key] = {"getter": getter, "setter": setter, "type": "string"}
        self._get_entry(key)

    def start(self):
        """Execute start."""
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            if self.verbose:
                print("[SmartNT] Update thread started")

    def stop(self):
        """Execute stop."""
        self._running = False
        if self._thread:
            self._thread.join()
            if self.verbose:
                print("[SmartNT] Update thread stopped")

    def _update_loop(self):

        """Execute _update_loop."""
        while self._running:
            for key, funcs in self._properties.items():
                entry = self._get_entry(key)
                getter = funcs["getter"]
                setter = funcs["setter"]
                typ = funcs["type"]

                try:
                    # Push current getter value to NetworkTables
                    val = getter()
                    if typ == "double":
                        entry.setDouble(float(val))
                    elif typ == "boolean":
                        entry.setBoolean(bool(val))
                    elif typ == "string":
                        entry.setString(str(val))
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Getter error for '{key}': {e}")

                try:
                    # Get the value from NT and update local property if different
                    if typ == "double":
                        nt_val = entry.getDouble(getter())
                        if abs(nt_val - getter()) > 1e-6:
                            setter(nt_val)
                    elif typ == "boolean":
                        nt_val = entry.getBoolean(getter())
                        if nt_val != getter():
                            setter(nt_val)
                    elif typ == "string":
                        nt_val = entry.getString(getter())
                        if nt_val != getter():
                            setter(nt_val)
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Setter error for '{key}': {e}")

            time.sleep(self.poll_period)

    # Optional legacy-style helpers
    def put_number(self, key: str, value: float):
        """Execute put_number."""
        self.put(key, float(value))

    def put_boolean(self, key: str, value: bool):
        """Execute put_boolean."""
        self.put(key, value)

    def put_string(self, key: str, value: str):
        """Execute put_string."""
        self.put(key, value)

    def get_number(self, key: str, default: float = 0.0) -> float:
        """Execute get_number."""
        return float(self.get(key, default))

    def get_boolean(self, key: str, default: bool = False) -> bool:
        """Execute get_boolean."""
        return bool(self.get(key, default))

    def get_string(self, key: str, default: str = "") -> str:
        """Execute get_string."""
        return str(self.get(key, default))

    def value(self, key: str, val_or_default: Any = None, set: bool = False) -> Any:
        """Execute value."""
        if set:
            self.put(key, val_or_default)
            return val_or_default
        else:
            return self.get(key, val_or_default)
