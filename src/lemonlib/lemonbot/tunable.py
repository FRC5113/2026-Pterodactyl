"""Module for tunable."""

import functools
from typing import Callable, Optional

from wpilib import DriverStation


def fms_feedback(f=None, *, key: Optional[str] = None) -> Callable:
    """Execute fms_feedback."""
    if f is None:
        return functools.partial(fms_feedback, key=key)

    if not callable(f):
        raise TypeError(f"Illegal use of fms_feedback decorator on non-callable {f!r}")

    @functools.wraps(f)
    def wrapper(self):
        """Execute wrapper."""
        return f(self)

    if not DriverStation.isFMSAttached():
        wrapper._magic_feedback = True
        wrapper._magic_feedback_key = key
    return wrapper
