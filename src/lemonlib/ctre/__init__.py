from typing import Callable

from phoenix6.status_code import StatusCode
from wpilib import RobotBase

from .pigeon import LemonPigeon
from .talonfx import LemonTalonFX

__all__ = ["LemonPigeon", "LemonTalonFX"]


def tryUntilOk(attempts: int, command: Callable[[], StatusCode]):
    # In simulation, only try once to avoid long initialization times
    if RobotBase.isSimulation():
        attempts = 1
    
    for _ in range(attempts):
        code = command()
        if code.is_ok():
            break
