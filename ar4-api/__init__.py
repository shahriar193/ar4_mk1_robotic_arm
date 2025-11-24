# ar4_api/__init__.py
"""
AR4 2-DOF arm API package.
Exposes Arm2D and TeensyLink for convenience.
"""

from .arm2d import Arm2D
from .teensy_link import TeensyLink

__all__ = ["Arm2D", "TeensyLink"]
