"""Layer 2 input -- external loads (WP-06).

Disturbances do NOT participate in control allocation: the controller must
reject them through feedback, which is the point of the shape-maintenance study.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class DisturbanceABC(ABC):
    name: str

    @abstractmethod
    def loads(self, t: float, state: Any) -> Any:
        """Return (n_bodies, 6) force/torque in the inertial frame."""
