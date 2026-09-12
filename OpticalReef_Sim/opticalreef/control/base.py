"""Layer 3 -- controller contract (WP-07/WP-08). Engine-agnostic by construction:
the same object instance is driven by MBDyn over a socket and by Chrono in
process, which is what makes the control comparison apples-to-apples."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass
class Measurements:
    t: float
    edge: Any = None          # (n_interfaces, 6) relative pose across interfaces
    ranges: Any = None        # sparse metrology ranges
    attitude: Any = None
    act_length: Any = None    # realized actuator lengths/angles
    oracle: Any = None        # exact state; baseline runs only


@dataclass
class Commands:
    """Commanded actuator rest lengths/angles, BEFORE limits and dynamics --
    the actuator stack owns saturation, not the controller."""
    rest_cmd: Any
    feedforward: Any = None   # logged separately so the feedback contribution
    feedback: Any = None      # is answerable after the fact
    meta: Mapping[str, Any] = field(default_factory=dict)


class ControllerABC(ABC):
    """Every law must implement anti-windup, soft start, and saturation
    awareness -- a controller that winds up during a 600 s rate-limited slew
    produces a spectacular and entirely artificial instability."""

    name: str

    @abstractmethod
    def reset(self, model: "ReefModel", cfg: Mapping[str, Any]) -> None: ...

    @abstractmethod
    def step(self, t: float, meas: Measurements) -> Commands: ...

    def diagnostics(self) -> Mapping[str, Any]:
        return {}
