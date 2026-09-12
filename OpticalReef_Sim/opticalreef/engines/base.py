"""Layer 2 boundary -- the ONLY layer permitted to import an engine.

Both MBDyn and Chrono adapters implement EngineAdapter. Everything above this
line is engine-agnostic; if a metric or controller needs something only one
engine can produce, either the metric is wrong or this ABC needs extending.
Never special-case an engine upstream of here.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence

import_guard = "engines"  # tests/test_layering.py keys off this package path


@dataclass(frozen=True)
class ModalData:
    """Free-free eigenanalysis result, rigid-body modes SEPARATED not dropped."""
    freqs_hz: Sequence[float]          # elastic modes only, ascending
    damping: Sequence[float]
    shapes: Any                        # (n_dof, n_modes) array
    n_rigid_found: int                 # MUST be 6; adapter asserts this


@dataclass
class RunResult:
    """Engine-neutral output of one simulation. Written to history.h5."""
    t: Any                             # (n_steps,)
    body_pos: Any                      # (n_steps, n_bodies, 3)
    body_quat: Any                     # (n_steps, n_bodies, 4)
    body_vel: Any
    body_omega: Any
    act_command: Any                   # (n_steps, n_act) commanded rest length
    act_realized: Any                  # after dynamics + saturation
    act_force: Any
    interface_load: Any                # (n_steps, n_interfaces, 6)
    sensor_values: Mapping[str, Any] = field(default_factory=dict)
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    manifest: Mapping[str, Any] = field(default_factory=dict)
    quarantined: bool = False
    quarantine_reason: str = ""


class Recorder(Protocol):
    def append(self, t: float, state: Any, commands: Any) -> None: ...
    def finalize(self) -> RunResult: ...


class EngineAdapter(ABC):
    """Contract for WP-03 (MBDyn) and WP-04 (Chrono)."""

    name: str

    @abstractmethod
    def build(self, model: "ReefModel", cfg: Mapping[str, Any]) -> None:
        """Translate the neutral model into the engine's own representation.

        Postconditions checked by tests (docs/02 Sec 5):
          - total mass and COM match `model` to 1e-10 relative
          - engine-side ids never leak upward
        """

    @abstractmethod
    def attach_controller(self, ctrl: "ControllerABC", rate_hz: float) -> None:
        """Wire the controller in at `rate_hz`.

        Both adapters MUST impose the same one-step command delay (see
        control/timing.py) so the engines stay comparable.
        """

    @abstractmethod
    def attach_disturbance(self, dist: "DisturbanceABC") -> None: ...

    @abstractmethod
    def run(self, recorder: Recorder) -> RunResult: ...

    @abstractmethod
    def eigenanalysis(self, n_modes: int) -> ModalData:
        """Free-free modes. MUST assert exactly 6 modes below 1e-4 Hz; anything
        else means the model is over- or under-constrained -- fail loudly."""
