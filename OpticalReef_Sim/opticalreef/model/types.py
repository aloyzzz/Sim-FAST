"""Layer 1 — the engine-neutral model (see docs/01_MODEL_SCHEMA.md).

NORMATIVE. Every layer above Layer 2 talks in these types, and both engine
adapters consume them. Nothing here may import an engine.

Units: SI throughout. Lengths m, masses kg, forces N, torques N*m, angles rad.
All dataclasses are frozen: a ReefModel is immutable after build().
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Sequence

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]   # (w, x, y, z)
Vec6 = tuple[float, float, float, float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


class Fidelity(str, Enum):
    RIGID = "rigid"
    FLEXIBLE = "flexible"
    REDUCED = "reduced"


class JointType(str, Enum):
    BUSHING = "bushing"
    REVOLUTE_COMPLIANT = "revolute_compliant"
    RIGID_WELD = "rigid_weld"


class ActuatorKind(str, Enum):
    TRUSS = "truss"
    JOINT = "joint"
    INTRA_MODULE = "intra_module"


class SensorKind(str, Enum):
    EDGE = "edge"
    METROLOGY_RANGE = "metrology_range"
    ATTITUDE = "attitude"
    ORACLE = "oracle"


@dataclass(frozen=True)
class Frame:
    """Rigid-body pose. `quat` is (w, x, y, z), unit norm."""
    pos: Vec3
    quat: Quat = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Node:
    """An attachment or FEM node. Present only for flexible/reduced modules."""
    id: int
    body_id: int
    pos_local: Vec3
    mass: float = 0.0


@dataclass(frozen=True)
class Body:
    id: int
    frame: Frame
    mass: float
    inertia: Mat3                 # about COM, expressed in body axes
    fidelity: Fidelity = Fidelity.RIGID
    node_ids: Sequence[int] = ()
    ring: int = 0
    tile_ij: tuple[int, int] = (0, 0)


@dataclass(frozen=True)
class Interface:
    """Compliant inter-module connection. NOT an ideal joint -- idealizing it
    removes the very compliance that sets the fundamental frequency."""
    id: int
    body_a: int
    body_b: int
    frame_a: Frame                # attachment frame in body_a local coords
    frame_b: Frame
    joint_type: JointType = JointType.BUSHING
    k: Vec6 = (1e6, 1e6, 1e6, 1e5, 1e5, 1e5)
    c: Vec6 = (1e4, 1e4, 1e4, 1e3, 1e3, 1e3)
    nominal_gap: float = 0.0


@dataclass(frozen=True)
class ActuatorLimits:
    """Applied in this fixed order: rate -> stroke -> dynamics -> force.
    Reordering changes the answer under saturation; see docs/03 Sec 2.2."""
    stroke: float = 0.5           # m (or rad for joint actuators)
    rate: float = 0.01            # m/s (or rad/s)
    force: float = 500.0          # N (or N*m)
    deadband: float = 0.0
    resolution: float = 0.0       # 0 => continuous
    noise_std: float = 0.0


@dataclass(frozen=True)
class ActuatorDynamics:
    """Second-order lag between commanded and realized rest length."""
    enabled: bool = True
    fn: float = 1.0               # Hz
    zeta: float = 0.7


@dataclass(frozen=True)
class Actuator:
    id: int
    kind: ActuatorKind
    body_a: int
    body_b: int
    point_a: Vec3                 # local to body_a (truss/intra-module)
    point_b: Vec3
    axis: Vec3 = (0.0, 0.0, 1.0)  # joint actuators only
    interface_id: int | None = None
    k: float = 1e7                # actuator internal stiffness
    c: float = 1e4
    rest_ref: float = 0.0         # reference rest length (m) or angle (rad)
    limits: ActuatorLimits = field(default_factory=ActuatorLimits)
    dynamics: ActuatorDynamics = field(default_factory=ActuatorDynamics)


@dataclass(frozen=True)
class Sensor:
    id: int
    kind: SensorKind
    target_ids: Sequence[int]     # interface ids (edge) or body ids (others)
    rate_hz: float = 10.0
    noise_std: float = 0.0
    latency_s: float = 0.0
    bias: float = 0.0
    quantization: float = 0.0


@dataclass(frozen=True)
class TargetSurface:
    """Analytic optical surface. Implementations live in geometry/surfaces.py."""
    kind: str                     # paraboloid|sphere|flat|offaxis_paraboloid|conic
    params: Mapping[str, float]


@dataclass(frozen=True)
class Reduction:
    """Craig-Bampton data, computed once in Layer 1 and fed to BOTH engines so
    the reduction can never become a source of cross-engine disagreement."""
    n_fix: int
    # Per-body CB matrices, stored out-of-line in the model HDF5 file.
    payload_path: str


@dataclass(frozen=True)
class ModelMeta:
    name: str
    level: str                    # L0 | L1 | L2 | L3
    git_sha: str = ""
    units: str = "SI"


@dataclass(frozen=True)
class ReefModel:
    meta: ModelMeta
    bodies: Sequence[Body]
    nodes: Sequence[Node]
    interfaces: Sequence[Interface]
    actuators: Sequence[Actuator]
    sensors: Sequence[Sensor]
    surfaces: Mapping[str, TargetSurface]
    reduction: Reduction | None = None

    @property
    def n_bodies(self) -> int:
        return len(self.bodies)

    @property
    def n_actuators(self) -> int:
        return len(self.actuators)
