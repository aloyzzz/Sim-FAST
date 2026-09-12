"""WP-01 acceptance: the schema is normative and must reject bad configs."""
from __future__ import annotations

import json
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCHEMA = ROOT / "schemas" / "reef_config.schema.json"


def test_schema_is_valid_json():
    json.loads(SCHEMA.read_text())


def test_every_shipped_config_validates():
    jsonschema = pytest.importorskip("jsonschema")
    yaml = pytest.importorskip("yaml")
    schema = json.loads(SCHEMA.read_text())
    for cfg in sorted((ROOT / "configs").glob("*.yaml")):
        data = yaml.safe_load(cfg.read_text())
        jsonschema.validate(data, schema)


def test_mbdyn_config_requires_explicit_rho():
    """Numerical damping silently destroys the modal metrics, so rho is not
    allowed to default (docs/02 Sec 2.1)."""
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(SCHEMA.read_text())
    bad = {
        "meta": {"name": "x", "level": "L0"},
        "geometry": {"aperture_diameter": 1.0, "module_pitch": 1.0,
                     "surfaces": {"initial": {"kind": "flat"}, "target": {"kind": "flat"}}},
        "module": {"fidelity": "rigid", "mass": 1.0},
        "interface": {"joint_type": "bushing", "k": [1, 1, 1, 1, 1, 1]},
        "actuation": {"topology": "joint_only",
                      "limits": {"stroke": 1.0, "rate": 1.0, "force": 1.0}},
        "control": {"law": "open_loop", "rate_hz": 1.0},
        "maneuver": {"profile": "step", "duration": 1.0},
        "solver": {"engine": "mbdyn", "dt": 0.01, "t_end": 1.0},   # no rho
    }
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(bad, schema)
