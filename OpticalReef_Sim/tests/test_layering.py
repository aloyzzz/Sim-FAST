"""Architectural guard (WP-11): only Layer 2 may know about an engine.

This is the one rule that keeps the cross-engine comparison meaningful, so it
is enforced mechanically rather than by review.
"""
from __future__ import annotations

import ast
import pathlib

PKG = pathlib.Path(__file__).resolve().parents[1] / "opticalreef"
ENGINE_TOKENS = ("pychrono", "chrono", "mbdyn")
ALLOWED_PREFIX = "engines"


def _imports(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text())
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_no_engine_imports_outside_layer2():
    offenders = []
    for path in PKG.rglob("*.py"):
        rel = path.relative_to(PKG)
        if rel.parts and rel.parts[0] == ALLOWED_PREFIX:
            continue
        for mod in _imports(path):
            root = mod.split(".")[0].lower()
            if root in ENGINE_TOKENS:
                offenders.append(f"{rel}: imports {mod}")
    assert not offenders, (
        "Engine imports outside opticalreef/engines/ break the cross-engine "
        "comparison:\n  " + "\n  ".join(offenders)
    )


def test_model_layer_imports_no_engine_and_no_control():
    """Layer 1 must not depend on anything above it."""
    forbidden = {"opticalreef.engines", "opticalreef.control", "opticalreef.metrics"}
    offenders = []
    for path in (PKG / "model").rglob("*.py"):
        for mod in _imports(path):
            if any(mod.startswith(f) for f in forbidden):
                offenders.append(f"{path.name}: imports {mod}")
    assert not offenders, "Layer 1 must not import upward:\n  " + "\n  ".join(offenders)
