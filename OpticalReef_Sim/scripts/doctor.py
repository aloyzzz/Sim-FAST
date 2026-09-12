#!/usr/bin/env python3
"""WP-00: report what this environment actually has. Run before anything else."""
from __future__ import annotations

import importlib
import shutil
import sys

PY_DEPS = ["numpy", "scipy", "yaml", "jsonschema", "h5py", "matplotlib", "jinja2", "pytest"]


def main() -> int:
    ok = True
    print(f"python              {sys.version.split()[0]}")
    for mod in PY_DEPS:
        try:
            m = importlib.import_module(mod)
            print(f"  {mod:<18} {getattr(m, '__version__', 'present')}")
        except ImportError:
            print(f"  {mod:<18} MISSING")
            ok = False

    mbdyn = shutil.which("mbdyn")
    print(f"mbdyn               {mbdyn or 'MISSING'}")
    ok &= mbdyn is not None

    try:
        import pychrono
        print(f"pychrono            {getattr(pychrono, '__version__', 'present')}")
        for sub in ("pychrono.modal", "pychrono.pardisomkl"):
            try:
                importlib.import_module(sub)
                print(f"  {sub:<18} present")
            except ImportError:
                print(f"  {sub:<18} MISSING (required for L3 / free-free solves)")
                ok = False
    except ImportError:
        print("pychrono            MISSING")
        ok = False

    print("\nSTATUS:", "ready" if ok else "NOT READY -- see WP-00 in docs/05_WORK_PACKAGES.md")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
