#!/usr/bin/env python3
"""Smoke test: verify gps_denied.parm values are parsed correctly.

Does NOT require a running SITL — just validates the param file syntax
and expected key-value pairs.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PARM_FILE = REPO / "params" / "gps_denied.parm"

EXPECTED = {
    "GPS1_TYPE":       0,
    "EK3_SRC1_POSXY":  0,
    "EK3_SRC1_VELXY":  5,
    "EK3_SRC1_POSZ":   1,
    "EK3_SRC1_VELZ":   0,
    "EK3_SRC1_YAW":    1,
    "FLOW_TYPE":        1,
    "FLOW_FXSCALER":    0,
    "FLOW_FYSCALER":    0,
    "VISO_TYPE":        0,
}


def parse_parm(path: Path) -> dict[str, float]:
    params = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            params[parts[0]] = float(parts[1])
    return params


def main() -> int:
    if not PARM_FILE.exists():
        print(f"FAIL: {PARM_FILE} not found")
        return 1

    parsed = parse_parm(PARM_FILE)
    ok = True

    for key, expected_val in EXPECTED.items():
        actual = parsed.get(key)
        if actual is None:
            print(f"  FAIL: {key} missing from param file")
            ok = False
        elif actual != expected_val:
            print(f"  FAIL: {key} = {actual}, expected {expected_val}")
            ok = False
        else:
            print(f"  OK:   {key} = {int(actual)}")

    extra = set(parsed.keys()) - set(EXPECTED.keys())
    if extra:
        print(f"  WARN: unexpected params: {extra}")

    if ok:
        print(f"\nPASS: {len(EXPECTED)} params validated")
        return 0
    else:
        print("\nFAIL: some params incorrect")
        return 1


if __name__ == "__main__":
    sys.exit(main())
