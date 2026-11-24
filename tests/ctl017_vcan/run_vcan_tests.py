# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""ctest entry of the vcan test.

Exits 77 (ctest SKIP_RETURN_CODE) when the vcan interface, python-can,
pytest or the harness binary is missing, so ctest reports SKIPPED instead
of a failure. Otherwise runs pytest and returns its result.
"""
import importlib.util
import os
import subprocess
import sys

SKIP = 77


def main():
    iface = os.environ.get("CTL017_VCAN", "vcan0")
    sim = os.environ.get("CTL017_SIM")
    missing = []
    if not sim or not os.path.exists(sim):
        missing.append("CTL017_SIM binary")
    if not os.path.exists("/sys/class/net/%s" % iface):
        missing.append("interface %s (run tools/ctl017_sim/setup_vcan.sh)" % iface)
    for mod in ("pytest", "can"):
        if importlib.util.find_spec(mod) is None:
            missing.append("python module %s" % mod)
    if missing:
        print("SKIPPED: missing " + ", ".join(missing))
        return SKIP
    here = os.path.dirname(os.path.abspath(__file__))
    return subprocess.call([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", here])


if __name__ == "__main__":
    sys.exit(main())
