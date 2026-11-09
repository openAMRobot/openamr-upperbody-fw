#!/bin/sh
# SPDX-License-Identifier: MIT
# Creates the virtual CAN interface used by the CTL-017 simulator and tests.
# Needs root (sudo) once per boot.
set -eu
IFACE="${1:-vcan0}"
modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  ip link add dev "$IFACE" type vcan
fi
ip link set up "$IFACE"
ip -details link show "$IFACE"
