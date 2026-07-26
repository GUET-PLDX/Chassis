#!/usr/bin/env bash
set -euo pipefail

cxx=${CXX:-c++}
binary=$(mktemp /tmp/chassis_wheel_telemetry_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"$cxx" -std=c++20 -Wall -Wextra -Werror \
  tests/chassis_wheel_telemetry_test.cpp -o "$binary"
"$binary"

printf 'PASS: chassis wheel telemetry semantics\n'
