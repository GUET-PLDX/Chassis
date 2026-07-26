#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
header="$module_dir/Omni.hpp"

python3 - "$header" <<'PY'
from pathlib import Path
import re
import sys

source = Path(sys.argv[1]).read_text()
thread_start = source.index("  static void ThreadFunction")
thread_end = source.index("  /**", thread_start + 10)
thread = source[thread_start:thread_end]

ordered = ("omni->Update();", "omni->SelfResolution();", "omni->PublishWheelTelemetry();")
positions = [thread.find(item) for item in ordered]
if min(positions) < 0 or positions != sorted(positions):
    raise SystemExit("telemetry publication must follow Update and SelfResolution")
unlock_after_publish = thread.find("omni->mutex_.Unlock();", positions[-1])
if unlock_after_publish < 0:
    raise SystemExit("telemetry snapshot must publish inside the chassis owner lock")

publish_start = source.index("  void PublishWheelTelemetry()")
publish_end = source.index("  /**", publish_start + 10)
publish = source[publish_start:publish_end]
required = (
    "TELEMETRY_PERIOD_US = 10000U",
    "motor_feedback_[index].omega",
    "motor_feedback_[index].received_time_us",
    "motor_feedback_[index].sequence",
    "++wheel_telemetry_sequence_",
    "wheel_telemetry_topic_.Publish(wheel_telemetry_)",
)
for token in required:
    if token not in publish:
        raise SystemExit(f"missing telemetry publication contract: {token}")
if "target_motor_omega_" in publish:
    raise SystemExit("measured telemetry must not use target wheel speed")
if not re.search(r'CreateTopic<ChassisWheelTelemetry>\(\s*"chassis_wheel_telemetry"', source):
    raise SystemExit("missing chassis_wheel_telemetry semantic topic")
PY

printf 'PASS: Omni atomic wheel telemetry publication contract\n'
