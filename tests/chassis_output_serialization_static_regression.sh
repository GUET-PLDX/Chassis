#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

extract_block() {
  local file=$1
  local start_pattern=$2

  awk -v start_pattern="$start_pattern" '
    $0 ~ start_pattern { found = 1 }
    found {
      print
      opens = gsub(/{/, "{")
      closes = gsub(/}/, "}")
      if (opens > 0) {
        started = 1
      }
      depth += opens - closes
      if (started && depth == 0) {
        exit
      }
    }
  ' "$file"
}

line_number() {
  local body=$1
  local needle=$2
  local occurrence=${3:-first}
  local matches

  matches=$(grep -nF "$needle" <<<"$body" || true)
  if [[ $occurrence == last ]]; then
    matches=$(tail -n 1 <<<"$matches")
  else
    matches=$(head -n 1 <<<"$matches")
  fi
  [[ -n $matches ]] || return 1
  printf '%s\n' "${matches%%:*}"
}

check_owner_thread() {
  local file=$1
  local variant=$2
  local object=$3
  local output_call=$4
  local body lock_line calculation_line output_line unlock_line

  body=$(extract_block "$file" 'static void ThreadFunction')
  lock_line=$(line_number "$body" "$object->mutex_.Lock();" last) || {
    printf 'missing: %s owner lock\n' "$variant"
    return 1
  }
  calculation_line=$(line_number "$body" "$object->UpdateCMD();") || {
    printf 'missing: %s control calculation\n' "$variant"
    return 1
  }
  output_line=$(line_number "$body" "$object->$output_call;") || {
    printf 'missing: %s motor output\n' "$variant"
    return 1
  }
  unlock_line=$(line_number "$body" "$object->mutex_.Unlock();" last) || {
    printf 'missing: %s owner unlock\n' "$variant"
    return 1
  }

  if ((lock_line >= calculation_line)); then
    printf 'misordered: %s calculation occurs before lock\n' "$variant"
    return 1
  fi
  if ((calculation_line >= output_line)); then
    printf 'misordered: %s output occurs before control calculation\n' "$variant"
    return 1
  fi
  if ((output_line >= unlock_line)); then
    printf 'misordered: %s unlock occurs before output\n' "$variant"
    return 1
  fi
}

check_lost_ctrl_callback() {
  local file=$1
  local variant=$2
  local object=$3
  local body lock_line relax_line lost_line unlock_line

  body=$(extract_block "$file" 'auto lost_ctrl_callback')
  lock_line=$(line_number "$body" "$object->mutex_.Lock();") || {
    printf 'missing: %s lost-control callback lock\n' "$variant"
    return 1
  }
  relax_line=$(line_number "$body" "$object->chassis_event_ = ChassisMode::RELAX;") || {
    printf 'missing: %s lost-control callback RELAX assignment\n' "$variant"
    return 1
  }
  lost_line=$(line_number "$body" "$object->LostCtrl();") || {
    printf 'missing: %s lost-control callback LostCtrl call\n' "$variant"
    return 1
  }
  unlock_line=$(line_number "$body" "$object->mutex_.Unlock();") || {
    printf 'missing: %s lost-control callback unlock\n' "$variant"
    return 1
  }

  if ! ((lock_line < relax_line && relax_line < lost_line &&
         lost_line < unlock_line)); then
    printf 'misordered: %s lost-control callback must Lock -> RELAX -> LostCtrl -> Unlock\n' \
      "$variant"
    return 1
  fi
}

check_track_output() {
  local file=$1
  local body lock_line relax_check_line relax_call_line first_unlock_line
  local control_line last_unlock_line

  body=$(extract_block "$file" 'void ControlTrack')
  lock_line=$(line_number "$body" 'mutex_.Lock();') || {
    printf 'missing: Mecanum ControlTrack lock\n'
    return 1
  }
  relax_check_line=$(line_number "$body" 'chassis_event_ == ChassisMode::RELAX') || {
    printf 'missing: Mecanum ControlTrack RELAX recheck\n'
    return 1
  }
  relax_call_line=$(line_number "$body" 'track_motor_->Relax();') || {
    printf 'missing: Mecanum ControlTrack relax submission\n'
    return 1
  }
  first_unlock_line=$(line_number "$body" 'mutex_.Unlock();') || {
    printf 'missing: Mecanum ControlTrack unlock\n'
    return 1
  }
  control_line=$(line_number "$body" 'track_motor_->Control(track_motor_cmd_);') || {
    printf 'missing: Mecanum ControlTrack control submission\n'
    return 1
  }
  last_unlock_line=$(line_number "$body" 'mutex_.Unlock();' last) || return 1

  if ! ((lock_line < relax_check_line && relax_check_line < relax_call_line &&
         relax_call_line < first_unlock_line)); then
    printf 'misordered: Mecanum ControlTrack observes RELAX after unlock\n'
    return 1
  fi
  if ((control_line >= last_unlock_line)); then
    printf 'misordered: Mecanum ControlTrack unlock occurs before control\n'
    return 1
  fi
}

check_all() {
  local source_dir=$1
  local status=0

  check_owner_thread "$source_dir/Omni.hpp" Omni omni 'OutputToDynamics()' || status=1
  check_owner_thread "$source_dir/Mecanum.hpp" Mecanum mecanum \
    'OutputToDynamics()' || status=1
  check_owner_thread "$source_dir/Helm.hpp" Helm helm 'Output()' || status=1
  check_lost_ctrl_callback "$source_dir/Omni.hpp" Omni omni || status=1
  check_lost_ctrl_callback "$source_dir/Mecanum.hpp" Mecanum mecanum || status=1
  check_lost_ctrl_callback "$source_dir/Helm.hpp" Helm helm || status=1
  check_track_output "$source_dir/Mecanum.hpp" || status=1

  return "$status"
}

run_mutation_checks() {
  local mutation_dir=$1

  cp "$ROOT_DIR/Omni.hpp" "$mutation_dir/Omni.hpp"
  cp "$ROOT_DIR/Mecanum.hpp" "$mutation_dir/Mecanum.hpp"
  cp "$ROOT_DIR/Helm.hpp" "$mutation_dir/Helm.hpp"

  perl -0pi -e \
    's/(      omni->OutputToDynamics\(\);\n)(      omni->mutex_\.Unlock\(\);\n)/$2$1/' \
    "$mutation_dir/Omni.hpp"
  if check_owner_thread "$mutation_dir/Omni.hpp" Omni omni \
    'OutputToDynamics()' >/dev/null; then
    printf 'mutation survived: Omni unlock-before-output\n'
    return 1
  fi

  perl -0pi -e \
    's/(          helm->LostCtrl\(\);\n)(          helm->mutex_\.Unlock\(\);\n)/$2$1/' \
    "$mutation_dir/Helm.hpp"
  if check_lost_ctrl_callback "$mutation_dir/Helm.hpp" Helm helm >/dev/null; then
    printf 'mutation survived: Helm callback unlock-before-LostCtrl\n'
    return 1
  fi

  perl -0pi -e \
    's/(    track_motor_->Control\(track_motor_cmd_\);\n)(    mutex_\.Unlock\(\);\n)/$2$1/' \
    "$mutation_dir/Mecanum.hpp"
  if check_track_output "$mutation_dir/Mecanum.hpp" >/dev/null; then
    printf 'mutation survived: Mecanum track unlock-before-control\n'
    return 1
  fi
}

if ! check_all "$ROOT_DIR"; then
  exit 1
fi

MUTATION_DIR=$(mktemp -d)
trap 'rm -rf "$MUTATION_DIR"' EXIT
run_mutation_checks "$MUTATION_DIR"

printf 'PASS: Omni, Mecanum, Helm, callbacks, and Mecanum ControlTrack are serialized\n'
