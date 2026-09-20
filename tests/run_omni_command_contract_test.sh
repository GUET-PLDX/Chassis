#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/omni-command-contract.XXXXXX")"
trap 'rm -f "${binary}"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror \
  -I"${test_dir}/.." "${test_dir}/omni_command_contract_test.cpp" -o "${binary}"
"${binary}"
