#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${test_dir}/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/omni-command-contract-test.XXXXXX")"
trap 'rm -f "${binary}"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -pedantic \
  -I"${module_dir}" "${test_dir}/omni_command_contract_test.cpp" \
  -o "${binary}"
"${binary}"

echo "PASS: omni command contract tests"
