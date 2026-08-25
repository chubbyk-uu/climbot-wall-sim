#!/usr/bin/env bash
# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# Run the useful Clang static-analyzer checks only on maintained product C++.
# Generated Qt sources, vendored GoogleTest code, and tests are intentionally
# excluded: their diagnostics are neither a property of this project nor an
# actionable quality gate for it.
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
log_file="${workspace}/clang-tidy.log"
if [[ ${1:-} == "--log" ]]; then
  log_file=${2:?--log requires a path}
  shift 2
fi
if [[ $# -ne 0 ]]; then
  echo "usage: $0 [--log PATH]" >&2
  exit 2
fi

: >"${log_file}"
for package in climbot_control climbot_coverage climbot_rviz_plugins; do
  database="${workspace}/build/${package}/compile_commands.json"
  if [[ ! -f ${database} ]]; then
    echo "Missing ${database}; build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON first." >&2
    exit 2
  fi
  while IFS= read -r source; do
    clang-tidy -p "${workspace}/build/${package}" -quiet \
      -checks='clang-analyzer-*' "${source}" >>"${log_file}" 2>&1
  done < <(rg --files "${workspace}/src/${package}/src" -g '*.cpp' | sort)
done

if rg -n 'warning:|error:' "${log_file}"; then
  echo "clang-tidy found actionable product-source diagnostics; see ${log_file}." >&2
  exit 1
fi
echo "clang-tidy passed; log: ${log_file}"
