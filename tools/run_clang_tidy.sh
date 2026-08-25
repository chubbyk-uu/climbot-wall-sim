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
if ! command -v clang-tidy >/dev/null 2>&1; then
  echo "clang-tidy is required for this quality gate but was not found in PATH." >&2
  exit 2
fi

: >"${log_file}"
analyzed=0
for package in climbot_control climbot_coverage climbot_inspection climbot_rviz_plugins; do
  database="${workspace}/build/${package}/compile_commands.json"
  if [[ ! -f ${database} ]]; then
    echo "Missing ${database}; build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON first." >&2
    exit 2
  fi
  found=0
  while IFS= read -r -d '' source; do
    found=1
    analyzed=$((analyzed + 1))
    clang-tidy -p "${workspace}/build/${package}" -quiet \
      -checks='clang-analyzer-*' "${source}" >>"${log_file}" 2>&1
  done < <(find "${workspace}/src/${package}/src" -type f -name '*.cpp' -print0 | sort -z)
  if [[ ${found} -eq 0 ]]; then
    echo "No product C++ sources found under ${workspace}/src/${package}/src." >&2
    exit 2
  fi
done

if grep -nE 'warning:|error:' "${log_file}"; then
  echo "clang-tidy found actionable product-source diagnostics; see ${log_file}." >&2
  exit 1
fi
if [[ ${analyzed} -eq 0 ]]; then
  echo "clang-tidy did not analyze any product source." >&2
  exit 2
fi
echo "clang-tidy passed (${analyzed} files); log: ${log_file}"
