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
# See the License for the specific language governing permissions and
# limitations under the License.

# Fetch the concrete source map the wall texture is baked from.
#
# The archive is 540 MB, so it is not carried in the repository. What is
# carried is this script and the checksum below, which together pin the exact
# bytes: a texture baked from a different revision of the asset would place
# different features at the same wall coordinates, and every stitching result
# measured against it would silently be measuring a different wall.
#
# Concrete044D is CC0 under ambientCG's site-wide licence; the asset metadata
# carries no per-asset licence field. It was chosen on appearance, against
# bare cast concrete of the kind a bridge pier is finished in.
#
# It records no real-world size. The 2.5 m the bake uses is a project
# declaration, not a property of the asset - see tools/bake_wall_texture.py,
# which refuses to guess it. Only the colour map is used: no normal or
# roughness map is baked, because neither carries detail a stitch can match
# and both would cost video memory that the colour map needs.
set -euo pipefail

ASSET="Concrete044D"
VARIANT="8K-JPG"
SHA256="bd48ea1206efebd7c549fd928f6d27aee625ccbd70752dc8e53b42cc3b57698d"
DEST="${1:-${TMPDIR:-/tmp}/climbot_wall_texture}"

ARCHIVE="${DEST}/${ASSET}_${VARIANT}.zip"
mkdir -p "${DEST}"

if [ -f "${ARCHIVE}" ] && echo "${SHA256}  ${ARCHIVE}" | sha256sum -c - >/dev/null 2>&1; then
  echo "already present: ${ARCHIVE}"
else
  echo "downloading ${ASSET}_${VARIANT} (540 MB) to ${DEST}"
  # The ambientCG endpoint redirects to a CDN.  TLS handshakes to that CDN can
  # fail transiently under WSL, and curl does not retry every connection-class
  # error unless --retry-all-errors is explicit.
  curl -fL --retry 5 --retry-all-errors --connect-timeout 30 \
    "https://ambientcg.com/get?file=${ASSET}_${VARIANT}.zip" -o "${ARCHIVE}"
  echo "${SHA256}  ${ARCHIVE}" | sha256sum -c -
fi

# Only the colour map. The archive also carries normal, roughness, ambient
# occlusion, displacement, metalness and editor project files, none of which
# the bake or the SDF reads.
unzip -o -q -j "${ARCHIVE}" "*_Color.jpg" -d "${DEST}/maps"
ls -1 "${DEST}/maps"
echo "source map in ${DEST}/maps"
