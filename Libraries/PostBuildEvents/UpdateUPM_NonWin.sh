#!/usr/bin/env bash

# © Siemens AG, 2024 
# Author: Mehmet Emre Cakal <emre.cakal@siemens.com>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Resolve paths relative to the script location instead of the working directory.
#   (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

set -euo pipefail

# location: Libraries/PostBuildEvents/UpdateUPM_NonWin.sh
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

source_dirs=(
  "Libraries/MessageGeneration"
  "Libraries/RosBridgeClient"
  "Libraries/Urdf"
)

target_dir="com.siemens.ros-sharp/Runtime/Libraries"
pack_library_dir="com.siemens.ros-sharp/Runtime"

# recreate targets
rm -rf -- "$target_dir"
mkdir -p -- "$target_dir"

for source_dir in "${source_dirs[@]}"; do
  find "$REPO_ROOT/$source_dir" \
    -type d \( -name bin -o -name obj -o -name Properties \) -prune -o \
    -type f -name '*.cs' -print0 |
  while IFS= read -r -d '' file; do
    rel_from_root="${file#"$REPO_ROOT"/}"
    destination="$pack_library_dir/$rel_from_root"

    mkdir -p -- "$(dirname -- "$destination")"
    cp -f -- "$file" "$destination"
  done
done

echo "UPM Updated!"