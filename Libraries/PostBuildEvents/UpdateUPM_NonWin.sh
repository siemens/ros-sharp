#!/bin/bash

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

# Run from root folder! (ros-sharp\) 

# Define the source directories
sourceDirs=("Libraries/MessageGeneration" "Libraries/RosBridgeClient" "Libraries/Urdf")

# Define the target directory
targetDir="com.siemens.ros-sharp/Runtime/Libraries"
packLibraryDir="com.siemens.ros-sharp/Runtime"

# Recreate the target directory
rm -rf "$targetDir"
mkdir -p "$targetDir"

# Copy .cs files from the source directories to the target directories, excluding specified folders
for sourceDir in "${sourceDirs[@]}"; do
    find "$sourceDir" -type f -name "*.cs" ! -path "*/bin/*" ! -path "*/obj/*" ! -path "*/Properties/*" | while IFS= read -r file; do
        relativePath="${file#$sourceDir/}"
        destination="$packLibraryDir/$relativePath"
        mkdir -p "$(dirname "$destination")"
        cp "$file" "$destination"
    done
done

# Recursively find and delete folders named "obj" and "Properties"
foldersToDelete=("obj" "Properties")
find "$targetDir" -type d -name "obj" -o -name "Properties" -exec rm -rf {} +

# Display the contents of the target directory for verification
find "$targetDir"
