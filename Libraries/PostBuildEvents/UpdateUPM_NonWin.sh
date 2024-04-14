#!/bin/bash
# Siemens AG 2024 - Author: Mehmet Emre Cakal <emre.cakal@siemens.com>
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
