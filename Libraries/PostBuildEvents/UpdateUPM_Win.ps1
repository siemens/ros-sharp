# Siemens AG 2024 - Author: Mehmet Emre Cakal <emre.cakal@siemens.com>
# Run from root folder! (ros-sharp\) 

# Define the source directories
$sourceDirs = @("Libraries\MessageGeneration", "Libraries\RosBridgeClient", "Libraries\Urdf")

# Define the target directory
$targetDir = "com.siemens.ros-sharp\Runtime\Libraries"
$packLibraryDir = "com.siemens.ros-sharp\Runtime";

# Recreate the target directory
Remove-Item -Recurse -Force $targetDir
New-Item -ItemType Directory -Path $targetDir | Out-Null

# Copy .cs files from the source directories to the target directories, excluding specified folders
foreach ($sourceDir in $sourceDirs) {
    Get-ChildItem -Path $sourceDir -Recurse -File -Exclude "bin", "obj", "Properties" -Include *.cs | ForEach-Object {
        $relativePath = $_.FullName.Substring($_.FullName.IndexOf($sourceDir))
        $destination = Join-Path $packLibraryDir $relativePath
        $null = New-Item -ItemType Directory -Path (Split-Path $destination) -Force
        Copy-Item $_.FullName -Destination $destination -Force
    }
}

# Recursively find and delete folders named "obj" and "Properties"
$foldersToDelete = @("obj", "Properties")
Get-ChildItem -Path $targetDir -Recurse -Directory | Where-Object { $foldersToDelete -contains $_.Name } | ForEach-Object {
    Remove-Item $_.FullName -Recurse -Force
}

# Display the contents of the target directory for verification
#Get-ChildItem -Path $targetDir -Recurse

Write-Output 'UPM Updated!'