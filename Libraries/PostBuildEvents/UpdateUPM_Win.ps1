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
# Exclude bin/obj/Properties during the copy rather than deleting them afterwards.
#   (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

# location: Libraries/PostBuildEvents/UpdateUPM_Win.ps1
$ErrorActionPreference = 'Stop'

try {
    $repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

    $sourceDirs = @("Libraries\MessageGeneration", "Libraries\RosBridgeClient", "Libraries\Urdf")

    $targetDir      = Join-Path $repoRoot "com.siemens.ros-sharp\Runtime\Libraries"
    $packLibraryDir = Join-Path $repoRoot "com.siemens.ros-sharp\Runtime"

    if (Test-Path $targetDir) { Remove-Item -Recurse -Force $targetDir }
    New-Item -ItemType Directory -Path $targetDir -Force | Out-Null

    foreach ($sourceDir in $sourceDirs) {
        $sourceFull = Join-Path $repoRoot $sourceDir

        Get-ChildItem -Path $sourceFull -Recurse -File -Filter *.cs | ForEach-Object {
            # path relative to the source project, e.g. "MessageTypes\ROS2\Foo.cs"
            $rel = $_.FullName.Substring($sourceFull.Length + 1)

            # prune build/meta directories anywhere in the relative path
            if ($rel -match '(^|\\)(bin|obj|Properties)\\') { return }

            $destination = Join-Path $packLibraryDir (Join-Path $sourceDir $rel)
            $null = New-Item -ItemType Directory -Path (Split-Path $destination) -Force
            Copy-Item $_.FullName -Destination $destination -Force
        }
    }

    Write-Output 'UPM Updated!'
}
catch {
    Write-Error $_
    exit 1
}