New-Item -ItemType Directory -Path .\Libraries\CombinedOutput -Force

Remove-Item -Path .\Libraries\CombinedOutput\* -Recurse -Force

#dotnet publish -c Release --self-contained .\Libraries\RosSharp.sln
dotnet publish -c Release --self-contained -r win-x64 .\Libraries\RosSharp.sln
#dotnet publish -c Release --runtime win-x64 .\Libraries\RosSharp.sln
#dotnet publish -c Release .\Libraries\RosSharp.sln

Remove-Item -Path .\com.siemens.ros-sharp-workflow\Plugins\* -Recurse -Force
Copy-Item .\Libraries\CombinedOutput\*.dll .\com.siemens.ros-sharp-workflow\Plugins\

$asmdefContent = @{
      name = "siemens.ros-sharp-workflow.Runtime"
      rootNamespace = ""
      references = @()
      includePlatforms = @()
      excludePlatforms = @()
      allowUnsafeCode = $false
      overrideReferences = $true
      precompiledReferences = (Get-ChildItem -Path .\Libraries\CombinedOutput -Filter *.dll | ForEach-Object { $_.Name })
      autoReferenced = $true
      defineConstraints = @()
      versionDefines = @()
      noEngineReferences = $false
  } | ConvertTo-Json

Remove-Item -Path .\com.siemens.ros-sharp-workflow\Runtime\siemens.ros-sharp-workflow.Runtime.asmdef -Force
New-Item -Path .\com.siemens.ros-sharp-workflow\Runtime -Name siemens.ros-sharp-workflow.Runtime.asmdef -ItemType File
Set-Content -Path .\com.siemens.ros-sharp-workflow\Runtime\siemens.ros-sharp-workflow.Runtime.asmdef -Value $asmdefContent


