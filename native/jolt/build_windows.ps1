param([string]$BuildDir = "")
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
if (-not $BuildDir) { $BuildDir = Join-Path $Root "build\jolt-windows" }
cmake -S (Join-Path $Root "native\jolt") -B $BuildDir -A x64
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build $BuildDir --config Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$Destination = Join-Path $Root "vendor\jolt_bridge\win_amd64"
New-Item -ItemType Directory -Force -Path $Destination | Out-Null
Copy-Item (Join-Path $BuildDir "Release\ka_jolt_bridge.dll") (Join-Path $Destination "ka_jolt_bridge.dll") -Force
Write-Host "Installed vendor/jolt_bridge/win_amd64/ka_jolt_bridge.dll"
