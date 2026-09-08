[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [switch]$KeepSimulator,
    [int]$DashboardPort = 8321
)

$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$workspace = Join-Path $projectRoot 'fsd_ws'
$settingsSource = Join-Path $workspace 'fsds\settings.json'
$simulatorExe = Join-Path $env:USERPROFILE 'FSDS\FSDS.exe'
$settingsTargetDir = Join-Path $env:USERPROFILE 'Formula-Student-Driverless-Simulator'
$settingsTarget = Join-Path $settingsTargetDir 'settings.json'
$binarySettingsTarget = Join-Path (Split-Path $simulatorExe -Parent) 'settings.json'
$wslRunner = '/root/fsd_ws/tools/run_fsds_demo.sh'

function Stop-FsdsProcesses {
    Get-Process -Name Blocks, FSDS -ErrorAction SilentlyContinue |
        Stop-Process -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $simulatorExe)) {
    throw "FSDS.exe was not found at '$simulatorExe'."
}
if (-not (Test-Path -LiteralPath $settingsSource)) {
    throw "FSDS settings were not found at '$settingsSource'."
}

$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Write-Host "FSD demo starting (C: free: $freeGb GB)" -ForegroundColor Cyan
Write-Host 'Ctrl+C stops the ROS stack. The simulator is also stopped unless -KeepSimulator was supplied.'

# FSDS reads settings from this fixed user-directory name, not from the binary folder.
New-Item -ItemType Directory -Path $settingsTargetDir -Force | Out-Null
Copy-Item -LiteralPath $settingsSource -Destination $settingsTarget -Force
# This FSDS binary also accepts settings beside FSDS.exe. A stale file here
# previously enabled 2x 785px cameras and two lidars, overloading the simulator
# until both ROS bridge RPC calls timed out. Keep both lookup locations equal.
Copy-Item -LiteralPath $settingsSource -Destination $binarySettingsTarget -Force

# Always use a fresh vehicle spawn. Reusing an old AirSim process preserves the
# previous crash/EBS location and makes a healthy stack appear broken.
Stop-FsdsProcesses
Start-Sleep -Seconds 2
Start-Process -FilePath $simulatorExe -ArgumentList @(
    '/Game/TrainingMap?listen', '-WINDOWED', '-ResX=1280', '-ResY=720'
) | Out-Null

Write-Host 'Waiting for the FSDS RPC server...' -ForegroundColor DarkCyan
$rpcReady = $false
for ($i = 0; $i -lt 30; $i++) {
    Start-Sleep -Seconds 2
    if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
        $rpcReady = $true
        break
    }
}
if (-not $rpcReady) {
    Stop-FsdsProcesses
    throw 'FSDS started, but its AirSim RPC server did not open port 41451 within 60 seconds.'
}

# Mirror source to the fast ext4 workspace. Build/install are deliberately not
# copied or deleted; this keeps repeat demos fast and avoids growing the VHDX.
$wslProject = '/mnt/c/' + ($projectRoot.Substring(3) -replace '\\', '/')
$syncScript = @"
set -e
mkdir -p /root/fsd_ws/src
rsync -a --delete '$wslProject/fsd_ws/src/fsd_cpp/' /root/fsd_ws/src/fsd_cpp/
rsync -a --delete '$wslProject/fsd_ws/src/fsd_msgs/' /root/fsd_ws/src/fsd_msgs/
rsync -a --delete '$wslProject/fsd_ws/src/fsd_stack/' /root/fsd_ws/src/fsd_stack/
rsync -a --delete '$wslProject/fsd_ws/fsds/' /root/fsd_ws/fsds/
rsync -a --delete '$wslProject/fsd_ws/tools/' /root/fsd_ws/tools/
chmod +x '$wslRunner'
"@
wsl -d kali-linux -u root -- bash -lc $syncScript
if ($LASTEXITCODE -ne 0) {
    Stop-FsdsProcesses
    throw 'Failed to synchronize the project into Kali WSL.'
}

$buildArg = if ($SkipBuild) { '--skip-build' } else { '--build' }
$portArg = "--dashboard-port=$DashboardPort"

# Open the UI after ROS has had time to build/start. Opening early is harmless;
# the browser will show the page as soon as the dashboard begins listening.
$browserJob = Start-Job -ScriptBlock {
    param($port)
    Start-Sleep -Seconds 12
    Start-Process "http://localhost:$port"
} -ArgumentList $DashboardPort

try {
    Write-Host 'Starting bridge, autonomous stack, recorder, and dashboard...' -ForegroundColor Green
    Write-Host 'Live logs follow. Press Ctrl+C once to stop.' -ForegroundColor DarkGray
    wsl -d kali-linux -u root -- bash $wslRunner $buildArg $portArg
    if ($LASTEXITCODE -ne 0) {
        throw "The demo stack exited with code $LASTEXITCODE. See fsd_ws\demo_logs for logs."
    }
}
finally {
    Stop-Job $browserJob -ErrorAction SilentlyContinue
    Remove-Job $browserJob -Force -ErrorAction SilentlyContinue
    wsl -d kali-linux -u root -- bash -lc "docker rm -f fsd-demo >/dev/null 2>&1 || true"
    # Copy the latest small text logs back to Windows. Build products and ROS
    # caches remain on ext4; only human-readable diagnostics are copied.
    $windowsLogDir = Join-Path $workspace 'demo_logs'
    New-Item -ItemType Directory -Path $windowsLogDir -Force | Out-Null
    $wslWindowsLogs = '/mnt/c/' + ($windowsLogDir.Substring(3) -replace '\\', '/')
    wsl -d kali-linux -u root -- bash -lc "if test -d /root/fsd_ws/demo_logs/latest; then mkdir -p '$wslWindowsLogs'; cp -f /root/fsd_ws/demo_logs/latest/*.log '$wslWindowsLogs/' 2>/dev/null || true; fi"
    if (-not $KeepSimulator) {
        Stop-FsdsProcesses
    }
    $freeAfterGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
    Write-Host "Demo stopped (C: free: $freeAfterGb GB)." -ForegroundColor Cyan
}
