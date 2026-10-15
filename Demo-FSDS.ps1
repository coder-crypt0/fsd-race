[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [switch]$KeepSimulator,
    [switch]$Research,
    [switch]$NoBrowser,
    [int]$DashboardPort = 8321,
    [ValidateRange(0, 900)]
    [int]$EvaluationSeconds = 0
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
$phaseClock = [Diagnostics.Stopwatch]::StartNew()
$startupPhases = [Collections.Generic.List[object]]::new()
function Add-StartupPhase([string]$Name) {
    $startupPhases.Add(@{ event = $Name; elapsed_s = $phaseClock.Elapsed.TotalSeconds;
        unix_s = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0 })
}
Add-StartupPhase 'launcher_entered'
if ($Research -and $EvaluationSeconds -lt 10) { throw '-Research requires -EvaluationSeconds between 10 and 900.' }

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
$settings = Get-Content -LiteralPath $settingsSource -Raw | ConvertFrom-Json
$configuredRpcPort = $settings.ApiServerPort
if (-not $configuredRpcPort) { $configuredRpcPort = 41451 }
if ($configuredRpcPort -lt 1 -or $configuredRpcPort -gt 65535) { throw 'Invalid ApiServerPort in FSDS settings.' }

function Test-RpcPortAvailable([int]$Port) {
    $listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Any, $Port)
    try {
        $listener.Start()
        return $true
    } catch [Net.Sockets.SocketException] {
        return $false
    } finally {
        $listener.Stop()
    }
}

$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Write-Host "FSD demo starting (C: free: $freeGb GB)" -ForegroundColor Cyan
Write-Host 'Ctrl+C stops the ROS stack. The simulator is also stopped unless -KeepSimulator was supplied.'

# Always use a fresh vehicle spawn. Reusing an old AirSim process preserves the
# previous crash/EBS location and makes a healthy stack appear broken.
Stop-FsdsProcesses
Start-Sleep -Seconds 2
$rpcPort = $null
foreach ($candidate in @($configuredRpcPort) + @(42000..42999)) {
    if (Test-RpcPortAvailable $candidate) {
        $rpcPort = $candidate
        break
    }
}
if (-not $rpcPort) { throw 'No bindable FSDS RPC port found in the configured port or fallback range 42000-42999.' }
if ($rpcPort -ne $configuredRpcPort) {
    Write-Host "RPC port $configuredRpcPort is unavailable (possibly reserved by Windows); using $rpcPort for this run." -ForegroundColor Yellow
}

# FSDS reads settings from this fixed user-directory name. Keep the binary-folder
# copy equal, but leave the checked-in settings unchanged for the next launch.
$settings | Add-Member -NotePropertyName ApiServerPort -NotePropertyValue $rpcPort -Force
$runtimeSettings = $settings | ConvertTo-Json -Depth 32
New-Item -ItemType Directory -Path $settingsTargetDir -Force | Out-Null
[IO.File]::WriteAllText($settingsTarget, $runtimeSettings, [Text.UTF8Encoding]::new($false))
Copy-Item -LiteralPath $settingsTarget -Destination $binarySettingsTarget -Force
Start-Process -FilePath $simulatorExe -ArgumentList @(
    '/Game/TrainingMap?listen', '-WINDOWED', '-ResX=960', '-ResY=540',
    '-ExecCmds="sg.ShadowQuality 0,sg.PostProcessQuality 0,sg.EffectsQuality 0,r.MotionBlurQuality 0,t.MaxFPS 30"'
) | Out-Null
Add-StartupPhase 'simulator_spawned'

Write-Host 'Waiting for the FSDS RPC server...' -ForegroundColor DarkCyan
$rpcReady = $false
for ($i = 0; $i -lt 30; $i++) {
    Start-Sleep -Seconds 2
    if (Get-NetTCPConnection -LocalPort $rpcPort -State Listen -ErrorAction SilentlyContinue) {
        $rpcReady = $true
        break
    }
}
if (-not $rpcReady) {
    Stop-FsdsProcesses
    throw "FSDS did not open RPC port $rpcPort. Inspect FSOnline/Saved/Logs/FSOnline.log before retrying."
}
Add-StartupPhase 'rpc_listening'

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
# PowerShell here-strings can retain CRLF; bash interprets the CR in `set -e`
# as an invalid option. Normalize before crossing the Windows/WSL boundary.
$syncScript = $syncScript.Replace([string][char]13, '')
wsl -d kali-linux -u root -- bash -elc $syncScript
if ($LASTEXITCODE -ne 0) {
    Stop-FsdsProcesses
    throw 'Failed to synchronize the project into Kali WSL.'
}
Add-StartupPhase 'source_sync_complete'

$buildArg = if ($SkipBuild) { '--skip-build' } else { '--build' }
$portArg = "--dashboard-port=$DashboardPort"
$evaluationArg = "--evaluation-seconds=$EvaluationSeconds"
$runnerArgs = @($buildArg, $portArg, $evaluationArg)
$runnerArgs += "--rpc-port=$rpcPort"
if ($Research) { $runnerArgs += '--research' }
$browserJob = $null
$resourceJob = $null
if ($Research) {
    $resourceJob = Start-Job -ScriptBlock {
        param($script, $seconds, $output)
        & $script -Seconds $seconds -OutputPath $output
    } -ArgumentList (Join-Path $projectRoot 'Measure-FSDS.ps1'), ([math]::Min(120, [int]($EvaluationSeconds / 2))), (Join-Path $workspace 'demo_logs\windows-resources.json')
}

# Open the UI after ROS has had time to build/start. Opening early is harmless;
# the browser will show the page as soon as the dashboard begins listening.
if (-not $NoBrowser) {
$browserJob = Start-Job -ScriptBlock {
    param($port)
    Start-Sleep -Seconds 12
    Start-Process "http://localhost:$port"
} -ArgumentList $DashboardPort
}

try {
    Write-Host 'Starting bridge, autonomous stack, recorder, and dashboard...' -ForegroundColor Green
    Write-Host 'Live logs follow. Press Ctrl+C once to stop.' -ForegroundColor DarkGray
    Add-StartupPhase 'wsl_runner_invoked'
    wsl -d kali-linux -u root -- bash $wslRunner @runnerArgs
    if ($LASTEXITCODE -ne 0) {
        throw "The demo stack exited with code $LASTEXITCODE. See fsd_ws\demo_logs for logs."
    }
}
finally {
    if ($browserJob) {
        Stop-Job $browserJob -ErrorAction SilentlyContinue
        Remove-Job $browserJob -Force -ErrorAction SilentlyContinue
    }
    if ($resourceJob) {
        if ($resourceJob.State -eq 'Running') { Stop-Job $resourceJob }
        Receive-Job $resourceJob -ErrorAction Continue
        Remove-Job $resourceJob -Force
    }
    wsl -d kali-linux -u root -- bash -lc "docker rm -f fsd-demo >/dev/null 2>&1 || true"
    # Copy the latest small text logs back to Windows. Build products and ROS
    # caches remain on ext4; only human-readable diagnostics are copied.
    $windowsLogDir = Join-Path $workspace 'demo_logs'
    New-Item -ItemType Directory -Path $windowsLogDir -Force | Out-Null
    $wslWindowsLogs = '/mnt/c/' + ($windowsLogDir.Substring(3) -replace '\\', '/')
    wsl -d kali-linux -u root -- bash -lc "if test -d /root/fsd_ws/demo_logs/latest; then mkdir -p '$wslWindowsLogs'; find -L /root/fsd_ws/demo_logs/latest -maxdepth 1 -type f \( -name '*.log' -o -name '*.json' -o -name '*.ndjson' \) -exec cp -t '$wslWindowsLogs' {} +; fi"
    Add-StartupPhase 'runner_finished'
    $startupPhases | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $windowsLogDir 'windows-startup.json') -Encoding utf8
    if (-not $KeepSimulator) {
        Stop-FsdsProcesses
    }
    $freeAfterGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
    Write-Host "Demo stopped (C: free: $freeAfterGb GB)." -ForegroundColor Cyan
}
