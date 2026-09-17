[CmdletBinding()]
param(
    [ValidateRange(2, 60)][int]$Seconds = 30,
    [string]$OutputPath = (Join-Path $PSScriptRoot 'artifacts\windows-resources.json')
)
$ErrorActionPreference = 'Stop'
$simProcesses = @(Get-Process Blocks -ErrorAction SilentlyContinue)
if ($simProcesses.Count -ne 1) { throw 'Start exactly one FSDS demo before profiling.' }
$simPid = $simProcesses[0].Id
$logicalCpus = (Get-CimInstance Win32_ComputerSystem).NumberOfLogicalProcessors
$totalRam = (Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory
$paths = @('\Processor(_Total)\% Processor Time', '\Memory\Available MBytes',
    '\Process(Blocks*)\% Processor Time', '\Process(Blocks*)\Working Set - Private')
$gpuAvailable = $true
try {
    Get-Counter '\GPU Engine(*)\Utilization Percentage' -MaxSamples 1 -ErrorAction Stop | Out-Null
    $paths += '\GPU Engine(*)\Utilization Percentage'
} catch {
    $gpuAvailable = $false
    Write-Warning 'GPU performance counters unavailable; GPU values will be null.'
}
$rows = @(Get-Counter $paths -SampleInterval 1 -MaxSamples $Seconds | ForEach-Object {
    $counters = $_.CounterSamples
    $cpu = ($counters | Where-Object Path -Like '*\processor(_total)\*').CookedValue
    $available = ($counters | Where-Object Path -Like '*\memory\*').CookedValue
    $simCpu = ($counters | Where-Object Path -Like '*\process(blocks*)\% processor time' | Measure-Object CookedValue -Sum).Sum
    $privateBytes = ($counters | Where-Object Path -Like '*\process(blocks*)\working set - private' | Measure-Object CookedValue -Sum).Sum
    $engines = @($counters | Where-Object { $_.Path -like '*\gpu engine(*)\*' -and $_.InstanceName -like "pid_${simPid}_*engtype_3D" })
    [pscustomobject]@{
        # Timestamps permit correlation with EBS and movement in the run log.
        sample_utc = $_.Timestamp.ToUniversalTime().ToString('o')
        host_cpu_percent = $cpu
        fsds_cpu_percent_host = $simCpu / $logicalCpus
        fsds_private_working_set_mib = $privateBytes / 1MB
        host_used_ram_gib = ($totalRam - $available * 1MB) / 1GB
        fsds_3d_engine_percent = if ($gpuAvailable -and $engines.Count) { ($engines | Measure-Object CookedValue -Sum).Sum } else { $null }
    }
})
$summary = [ordered]@{}
foreach ($metric in ($rows[0].PSObject.Properties.Name | Where-Object { $_ -ne 'sample_utc' })) {
    $values = @($rows | ForEach-Object { $_.$metric } | Where-Object { $null -ne $_ } | Sort-Object)
    $summary[$metric] = if ($values.Count) {
        @{ mean = [math]::Round(($values | Measure-Object -Average).Average, 3)
           p95 = [math]::Round($values[[math]::Ceiling(.95 * $values.Count) - 1], 3)
           max = [math]::Round(($values | Measure-Object -Maximum).Maximum, 3) }
    } else { $null }
}
$result = @{ recorded_utc = [DateTime]::UtcNow.ToString('o'); logical_cpus = $logicalCpus
    cpu_convention = 'CPU normalized to whole Windows host; GPU = summed FSDS PID 3D-engine counters, not all applications'
    summary = $summary; samples = $rows }
$target = [IO.Path]::GetFullPath($OutputPath)
New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
$result | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $target -Encoding utf8
$summary | ConvertTo-Json -Depth 4
Write-Host "Saved $target"
