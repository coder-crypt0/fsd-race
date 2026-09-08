$ErrorActionPreference = 'Stop'

# Full demo launch. Rebuilds changed ROS packages before starting FSDS.
& (Join-Path $PSScriptRoot 'Demo-FSDS.ps1') @args
exit $LASTEXITCODE
