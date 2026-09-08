$ErrorActionPreference = 'Stop'

# Fast repeat demo: reuse the last successful ROS build, but still force a
# clean FSDS vehicle spawn and restart the bridge/stack/dashboard.
& (Join-Path $PSScriptRoot 'Demo-FSDS.ps1') -SkipBuild @args
exit $LASTEXITCODE
