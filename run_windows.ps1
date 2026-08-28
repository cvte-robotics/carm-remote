$ErrorActionPreference = 'Stop'

& (Join-Path $PSScriptRoot 'qt_app\run_windows.ps1') @args
exit $LASTEXITCODE
