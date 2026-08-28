param(
    [switch]$PreflightOnly
)

$ErrorActionPreference = 'Stop'
$QtRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$BackendRoot = Split-Path -Parent $QtRoot
$VenvRoot = Join-Path $QtRoot '.venv-windows'
$VenvPython = Join-Path $VenvRoot 'Scripts\python.exe'
$BootstrapPython = (Get-Command python -ErrorAction Stop).Source

function Assert-NativeSuccess {
    param(
        [int]$ExitCode,
        [string]$Action
    )
    if ($ExitCode -ne 0) {
        throw "$Action failed with exit code $ExitCode"
    }
}

& $BootstrapPython -c 'import sys; sys.exit(0 if (3, 11) <= sys.version_info[:2] < (3, 13) else 1)'
Assert-NativeSuccess $LASTEXITCODE 'Python version check (requires >=3.11,<3.13)'

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating Windows desktop environment at $VenvRoot"
    & $BootstrapPython -m venv $VenvRoot
    Assert-NativeSuccess $LASTEXITCODE 'Creating Windows virtual environment'
}

$DependencyProbe = @'
try:
    import importlib.metadata as metadata
    import PyQt6, flask, werkzeug, numpy, scipy, websocket, zeroconf, carm
    ready = metadata.version("carm") >= "0.1.20260819"
except Exception:
    ready = False
print("ready" if ready else "missing")
'@
$DependencyStatus = & $VenvPython -c $DependencyProbe 2>$null
Assert-NativeSuccess $LASTEXITCODE 'Checking desktop runtime dependencies'
if ($DependencyStatus.Trim() -ne 'ready') {
    Write-Host 'Installing desktop runtime dependencies'
    & $VenvPython -m pip install --disable-pip-version-check -r (Join-Path $QtRoot 'requirements.txt')
    Assert-NativeSuccess $LASTEXITCODE 'Installing desktop runtime dependencies'
}

if ($env:CARM_SDK_ROOT) {
    $CarmSdkRoot = $env:CARM_SDK_ROOT
    $CarmPyproject = Join-Path $CarmSdkRoot 'pyproject.toml'
    if (-not (Test-Path $CarmPyproject -PathType Leaf)) {
        throw "CARM_SDK_ROOT does not contain pyproject.toml: $CarmSdkRoot"
    }
    Write-Host "Installing pinned Carm SDK from $CarmSdkRoot"
    & $VenvPython -m pip install --disable-pip-version-check -e $CarmSdkRoot
    Assert-NativeSuccess $LASTEXITCODE 'Installing Carm development override'
}

& $VenvPython (Join-Path $QtRoot 'preflight.py')
Assert-NativeSuccess $LASTEXITCODE 'Desktop preflight'
if ($PreflightOnly) {
    exit 0
}

Set-Location $BackendRoot
& $VenvPython (Join-Path $QtRoot 'main.py') @args
$ApplicationExitCode = $LASTEXITCODE
exit $ApplicationExitCode
