param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
$hostRoot = $PSScriptRoot
$hostVenv = Join-Path $hostRoot '.venv'
& $Python -c "import sys,tkinter; assert sys.version_info >= (3,10), 'Python 3.10+ required'"
if ($LASTEXITCODE -ne 0) { throw 'Install Python 3.10+ with Tcl/Tk, then pass -Python C:\path\python.exe.' }
if (-not (Test-Path -LiteralPath (Join-Path $hostVenv 'Scripts\python.exe'))) {
    & $Python -m venv $hostVenv
    if ($LASTEXITCODE -ne 0) { throw 'Virtual environment creation failed.' }
}
$hostPython = Join-Path $hostVenv 'Scripts\python.exe'
& $hostPython -m pip install -r (Join-Path $hostRoot 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
& $hostPython -c 'import numpy,serial,tkinter,cv2,PIL; print("Host dependencies ready. start_gui.cmd opens controls without sending configuration.")'
exit $LASTEXITCODE
