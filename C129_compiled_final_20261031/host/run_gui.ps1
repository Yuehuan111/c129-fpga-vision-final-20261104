$ErrorActionPreference = 'Stop'
$appRoot = $PSScriptRoot
$pythonCandidates = @(
    (Join-Path $appRoot '.venv\Scripts\python.exe')
)
$pythonHint = Join-Path $appRoot 'python_path.txt'
if (Test-Path -LiteralPath $pythonHint) {
    $pythonCandidates += (Get-Content -LiteralPath $pythonHint -Raw).Trim()
}
$command = Get-Command python -ErrorAction SilentlyContinue
if ($command) { $pythonCandidates += $command.Source }
$appPython = $null
foreach ($candidate in $pythonCandidates) {
    if (Test-Path -LiteralPath $candidate) {
        & $candidate -c 'import numpy, serial, tkinter, cv2, PIL' 2>$null
        if ($LASTEXITCODE -eq 0) { $appPython = $candidate; break }
    }
}
if (-not $appPython) {
    throw 'Python/numpy/pyserial/tkinter/OpenCV/Pillow not ready. Run setup_host.ps1 with an installed Python 3.10+ interpreter first.'
}
& $appPython (Join-Path $appRoot 'imu_app.py') gui
exit $LASTEXITCODE
