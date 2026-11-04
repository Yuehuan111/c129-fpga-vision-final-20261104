param([switch]$Offline,[switch]$Demo,[switch]$CheckOnly)
$ErrorActionPreference = 'Stop'
$appRoot = Join-Path $PSScriptRoot 'host'
$pythonCandidates = @((Join-Path $appRoot '.venv/Scripts/python.exe'))
$pythonHint = Join-Path $appRoot 'python_path.txt'
if (Test-Path -LiteralPath $pythonHint) {
    $pythonCandidates += (Get-Content -LiteralPath $pythonHint -Raw).Trim()
}
$pythonCommand = Get-Command python -ErrorAction SilentlyContinue
if ($pythonCommand) { $pythonCandidates += $pythonCommand.Source }
$appPython = $null
foreach ($candidate in ($pythonCandidates | Select-Object -Unique)) {
    if ($candidate -and (Test-Path -LiteralPath $candidate)) {
        & $candidate -c 'import numpy, serial, tkinter, cv2, PIL, matplotlib' 2>$null
        if ($LASTEXITCODE -eq 0) { $appPython = $candidate; break }
    }
}
if (-not $appPython) {
    throw 'Python 3.10+ with numpy, pyserial, tkinter, opencv-python, Pillow and matplotlib is required. See README.md. Set host/python_path.txt to the interpreter path.'
}
if ($CheckOnly) { Write-Output "Ready: $appPython"; exit 0 }
$launchArgs = @('-B',(Join-Path $appRoot 'demo_app.py'))
if ($Offline) { $launchArgs += '--offline' }
if ($Demo) { $launchArgs += @('--demo-image',(Join-Path $appRoot 'assets/demo_photo.jpg')) }
Push-Location -LiteralPath $appRoot
try {
    & $appPython @launchArgs
} finally {
    Pop-Location
}
exit $LASTEXITCODE
