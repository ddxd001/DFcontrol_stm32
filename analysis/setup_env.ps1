# Analysis tooling: matplotlib for parse_test1rec.py --plot / --plot-save
# Run from repo root or this folder:  powershell -ExecutionPolicy Bypass -File analysis\setup_env.ps1

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here

# Windows often sets HTTP(S) proxy to 127.0.0.1; if that port is dead, pip breaks.
# NO_PROXY=* makes urllib skip the system proxy for PyPI (see pip / requests behavior).
$env:NO_PROXY = "*"
$env:no_proxy = "*"

python -m pip install -r (Join-Path $here "requirements.txt") `
    --trusted-host "pypi.org" `
    --trusted-host "files.pythonhosted.org"

Write-Host "OK. Test: python parse_test1rec.py data.txt --plot-save gray.png --no-show" -ForegroundColor Green
