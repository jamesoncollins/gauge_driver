param(
  [string]$MsysRoot = "C:\msys64"
)

$ErrorActionPreference = "Stop"

$ucrtBin = Join-Path $MsysRoot "ucrt64\bin"
$usrBin = Join-Path $MsysRoot "usr\bin"

if (-not (Test-Path -LiteralPath $ucrtBin)) {
  throw "MSYS2 UCRT64 bin directory was not found at $ucrtBin. Run scripts/setup-msys2-ucrt.ps1 first."
}

$env:MSYSTEM = "UCRT64"
$env:CHERE_INVOKING = "1"
$env:PATH = "$ucrtBin;$usrBin;$env:PATH"

Write-Host "MSYS2 UCRT64 environment is active for this PowerShell process."
Write-Host "gcc:   $((Get-Command gcc -ErrorAction SilentlyContinue).Source)"
Write-Host "cmake: $((Get-Command cmake -ErrorAction SilentlyContinue).Source)"
Write-Host "ninja: $((Get-Command ninja -ErrorAction SilentlyContinue).Source)"
