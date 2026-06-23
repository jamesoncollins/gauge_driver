param(
  [string]$MsysRoot = "C:\msys64",
  [switch]$NoSystemUpdate
)

$ErrorActionPreference = "Stop"

$pacman = Join-Path $MsysRoot "usr\bin\pacman.exe"
$bash = Join-Path $MsysRoot "usr\bin\bash.exe"
$packageFile = Join-Path $PSScriptRoot "..\env\msys2-ucrt-packages.txt"

if (-not (Test-Path -LiteralPath $pacman)) {
  throw "MSYS2 pacman was not found at $pacman. Install MSYS2 to $MsysRoot first: https://www.msys2.org/"
}

if (-not (Test-Path -LiteralPath $packageFile)) {
  throw "Package list not found: $packageFile"
}

$packages = Get-Content -LiteralPath $packageFile |
  Where-Object { $_ -and -not $_.TrimStart().StartsWith("#") } |
  ForEach-Object { $_.Trim() }

if (-not $NoSystemUpdate) {
  & $pacman --noconfirm -Syu
}

& $pacman --noconfirm --needed -S @packages

if (Test-Path -LiteralPath $bash) {
  & $bash -lc "gcc --version && cmake --version && ninja --version"
}
