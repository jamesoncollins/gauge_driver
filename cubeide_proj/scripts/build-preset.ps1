param(
  [Parameter(Mandatory = $true)]
  [string]$Preset,

  [switch]$Fresh,
  [switch]$NoBuild
)

$ErrorActionPreference = "Stop"

if ($Preset -like "*ucrt*") {
  . (Join-Path $PSScriptRoot "enter-msys2-ucrt.ps1")
}

$configureArgs = @("--preset", $Preset)
if ($Fresh) {
  $configureArgs = @("--fresh") + $configureArgs
}

& cmake @configureArgs

if (-not $NoBuild) {
  & cmake --build --preset $Preset
}
