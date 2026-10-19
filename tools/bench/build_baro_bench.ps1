#---------------------------------------------------------------------------------------------------
# Baro (BMP388 / I2C1) bench firmware build - thin wrapper over build_fc_bench.ps1.
#
# Usage
#   powershell -ExecutionPolicy Bypass -File .\tools\bench\build_baro_bench.ps1
#   powershell -ExecutionPolicy Bypass -File .\tools\bench\build_baro_bench.ps1 -Rebuild
#
# It fixes two parameters of the shared builder:
#   -Main test\bench\baro_bench_main.cpp
#   -Name baro
# which yields
#   bench object   <outdir>\obj\zzz_baro_bench_main.o
#   image          build\bench_baro\dl_fly_baro_bench.axf / .hex / .bin
#   fallback dir   test\bench\_out_baro   (when build\bench_baro is not writable from this session)
#   build log      <outdir>\bench_build.log
#
# -OutDir is passed through; everything protocol-specific lives in build_fc_bench.ps1.
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
param(
    [switch] $Rebuild,          # recompile everything even if objects look current
    [string] $OutDir = ''       # override output dir (default: build\bench_baro with the usual fallback)
)

$ErrorActionPreference = 'Stop'

$inner = Join-Path $PSScriptRoot 'build_fc_bench.ps1'
if (-not (Test-Path $inner)) {
    Write-Host ("shared builder not found: {0}" -f $inner) -ForegroundColor Red
    exit 2
}

# NOTE (2026-10-18): do NOT build the argument list as an array and splat it.
#   Windows PowerShell 5.1 splats an ARRAY positionally (only a HASHTABLE splats by name), so
#   @('-Main', <src>, '-Name', 'baro') silently bound $OutDir='-Main' / $Name='-Name' and wrote the
#   whole object tree into a repo-root "-Main" folder, then armlink rejected '-Main\obj\NNN.o'.
#   Pass the named parameters explicitly (build_fc_bench.ps1 also re-validates them).
$echo = '{0} -Main test\bench\baro_bench_main.cpp -Name baro -Rebuild:{1}' -f $inner, $Rebuild
if (-not [string]::IsNullOrEmpty($OutDir)) { $echo += (' -OutDir "{0}"' -f $OutDir) }
Write-Host ("[baro-bench] {0}" -f $echo)

& $inner -Main 'test\bench\baro_bench_main.cpp' -Name 'baro' -Rebuild:$Rebuild -OutDir $OutDir
exit $LASTEXITCODE
