#---------------------------------------------------------------------------------------------------
# FlashManager host-side logic test (no hardware needed).
#
# It copies the FlashManager sources from DL_LIB into _build\, together with the simulated
# W25Q128 driver (dlx_w25q128.hpp) from this folder, builds them with desktop g++ and runs the
# test cases on a 16MB fake flash held in RAM:
#   - programming can only clear bits (1 -> 0), byte by byte;
#   - erase restores 0xFF (sector / 64KB block / whole chip);
#   - "power loss in the middle of a write" can be injected.
#
# Usage:  powershell -ExecutionPolicy Bypass -File .\test\flashfs_sim\run.ps1
#         (needs g++ in PATH; on this machine it is E:\MINGW\bin)
#
# NOTE: sources are copied fresh from DL_LIB on every run, so just re-run after editing
#       dlx_flash_manager.hpp / dlx_flash_manager_config.h and the test uses the new code.
#
# This script file is intentionally ASCII-only: Windows PowerShell 5.1 reads .ps1 files as ANSI
# when they have no BOM, which would garble non-ASCII text and can break parsing.
# See README.md (UTF-8) for the details.
#---------------------------------------------------------------------------------------------------
$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$stage    = Join-Path $PSScriptRoot '_build'

$sources = @(
    'DL_LIB\dlx_exception.h',
    'DL_LIB\W25Q128\dlx_w25q128_config.h',
    'DL_LIB\W25Q128\dlx_flash_manager_config.h',
    'DL_LIB\W25Q128\dlx_flash_manager.hpp'
)

New-Item -ItemType Directory -Force -Path $stage | Out-Null
foreach ($file in $sources) {
    Copy-Item -Force (Join-Path $repoRoot $file) $stage
}
Copy-Item -Force (Join-Path $PSScriptRoot 'dlx_w25q128.hpp') $stage
Copy-Item -Force (Join-Path $PSScriptRoot 'test_main.cpp') $stage

Push-Location $stage
try {
    & g++ -std=c++11 -Wall -Wextra -O1 -o test_main.exe test_main.cpp
    if ($LASTEXITCODE -ne 0) {
        Write-Host 'BUILD FAILED' -ForegroundColor Red
        exit 1
    }
    & .\test_main.exe
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

if ($code -eq 0) {
    Write-Host 'ALL CHECKS PASSED' -ForegroundColor Green
} else {
    Write-Host 'SOME CHECKS FAILED (see [FAIL] lines above)' -ForegroundColor Red
}
exit $code
