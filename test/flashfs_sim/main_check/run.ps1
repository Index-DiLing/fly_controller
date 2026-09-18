#---------------------------------------------------------------------------------------------------
# Dry-run the on-board self test (main_flash_fs.cpp) on the desktop, no hardware needed.
#
# How it works:
#   - the STM32/USART/GPIO/SPI headers are replaced by the stubs in this folder (they print to the
#     terminal instead of the UART and simulate a 16MB flash in RAM);
#   - main_flash_fs.cpp is compiled with -Dmain=onboard_test_main and runner.cpp provides the real
#     main() plus a delay_ms that throws once the test reaches its final idle loop, so the run ends.
#
# Usage:  powershell -ExecutionPolicy Bypass -File .\test\flashfs_sim\main_check\run.ps1
#
# The output is exactly what the board would print over USART1, and it ends with the pass/fail
# summary. This script only checks that the on-board test itself is correct; the real hardware test
# still has to be run once on the board (timings, SPI speed, electrical issues).
#
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
$stage    = Join-Path $PSScriptRoot '_build'

$sources = @(
    'DL_LIB\dlx_exception.h',
    'DL_LIB\W25Q128\dlx_w25q128_config.h',
    'DL_LIB\W25Q128\dlx_flash_manager_config.h',
    'DL_LIB\W25Q128\dlx_flash_manager.hpp',
    'main_flash_fs.cpp'
)

New-Item -ItemType Directory -Force -Path (Join-Path $stage 'W25Q128') | Out-Null
foreach ($file in $sources) {
    Copy-Item -Force (Join-Path $repoRoot $file) $stage
}
# real driver header is replaced by the simulated one that lives under W25Q128\
Copy-Item -Force (Join-Path $PSScriptRoot 'W25Q128\dlx_w25q128.hpp') (Join-Path $stage 'W25Q128')
Copy-Item -Force (Join-Path $repoRoot 'DL_LIB\W25Q128\dlx_w25q128_config.h') (Join-Path $stage 'W25Q128')
Copy-Item -Force (Join-Path $repoRoot 'DL_LIB\W25Q128\dlx_flash_manager_config.h') (Join-Path $stage 'W25Q128')
Copy-Item -Force (Join-Path $repoRoot 'DL_LIB\W25Q128\dlx_flash_manager.hpp') (Join-Path $stage 'W25Q128')
Get-ChildItem -Path $PSScriptRoot -File -Filter *.h* | Copy-Item -Destination $stage -Force
Copy-Item -Force (Join-Path $PSScriptRoot 'runner.cpp') $stage

Push-Location $stage
try {
    & g++ -std=c++11 -Wall -Wextra -I. -Dmain=onboard_test_main -c main_flash_fs.cpp -o obj_test.o
    if ($LASTEXITCODE -ne 0) { Write-Host 'BUILD FAILED (test)' -ForegroundColor Red; exit 1 }
    & g++ -std=c++11 -Wall -Wextra -I. -c runner.cpp -o obj_runner.o
    if ($LASTEXITCODE -ne 0) { Write-Host 'BUILD FAILED (runner)' -ForegroundColor Red; exit 1 }
    & g++ -o run.exe obj_test.o obj_runner.o
    if ($LASTEXITCODE -ne 0) { Write-Host 'LINK FAILED' -ForegroundColor Red; exit 1 }
    & .\run.exe
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $code
