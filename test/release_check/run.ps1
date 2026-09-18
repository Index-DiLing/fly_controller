#---------------------------------------------------------------------------------------------------
# Host-side check for the release item (release.cpp / flight_config_struct.hpp):
#   - log entry layout (no padding, frame stride, entries per slot, session limit)
#   - DLX frame codec for the new ground-mode messages (header / CRC4 / little endian payload)
#   - noise re-sync, half frame, bad CRC, legacy UnBlock
#
# Usage:  powershell -ExecutionPolicy Bypass -File .\test\release_check\run.ps1
#
# The real hardware (SPI/I2C/timers/threads) is NOT covered here; that still needs the board.
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$stage    = Join-Path $PSScriptRoot '_build'

New-Item -ItemType Directory -Force -Path $stage | Out-Null
Copy-Item -Force (Join-Path $repoRoot 'test\release_check\check.cpp') $stage

Push-Location $stage
try {
    # -I stubs 放最前面: 把 GPIO/USART/DMA 换成主机桩(其余用真实头文件)
    & g++ -std=c++11 -Wall -Wextra -I (Join-Path $PSScriptRoot 'stubs') -I (Join-Path $repoRoot '.') `
        -I (Join-Path $repoRoot 'DL_LIB') `
        -I (Join-Path $repoRoot 'DL_LIB\W25Q128') check.cpp -o check.exe
    if ($LASTEXITCODE -ne 0) { Write-Host 'BUILD FAILED' -ForegroundColor Red; exit 1 }
    & .\check.exe
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $code
