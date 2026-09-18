#---------------------------------------------------------------------------------------------------
# Build the host controller used by the JSBSim simulation.
#
# fc_sim.cpp pulls in the real firmware headers (control law, mixer, EKF, Madgwick,
# vertical velocity estimator) and exposes a small C ABI DLL that
# sim/jsbsim_drone_sim.py loads with ctypes.
#
# Usage:  powershell -ExecutionPolicy Bypass -File .\sim\build_fc_sim.ps1
#         set DLX_GXX=C:\path\to\g++.exe   to force a specific compiler
#
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$out      = Join-Path $PSScriptRoot 'fc_sim.dll'

function Find-Gxx {
    if ($env:DLX_GXX -and (Test-Path $env:DLX_GXX)) { return $env:DLX_GXX }
    $cmd = Get-Command g++ -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($dir in $env:PATH.Split(';')) {
        if (-not $dir) { continue }
        $p = Join-Path $dir 'g++.exe'
        if (Test-Path $p) { return $p }
    }
    foreach ($p in @('E:\MINGW\bin\g++.exe', 'C:\MinGW\bin\g++.exe',
                     'C:\msys64\mingw64\bin\g++.exe', 'D:\MINGW\bin\g++.exe')) {
        if (Test-Path $p) { return $p }
    }
    throw 'g++ not found. Install MinGW-w64 (or MSYS2) and put g++ on PATH, or set DLX_GXX.'
}

# Run a native command and return @{ Code = <int>; Out = <string> }.
# Tries the normal call operator first and falls back to a .NET Process, which also
# works in locked-down hosts where command resolution for external programs is limited.
function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $text = & $Exe @Arguments 2>&1 | Out-String
        $code = $LASTEXITCODE
        if ($null -ne $code) { return @{ Code = $code; Out = $text } }
    } catch {
        # fall through to the .NET path
    } finally {
        $ErrorActionPreference = $prev
    }
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName               = $Exe
    $psi.Arguments              = ($Arguments | ForEach-Object { if ($_ -match '[ \s]') { '"' + $_ + '"' } else { $_ } }) -join ' '
    $psi.UseShellExecute        = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError  = $true
    $proc = [System.Diagnostics.Process]::Start($psi)
    $text = $proc.StandardOutput.ReadToEnd() + $proc.StandardError.ReadToEnd()
    $proc.WaitForExit()
    return @{ Code = $proc.ExitCode; Out = $text }
}

$gxx = Find-Gxx
Write-Host "compiler: $gxx"

$args = @('-std=c++11', '-O2', '-Wall', '-Wextra', '-shared', '-static-libgcc', '-static-libstdc++',
          '-I', $repoRoot, '-I', (Join-Path $repoRoot 'DL_LIB'),
          (Join-Path $PSScriptRoot 'fc_sim.cpp'), '-o', $out)

$res = Invoke-Native -Exe $gxx -Arguments $args
if ($res.Out.Trim()) { Write-Host $res.Out.Trim() }
if ($res.Code -ne 0) { Write-Host "BUILD FAILED (exit $($res.Code))" -ForegroundColor Red; exit 1 }

Write-Host "built $out"
