#---------------------------------------------------------------------------------------------------
# One shot: build the host controller, check the airframe, run every scenario.
#
#   powershell -ExecutionPolicy Bypass -File .\sim\run_sim.ps1
#   powershell -ExecutionPolicy Bypass -File .\sim\run_sim.ps1 -Clean
#
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
param(
    [switch]$Clean,
    [string[]]$Scenarios
)

$ErrorActionPreference = 'Stop'

# Python prints UTF-8 (Chinese scenario names); make the console agree.
try {
    [Console]::OutputEncoding = [System.Text.Encoding]::UTF8
    $env:PYTHONIOENCODING = 'utf-8'
} catch { }

$here = $PSScriptRoot
$repo = (Resolve-Path (Join-Path $here '..')).Path

function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $text = & $Exe @Arguments 2>&1 | Out-String
        $code = $LASTEXITCODE
        if ($null -ne $code) { return @{ Code = $code; Out = $text } }
    } catch { }
    finally { $ErrorActionPreference = $prev }

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

function Need-Python {
    if ($env:DLX_PYTHON -and (Test-Path $env:DLX_PYTHON)) { return $env:DLX_PYTHON }
    $c = Get-Command python -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    foreach ($p in @('E:\Python3.10\python.exe', 'C:\Python310\python.exe')) {
        if (Test-Path $p) { return $p }
    }
    throw 'python not found. Set DLX_PYTHON to the interpreter that has jsbsim/numpy/matplotlib.'
}

$py = Need-Python
Write-Host "[1/4] build fc_sim.dll" -ForegroundColor Cyan
& (Join-Path $here 'build_fc_sim.ps1')

Write-Host "[2/4] plant_check" -ForegroundColor Cyan
$r = Invoke-Native -Exe $py -Arguments @((Join-Path $here 'plant_check.py'))
Write-Host $r.Out.Trim()
if ($r.Code -ne 0) { Write-Host 'plant check FAILED' -ForegroundColor Red; exit 1 }

Write-Host "[3/4] closed loop scenarios" -ForegroundColor Cyan
$args2 = @((Join-Path $here 'jsbsim_drone_sim.py'))
if ($Clean) { $args2 += '--clean' }
if ($Scenarios) { $args2 += '-s'; $args2 += $Scenarios }
$r = Invoke-Native -Exe $py -Arguments $args2
Write-Host $r.Out.Trim()
if ($r.Code -ne 0) { exit $r.Code }

Write-Host "[4/4] filter comparison (EKF vs EKF tuned vs Madgwick)" -ForegroundColor Cyan
$args3 = @((Join-Path $here 'compare_filters.py'))
if ($Clean) { $args3 += '--clean' }
$r = Invoke-Native -Exe $py -Arguments $args3
Write-Host $r.Out.Trim()
exit $r.Code
