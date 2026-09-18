#---------------------------------------------------------------------------------------------------
# Run the whole scenario suite for several RATE_*_I gains, all processes in parallel.
#
# Every process runs with DLX_KEEP_CG=1 so nobody restores Mass.xml behind the others back;
# the CG offset is set once here and restored at the very end.
#
#   powershell -ExecutionPolicy Bypass -File .\sim\sweep_rate_i.ps1
#   powershell -ExecutionPolicy Bypass -File .\sim\sweep_rate_i.ps1 -I 3,4,5,6 -Cg 5
#
# ASCII-only on purpose.
#---------------------------------------------------------------------------------------------------
param(
    [string]$I = '3,4,5,6',
    [double]$Cg = 5.0,
    [switch]$NoBaseline
)

$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$mass = 'E:\JSBSim\aircraft\DLX450\Mass.xml'
$massOrig = "$mass.orig"

function Need-Python {
    if ($env:DLX_PYTHON -and (Test-Path $env:DLX_PYTHON)) { return $env:DLX_PYTHON }
    $c = Get-Command python -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    foreach ($p in @('E:\Python3.10\python.exe', 'C:\Python310\python.exe')) {
        if (Test-Path $p) { return $p }
    }
    throw 'python not found (set DLX_PYTHON)'
}
$py  = Need-Python
$sim = Join-Path $here 'jsbsim_drone_sim.py'
$out = Join-Path $here 'out'

# main_att_ekf style accelerometer constraints + persistent-quiet detection
$FCFG = @('--tilt-sigma','0.08','--tilt-gate','0.20','--tilt-inno-limit','0.15','--freeze-bias','1','--accel-hold','1')

# 1) one pristine copy, then preset the CG offset once for all runs
if (-not (Test-Path $massOrig)) { Copy-Item -LiteralPath $mass -Destination $massOrig }
$xml = Get-Content -LiteralPath $massOrig -Raw
$pat = '(<location name="CG"[^>]*>\s*<x>)[^<]*(</x>)'
$rep = '${1}' + ((-$Cg / 1000.0).ToString('0.000000')) + '${2}'
$xml = [regex]::Replace($xml, $pat, $rep, 1)
Set-Content -LiteralPath $mass -Value $xml -NoNewline -Encoding UTF8
Write-Host ("CG preset to {0:+#;-#;0} mm (positive = nose heavy)" -f $Cg) -ForegroundColor Cyan

# 2) launch all runs in parallel
$env:DLX_KEEP_CG = '1'
$jobs = @()
try {
    if (-not $NoBaseline) {
        $jobs += Start-Process -FilePath $py -PassThru -NoNewWindow -ArgumentList @('-X','utf8',$sim,'--tag','_BASE') -RedirectStandardOutput (Join-Path $out 'sweep_base.log') -RedirectStandardError (Join-Path $out 'sweep_base.err')
    }
    $ivals = @($I.Split(',') | ForEach-Object { [double]$_.Trim() })
    foreach ($iv in $ivals) {
        $tag = '_F' + ($iv.ToString().Replace('.','p'))
        $jobs += Start-Process -FilePath $py -PassThru -NoNewWindow -ArgumentList (@('-X','utf8',$sim,'--tag',$tag) + $FCFG + @('--rate-i-scale', $iv)) -RedirectStandardOutput (Join-Path $out ('sweep' + $tag + '.log')) -RedirectStandardError (Join-Path $out ('sweep' + $tag + '.err'))
    }
    $jobs | Wait-Process
    Write-Host "all $($jobs.Count) runs finished" -ForegroundColor Cyan
} finally {
    $env:DLX_KEEP_CG = $null
    if (Test-Path $massOrig) {
        Copy-Item -LiteralPath $massOrig -Destination $mass -Force
        Remove-Item -LiteralPath $massOrig -Force
        Write-Host 'Mass.xml restored' -ForegroundColor Cyan
    }
}
Write-Host 'each run summary is in out\sweep*.log' -ForegroundColor Cyan
