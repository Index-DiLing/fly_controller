#---------------------------------------------------------------------------------------------------
# Bench firmware build (flight controller side): link bench by default, any bench main via -Main.
#
# What it does
#   1. Reads build/FLY/compile_commands.json (written by EIDE) and replays **exactly** the armclang
#      command lines EIDE used, so the bench objects are compiled with the same flags/includes as the
#      flight firmware. Nothing here invents a compiler setup.
#   2. Compiles the bench main (-Main) with release.cpp's flag set (C++11, same -I/-D).
#   3. Links every object EXCEPT release.cpp (that one owns the flight `main`), using the existing
#      build/FLY/dl_fly.sct scatter file, so the bench image has the same memory layout as the flight
#      image.
#   4. Emits axf/hex/bin to the output dir.
#
# Why release.cpp is dropped instead of patched
#   The bench must be a different `main`. Patching release.cpp in place would touch the mainline
#   source; dropping it from the link is the only change, and it is a build-script-only change.
#   release.cpp defines no globals (logContext lives in app/runtime/release_header.cpp), so the link
#   stays complete.
#
# Naming (parameterised 2026-10-18; the default call behaves exactly as before)
#   -Main <repo-relative or absolute path>  bench main source, default test\bench\link_bench_main.cpp
#   -Name <short>                           short name; drives the bench object leaf, the image leaf
#                                           names and the default/fallback output folders:
#       link (default) -> dl_fly_bench.axf/.hex/.bin       build\bench       test\bench\_out
#       baro           -> dl_fly_baro_bench.axf/.hex/.bin  build\bench_baro  test\bench\_out_baro
#   For the baro bench use tools\bench\build_baro_bench.ps1 (thin wrapper around this script).
#
# Usage
#   powershell -ExecutionPolicy Bypass -File .\tools\bench\build_fc_bench.ps1
#   powershell -ExecutionPolicy Bypass -File .\tools\bench\build_fc_bench.ps1 -Rebuild   # force
#   powershell -ExecutionPolicy Bypass -File .\tools\bench\build_fc_bench.ps1 -Main test\bench\baro_bench_main.cpp -Name baro
#
# Outputs (default link build)
#   build/bench/dl_fly_bench.axf / .hex / .bin
#   build/bench/bench_build.log   (full compiler + linker output)
#
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
param(
    [switch] $Rebuild,          # recompile everything even if objects look current
    [string] $OutDir = '',      # override output dir (default: <repo>\build\bench[_<Name>])
    [string] $Main = 'test\bench\link_bench_main.cpp',  # bench main source (repo-relative or absolute)
    [string] $Name = 'link'     # short name: object leaf, image leaf names, default/fallback folders
)

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$ccFile   = Join-Path $repoRoot 'build\FLY\compile_commands.json'
$scatter  = Join-Path $repoRoot 'build\FLY\dl_fly.sct'

# --- parameter sanity: fail loudly instead of building into the wrong place ------------------------
# Callers must pass -Main/-Name/-OutDir BY NAME. Windows PowerShell 5.1 splats an ARRAY positionally
# (only a hashtable splats by name), so a wrapper doing @('-Main', <src>, '-Name', 'baro') silently
# bound $OutDir='-Main' / $Name='-Name' and wrote all 77 objects into a repo-root "-Main" folder.
# These checks turn that class of mistake into an immediate, explicit exit 2.
if ($Name -notmatch '^[A-Za-z0-9_]+$') {
    Write-Host ("invalid -Name '{0}': expected [A-Za-z0-9_]+ - pass parameters by name, not via array splat" -f $Name) -ForegroundColor Red
    exit 2
}
if ($Main -notmatch '\.cpp$') {
    Write-Host ("invalid -Main '{0}': expected a .cpp path" -f $Main) -ForegroundColor Red
    exit 2
}
if (-not [string]::IsNullOrEmpty($OutDir)) {
    if ($OutDir.StartsWith('-')) {
        Write-Host ("invalid -OutDir '{0}': looks like a stray option - pass parameters by name, not via array splat" -f $OutDir) -ForegroundColor Red
        exit 2
    }
    if (-not [IO.Path]::IsPathRooted($OutDir)) {
        $OutDir = Join-Path $repoRoot $OutDir      # relative paths resolve against the repo root
    }
}

# --- name derivation ------------------------------------------------------------------------------
# -Main is documented repo-relative; an absolute path is accepted too (EIDE always writes absolute
# paths, so being lenient here costs nothing).
if ([IO.Path]::IsPathRooted($Main)) {
    $benchSrc = $Main
} else {
    $benchSrc = Join-Path $repoRoot $Main
}
$benchLeaf = Split-Path $benchSrc -Leaf

# The bench object keeps the old leaf spelling for -Name link (zzz_link_bench_main.o), so the default
# invocation stays byte-identical; -Name only swaps the middle word.
$benchObjName = 'zzz_{0}_bench_main.o' -f $Name

if ($Name -ieq 'link') {
    $product   = 'dl_fly_bench'
    $preferred = Join-Path $repoRoot 'build\bench'
    $fallback  = Join-Path $repoRoot 'test\bench\_out'
} else {
    $product   = 'dl_fly_{0}_bench' -f $Name
    $preferred = Join-Path $repoRoot ('build\bench_{0}' -f $Name)
    $fallback  = Join-Path $repoRoot ('test\bench\_out_{0}' -f $Name)
}

# WHERE THE IMAGE GOES
#   Default build\bench is what the task asks for, but this workspace can be opened by a DSH sandbox
#   whose low-integrity token may not be allowed to write under build\ (EIDE created that tree
#   unconfined, so it has no low-integrity label). Rather than fail with a bare "access denied",
#   probe first and fall back to test\bench\_out, printing the exact command to re-run with full
#   access. The image is identical either way; only the folder differs.
if ([string]::IsNullOrEmpty($OutDir)) {
    $probeOk   = $false
    try {
        New-Item -ItemType Directory -Force -Path $preferred | Out-Null
        $probe = Join-Path $preferred '.write_probe'
        [IO.File]::WriteAllText($probe, 'x')
        Remove-Item -Force $probe
        $probeOk = $true
    } catch { $probeOk = $false }
    if (-not $probeOk) {
        Write-Host ("[warn] {0} is not writable from this session (sandbox integrity label)." -f $preferred) -ForegroundColor Yellow
        Write-Host ("[warn] falling back to {0}" -f $fallback) -ForegroundColor Yellow
        # Re-print the exact call that reproduces this image with full access (defaults left out).
        $hint = '.\tools\bench\build_fc_bench.ps1'
        if ($Main -ne 'test\bench\link_bench_main.cpp') { $hint += (' -Main "{0}"' -f $Main) }
        if ($Name -ne 'link')                           { $hint += (' -Name {0}' -f $Name) }
        $hint += (' -OutDir "{0}"' -f $preferred)
        Write-Host ("[warn] with full access run:  {0}" -f $hint) -ForegroundColor Yellow
        $OutDir = $fallback
    } else {
        $OutDir = $preferred
    }
}
$outDir   = $OutDir
$objDir   = Join-Path $outDir 'obj'
$logFile  = Join-Path $outDir 'bench_build.log'

if (-not (Test-Path $ccFile)) {
    Write-Host ("compile_commands.json not found: {0}" -f $ccFile) -ForegroundColor Red
    Write-Host "Build once from EIDE so it regenerates." -ForegroundColor Red
    exit 2
}
if (-not (Test-Path $scatter)) {
    Write-Host ("scatter file not found: {0}" -f $scatter) -ForegroundColor Red
    exit 2
}
if (-not (Test-Path $benchSrc)) {
    Write-Host ("bench main not found: {0}" -f $benchSrc) -ForegroundColor Red
    exit 2
}

New-Item -ItemType Directory -Force -Path $outDir, $objDir | Out-Null
if ($Rebuild) {
    Get-ChildItem $objDir -Include *.o, *.d -File -Recurse -ErrorAction SilentlyContinue | Remove-Item -Force
}

$entries = Get-Content $ccFile -Raw | ConvertFrom-Json
Write-Host ("compile_commands entries: {0}" -f $entries.Count)

# --- toolchain: take it from the recorded command line, never guess -------------------------------
$ccExe   = ($entries[0].command -split '\s+')[0].Trim('"')
$toolDir = Split-Path $ccExe -Parent
$armlink = Join-Path $toolDir 'armlink.exe'
$fromelf = Join-Path $toolDir 'fromelf.exe'
if (-not (Test-Path $armlink)) { Write-Host ("armlink not found: {0}" -f $armlink) -ForegroundColor Red; exit 2 }
if (-not (Test-Path $fromelf)) { Write-Host ("fromelf not found: {0}" -f $fromelf) -ForegroundColor Red; exit 2 }
Write-Host ("armclang: {0}" -f $ccExe)

$log = New-Object System.Collections.Generic.List[string]
function Log([string]$s) { $log.Add($s) | Out-Null }

$failed = @()
$i = 0

# Redirect -o / --depend to our object dir; keep every other token verbatim.
function Rewrite-Command([string]$cmd, [string]$obj) {
    $c = [regex]::Replace($cmd, '-o\s+("[^"]+"|\S+)', ('-o "' + $obj + '"'))
    $dep = [IO.Path]::ChangeExtension($obj, '.d')
    $c = [regex]::Replace($c, '--depend\s+("[^"]+"|\S+)', ('--depend "' + $dep + '"'))
    return $c
}

$ErrorActionPreference = 'Continue'
$objs = New-Object System.Collections.Generic.List[string]

foreach ($e in $entries) {
    $leaf = Split-Path $e.file -Leaf
    if ($leaf -ieq 'release.cpp') {
        Write-Host ("skip  {0}  (owns main; bench provides its own)" -f $leaf) -ForegroundColor Yellow
        continue
    }
    if ($leaf -ieq $benchLeaf) {
        # The bench main must not also be compiled here: it is built below with its own object leaf,
        # and keeping both copies would put two `main`s in the link. No-op for the default -Main,
        # because test\bench\* is not part of the mainline srcDirs.
        Write-Host ("skip  {0}  (bench main; compiled separately)" -f $leaf) -ForegroundColor Yellow
        continue
    }
    $i++
    $obj = Join-Path $objDir ("{0:d3}_{1}.o" -f $i, [IO.Path]::GetFileNameWithoutExtension($leaf))
    $cmd = Rewrite-Command $e.command $obj
    $out = cmd /c "cd /d `"$($e.directory)`" && $cmd" 2>&1
    $code = $LASTEXITCODE
    Log ("--- [{0}] {1}" -f $code, $e.file)
    Log ($out -join "`n")
    if ($code -ne 0) {
        $failed += [pscustomobject]@{ File = $e.file; Output = ($out -join "`n") }
        Write-Host ("FAIL  {0}" -f $leaf) -ForegroundColor Red
    } else {
        $objs.Add($obj) | Out-Null
        if (($i % 10) -eq 0) { Write-Host ("ok    {0} ({1} objects)" -f $leaf, $objs.Count) }
    }
}

# --- bench main: same flags as release.cpp, source substituted -----------------------------------
Write-Host "compiling bench main..."
$rel = $entries | Where-Object { (Split-Path $_.file -Leaf) -ieq 'release.cpp' } | Select-Object -First 1
if ($null -eq $rel) {
    Write-Host "release.cpp entry missing from compile_commands.json (needed as the flag template)" -ForegroundColor Red
    exit 2
}
# Everything before " -o " is: compiler + --target + -c + -std + includes + defines + tuning flags.
$head = ($rel.command -split '\s+-o\s+', 2)[0]
$benchObj = Join-Path $objDir $benchObjName
$benchCmd = $head + ' -o "' + $benchObj + '" -MMD "' + $benchSrc + '"'
$out = cmd /c "cd /d `"$repoRoot`" && $benchCmd" 2>&1
$code = $LASTEXITCODE
Log ("--- [{0}] {1}" -f $code, $benchSrc)
Log ($out -join "`n")
if ($code -ne 0) {
    $failed += [pscustomobject]@{ File = $benchSrc; Output = ($out -join "`n") }
    Write-Host ("FAIL  {0}" -f $benchLeaf) -ForegroundColor Red
} else {
    $objs.Add($benchObj) | Out-Null
    Write-Host ("ok    {0}" -f $benchLeaf)
}
$ErrorActionPreference = 'Stop'

if ($failed.Count -ne 0) {
    Write-Host ""
    Write-Host ("BENCH BUILD FAIL ({0} TU failed)" -f $failed.Count) -ForegroundColor Red
    foreach ($f in $failed) {
        Write-Host ""
        Write-Host ("===== {0} =====" -f $f.File) -ForegroundColor Red
        Write-Host $f.Output
    }
    Set-Content -Path $logFile -Value $log -Encoding UTF8
    Write-Host ("log: {0}" -f $logFile)
    exit 1
}

# --- link ----------------------------------------------------------------------------------------
$axf = Join-Path $outDir ("{0}.axf" -f $product)
$map = Join-Path $outDir ("{0}.map" -f $product)
Write-Host ("linking {0} objects with {1} ..." -f $objs.Count, (Split-Path $scatter -Leaf))
$ldOut = & $armlink --cpu=Cortex-M4.fp.sp "--scatter=$scatter" --diag_suppress=L6329 --strict `
    --info=sizes --info=totals --map "--list=$map" -o $axf @objs 2>&1
$code = $LASTEXITCODE
Log ("--- [link {0}] armlink" -f $code)
Log ($ldOut -join "`n")
if ($code -ne 0) {
    Write-Host "LINK FAIL" -ForegroundColor Red
    Write-Host ($ldOut -join "`n")
    Set-Content -Path $logFile -Value $log -Encoding UTF8
    exit 1
}
Write-Host "LINK OK" -ForegroundColor Green

# --- hex / bin -----------------------------------------------------------------------------------
$hex = Join-Path $outDir ("{0}.hex" -f $product)
$bin = Join-Path $outDir ("{0}.bin" -f $product)
& $fromelf --i32combined -o $hex $axf 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Host "fromelf --i32combined failed" -ForegroundColor Red; exit 1 }
& $fromelf --bin -o $bin $axf 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Host "fromelf --bin failed" -ForegroundColor Red; exit 1 }

Set-Content -Path $logFile -Value $log -Encoding UTF8

Write-Host ""
Write-Host "=== bench image ===" -ForegroundColor Green
foreach ($p in @($axf, $hex, $bin)) {
    $fi = Get-Item $p
    Write-Host ("  {0}  ({1} bytes)" -f $fi.FullName, $fi.Length)
}
Write-Host ("log: {0}" -f $logFile)
Write-Host ""
Write-Host "Flash with your usual tool (e.g. ST-LINK Utility / pyocd)." -ForegroundColor Cyan
exit 0
