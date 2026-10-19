#---------------------------------------------------------------------------------------------------
# Syntax/type check for the flight-controller firmware using the real Keil AC6 compiler.
#
# Why this exists: there is no Keil IDE in this environment, but armclang IS installed, and EIDE
# already wrote a full compile_commands.json (build/FLY/). Replaying those exact command lines
# gives a true compiler check of every translation unit -- much stronger than "eyeballing the diff".
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1              # project sources only
#   powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1 -All         # + StdPeriph/CMSIS lib
#   powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1 -Filter release
#   powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1 -Link        # + real armlink pass
#
# WHY -Link EXISTS (2026-10-03, learned the hard way):
#   A compile-only check CANNOT see duplicate/undefined symbols. `HardFault_Handler` was defined both
#   in stm32f4xx_it.cpp and in RT-Thread's context_rvds.S -- every TU compiled fine, and the failure
#   only appeared at link time in the IDE ("L6200E: Symbol HardFault_Handler multiply defined").
#   `-Link` compiles ALL translation units (it implies -All: the StdPeriph driver objects are needed
#   to resolve undefined symbols) and then runs the real armlink with EIDE's scatter file.
#
# ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI).
#---------------------------------------------------------------------------------------------------
param(
    [switch] $All,
    [switch] $Link,
    [string] $Filter = ''
)

$ErrorActionPreference = 'Stop'

if ($Link) { $All = $true }   # 链接需要 StdPeriph 的目标文件来解析未定义符号

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$ccFile   = Join-Path $repoRoot 'build\FLY\compile_commands.json'
$outDir   = Join-Path $env:TEMP 'fc_build_check'

if (-not (Test-Path $ccFile)) {
    Write-Host "compile_commands.json not found: $ccFile" -ForegroundColor Red
    Write-Host "Build once from EIDE so it regenerates, or point -ccFile at another build directory." -ForegroundColor Red
    exit 2
}

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

# Wipe previous artifacts. WHY THIS IS NOT OPTIONAL:
#   The object file names are "<index>_<name>.o", so a filtered run (-Filter release) and a full run
#   produce DIFFERENT indexes for the same source -> both copies survive in this directory, and the
#   link step globs *.o. Result: "$Super$$main multiply defined" from two copies of release.o that
#   are both ours. That looks exactly like a real duplicate-symbol bug and costs an hour.
# Only *.o / *.d / *.axf are removed -- never the directory itself.
Get-ChildItem $outDir -Include *.o, *.d, *.axf -File -Recurse -ErrorAction SilentlyContinue |
    Remove-Item -Force -ErrorAction SilentlyContinue

$entries = Get-Content $ccFile -Raw | ConvertFrom-Json
Write-Host ("compile_commands entries: {0}" -f $entries.Count)

$recordedSources = @($entries | ForEach-Object {
    $sourcePath = $_.file
    if (-not [IO.Path]::IsPathRooted($sourcePath)) { $sourcePath = Join-Path $_.directory $sourcePath }
    [IO.Path]::GetFullPath($sourcePath)
})
$missingSources = @(Get-ChildItem (Join-Path $repoRoot 'app') -Recurse -Filter *.cpp -File |
    Where-Object { $_.FullName -notin $recordedSources })
if ($missingSources.Count -ne 0) {
    Write-Host 'Application sources missing from compile_commands.json:' -ForegroundColor Red
    $missingSources | ForEach-Object { Write-Host ("  {0}" -f $_.FullName) }
    Write-Host 'Add app to .eide/eide.yml srcDirs, then Rebuild in EIDE to regenerate the build inputs.' -ForegroundColor Red
    exit 2
}

$rootLower = $repoRoot.ToLower()
$targets = @()
foreach ($e in $entries) {
    $isProjectSource = $e.file.ToLower().StartsWith($rootLower)
    if (-not $All -and -not $isProjectSource) { continue }
    if ($Filter -and ($e.file -notmatch $Filter)) { continue }
    $targets += $e
}

if ($targets.Count -eq 0) {
    Write-Host "nothing to check (filter='$Filter')" -ForegroundColor Yellow
    exit 0
}

Write-Host ("checking {0} translation unit(s)..." -f $targets.Count)

$failed = @()
$i = 0
# WHY THIS IS NEEDED (2026-10-05):
#   Windows PowerShell 5.1 turns a native command's stderr into a terminating error when
#   $ErrorActionPreference = 'Stop' and the output is redirected with `2>&1`. The compiler writes
#   diagnostics (errors AND warnings) to stderr, so the FIRST translation unit that emits anything
#   made the script die inside the loop -- printing "NativeCommandError" instead of the compiler
#   output, i.e. a FAILING BUILD LOOKED LIKE A BROKEN SCRIPT and the real diagnostics were lost.
#   'Continue' keeps the capture working (stderr lines land in $output) and lets us report them below.
$ErrorActionPreference = 'Continue'
foreach ($e in $targets) {
    $i++
    $name = Split-Path $e.file -Leaf
    $obj  = Join-Path $outDir ("{0}_{1}.o" -f $i, [IO.Path]::GetFileNameWithoutExtension($name))

    # Take the recorded command line verbatim and only redirect the output object.
    # armclang uses "-o <obj>", armasm uses "-o <obj> --depend <dep>" -- redirect both, otherwise the
    # assembler tries to write into build/FLY/.obj (which is not writable from here).
    $cmd = $e.command
    $cmd = [regex]::Replace($cmd, '-o\s+("[^"]+"|\S+)', ('-o "' + $obj + '"'))
    $dep = [IO.Path]::ChangeExtension($obj, '.d')
    $cmd = [regex]::Replace($cmd, '--depend\s+("[^"]+"|\S+)', ('--depend "' + $dep + '"'))

    $output = cmd /c "cd /d `"$($e.directory)`" && $cmd" 2>&1
    $code = $LASTEXITCODE
    if ($code -ne 0) {
        $failed += [pscustomobject]@{ File = $e.file; Code = $code; Output = ($output -join "`n") }
        Write-Host ("FAIL  {0}" -f $name) -ForegroundColor Red
    } else {
        Write-Host ("ok    {0}" -f $name)
    }
}
$ErrorActionPreference = 'Stop'

Write-Host ""
if ($failed.Count -ne 0) {
    Write-Host ("BUILD CHECK FAIL ({0}/{1})" -f $failed.Count, $targets.Count) -ForegroundColor Red
    foreach ($f in $failed) {
        Write-Host ""
        Write-Host ("===== {0} =====" -f $f.File) -ForegroundColor Red
        Write-Host $f.Output
    }
    exit 1
}
Write-Host ("BUILD CHECK PASS ({0}/{1})" -f $targets.Count, $targets.Count) -ForegroundColor Green

if (-not $Link) { exit 0 }

# ------------------------------------------------------------------------------------------------
# Link check: replay EIDE's armlink with the objects we just produced.
#   Catches what compilation cannot: duplicate symbols (L6200E), undefined symbols (L6218E),
#   and "the image does not fit" (L6220E). See the header comment for why this exists.
# ------------------------------------------------------------------------------------------------
$ccExe   = ($entries[0].command -split '\s+')[0].Trim('"')
$armlink = Join-Path (Split-Path $ccExe -Parent) 'armlink.exe'
if (-not (Test-Path $armlink)) {
    Write-Host ("armlink not found next to the recorded compiler: {0}" -f $armlink) -ForegroundColor Red
    exit 2
}

$scatter = Join-Path $repoRoot 'build\FLY\dl_fly.sct'
if (-not (Test-Path $scatter)) {
    Write-Host ("scatter file not found: {0} (build once from EIDE)" -f $scatter) -ForegroundColor Red
    exit 2
}

$objs = Get-ChildItem $outDir -Filter *.o | ForEach-Object { $_.FullName }
Write-Host ("linking {0} object(s) with {1}" -f $objs.Count, (Split-Path $scatter -Leaf))
$axf  = Join-Path $outDir 'dl_fly_check.axf'
$ldOut = & $armlink --cpu=Cortex-M4.fp.sp "--scatter=$scatter" --diag_suppress=L6329 -o $axf @objs 2>&1
$ldCode = $LASTEXITCODE
if ($ldCode -ne 0) {
    Write-Host "LINK CHECK FAIL" -ForegroundColor Red
    Write-Host ($ldOut -join "`n")
    exit 1
}
Write-Host "LINK CHECK PASS" -ForegroundColor Green
exit 0
