#===================================================================================================
# tools/link_v2/run_verify.ps1 —— 链路 v2 一键回归（验证者）
#
# 跑完五块，任何失败非零退出：
#   1) 独立参考实现自检（规格 §10 黄金向量 vs 独立 oracle）
#   2) 常量头一致性（tools/check_link_config.py）
#   3) 轮次离散事件仿真：确定性自检 + N 个随机场景（默认 10^5）
#   4) 帧层两端独立对拍（独立 oracle + 两端真实编解码器：编码/解码回环/坏流重同步）
#   5) 两端轮次实现口径断言（RC 命令门控/600ms 超时、FC HELLO 计数不推进遥测）
#   最后生成 test/link_v2/sim/REPORT.md
#
# 用法（仓库根目录）:
#   pwsh tools/link_v2/run_verify.ps1
#   pwsh tools/link_v2/run_verify.ps1 -Scenarios 1000          # 快速自检
#   pwsh tools/link_v2/run_verify.ps1 -RcRoot "D:\controller"  # 遥控器仓库路径不同
#   pwsh tools/link_v2/run_verify.ps1 -SkipBuild               # 复用已编译探针
#===================================================================================================
param(
    [int]$Scenarios = 100000,
    [string]$RcRoot = "E:\ESP_PROJECT\controller",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
Set-Location $Root
$env:PYTHONIOENCODING = "utf-8"
$OutDir = Join-Path $Root "test\link_v2\sim\out"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$steps = New-Object System.Collections.ArrayList
function Run-Step {
    param([string]$Name, [string]$Exe, [string[]]$CmdArgs, [string]$LogName)
    Write-Host ("`n=== {0} ===" -f $Name) -ForegroundColor Cyan
    Write-Host ("$Exe " + ($CmdArgs -join " "))
    $sw = [Diagnostics.Stopwatch]::StartNew()
    & $Exe @CmdArgs 2>&1 | Tee-Object -FilePath (Join-Path $OutDir $LogName) | Out-Null
    $code = $LASTEXITCODE
    $sw.Stop()
    [void]$steps.Add([ordered]@{
        name = $Name; cmd = "$Exe " + ($CmdArgs -join " "); code = $code;
        seconds = [math]::Round($sw.Elapsed.TotalSeconds, 1); log = $LogName
    })
    if ($code -ne 0) { Write-Host ("FAIL: {0} exit={1}" -f $Name, $code) -ForegroundColor Red }
    return $code
}

try {
    # 1) 独立 oracle 自检
    $c1 = Run-Step "独立参考实现 vs 规格 §10 黄金向量" "python" @("test/link_v2/sim/ref_frame.py", "selfcheck") "selftest_ref.txt"

    # 2) 常量头一致性
    $c2 = Run-Step "两端常量头逐名逐值一致" "python" @("tools/check_link_config.py") "check_link_config.txt"

    # 3) 轮次仿真：确定性自检 + 随机场景
    $c3 = Run-Step "轮次仿真确定性自检" "python" @("test/link_v2/sim/link_sim.py", "--selftest") "selftest.txt"
    $simArgs = @("test/link_v2/sim/link_sim.py", "--n", "$Scenarios",
                 "--json", "test/link_v2/sim/out/sim_100k.json")
    $c4 = Run-Step "轮次仿真 $Scenarios 随机场景" "python" $simArgs "sim_run.txt"

    # 4) 帧层两端对拍（探针在步骤内编译）
    $xcArgs = @("test/link_v2/sim/xcheck.py", "--rc-root", $RcRoot,
                "--json", "test/link_v2/sim/out/xcheck.json")
    if ($SkipBuild) { $xcArgs += "--skip-build" }
    $c5 = Run-Step "帧层两端独立对拍" "python" $xcArgs "xcheck.txt"

    # 5) 两端轮次实现口径断言（Lead 拍板的偏差项回归）
    $c6 = Run-Step "两端轮次实现口径断言" "python" @("test/link_v2/sim/host_checks.py",
                                                    "--rc-root", $RcRoot) "host_checks.txt"

    $allPass = ($c1 -eq 0 -and $c2 -eq 0 -and $c3 -eq 0 -and $c4 -eq 0 -and $c5 -eq 0 -and $c6 -eq 0)
} catch {
    Write-Host ("异常: " + $_.Exception.Message) -ForegroundColor Red
    $allPass = $false
}

# 步骤汇总给报告生成器
$summary = [ordered]@{ all_pass = $allPass; steps = $steps }
$summary | ConvertTo-Json -Depth 5 | Set-Content -Path (Join-Path $OutDir "steps.json") -Encoding utf8

# 5) 生成 REPORT.md
python test/link_v2/sim/make_report.py
$reportCode = $LASTEXITCODE
if ($reportCode -ne 0) { $allPass = $false }

Write-Host ""
if ($allPass) {
    Write-Host "==== 链路 v2 回归：全部通过 ====" -ForegroundColor Green
    exit 0
} else {
    Write-Host "==== 链路 v2 回归：存在失败（见上） ====" -ForegroundColor Red
    exit 1
}



