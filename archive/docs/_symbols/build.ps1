# build.ps1 —— 把 docs/_symbols/*.tsv(6 列 TSV) 生成 docs/app_symbol_table.html
# 只读源 TSV, 只写一个 HTML; 不改任何源码。
$ErrorActionPreference = 'Stop'
$dir = $PSScriptRoot
$out = Join-Path (Split-Path $dir -Parent) 'app_symbol_table.html'

# 文件展示顺序(按"业务层从内核往外"排)
$order = @(
  'app_state.h','app_state.cpp','app_task.hpp','app_guards.hpp',
  'app_flight_stack.hpp','app_sensors.hpp','app_health.hpp','app_health.cpp',
  'app_link_thread.hpp','app_link_cmd.hpp','app_flight_shared.hpp','app_log_channel.hpp',
  'app_log_thread.hpp','app_exception_handler.h','app_exception_handler.cpp',
  'app_boot_mode.hpp','app_boot_report.hpp','app_bringup.hpp',
  'release.cpp','release_header.h','release_header.cpp','release_log_manage.hpp',
  'app_stack.hpp','app_fault.hpp','hardware_claims.hpp','stm32f4xx_it.cpp'
)

$rows = New-Object System.Collections.Generic.List[object]
Get-ChildItem (Join-Path $dir '*.tsv') | Where-Object { $_.Name -ne '_tabtest.tsv' } | ForEach-Object {
  $src = $_.Name
  foreach ($line in (Get-Content -LiteralPath $_.FullName -Encoding UTF8)) {
    if ($line -match '^\s*#' -or $line.Trim() -eq '') { continue }
    $f = $line -split "`t"
    if ($f.Count -ne 6) { Write-Warning ("列数不是 6: {0} :: {1}" -f $src, $line); continue }
    $rows.Add([pscustomobject]@{ file=$f[0]; kind=$f[1]; symbol=$f[2]; desc=$f[3]; callers=$f[4]; note=$f[5]; src=$src })
  }
}

function Esc([string]$s) {
  if ($null -eq $s) { return '' }
  $s.Replace('&','&amp;').Replace('<','&lt;').Replace('>','&gt;')
}
function KindClass([string]$k) {
  switch ($k) {
    'member'        { 'k-mem' }
    'function'      { 'k-fn' }
    'method'        { 'k-fn' }
    'static-func'   { 'k-fn' }
    'static-method' { 'k-fn' }
    'ISR'           { 'k-isr' }
    'struct'        { 'k-type' }
    'enum'          { 'k-type' }
    'enumrator'     { 'k-type' }
    'enumerator'    { 'k-const' }
    'constexpr'     { 'k-const' }
    'static-const'  { 'k-const' }
    'global'        { 'k-glob' }
    'namespace'     { 'k-ns' }
    'using'         { 'k-ns' }
    'macro'         { 'k-mac' }
    'include'       { 'k-inc' }
    'static-assert' { 'k-as' }
    default         { 'k-mem' }
  }
}

$sb = New-Object System.Text.StringBuilder
[void]$sb.Append(@'
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DLX 飞控 v2 · app 业务层符号表</title>
<style>
:root{
  --bg:#f6f7f9;--panel:#fff;--panel2:#fbfcfd;--ink:#1b2130;--ink2:#3d4658;--muted:#6b7484;
  --line:#e3e6ec;--line2:#eef1f5;--acc:#2563eb;--bad:#dc2626;--warn:#b45309;--ok:#15803d;
  --mono:"Cascadia Mono",Consolas,"JetBrains Mono",Menlo,monospace;
  --sans:-apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei","PingFang SC",sans-serif;
}
html[data-theme="dark"]{
  --bg:#12161d;--panel:#1a1f28;--panel2:#1e242e;--ink:#e7ebf2;--ink2:#c3cbd8;--muted:#8d97a6;
  --line:#2b323d;--line2:#232932;--acc:#7aa2f7;--bad:#f87171;--warn:#fbbf24;--ok:#4ade80;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);font-size:13px;line-height:1.6}
header.top{position:sticky;top:0;z-index:60;background:var(--panel);border-bottom:1px solid var(--line);box-shadow:0 1px 3px rgba(0,0,0,.04)}
.top-in{max-width:1700px;margin:0 auto;padding:9px 18px;display:flex;gap:14px;align-items:center;flex-wrap:wrap}
.brand{font-weight:700;font-size:15px}
.brand span{color:var(--muted);font-weight:500;font-size:12.5px;margin-left:8px}
.tbtn{font:inherit;font-size:12px;padding:4px 10px;border:1px solid var(--line);background:var(--panel2);color:var(--ink2);border-radius:8px;cursor:pointer;margin-left:auto}
main{max-width:1700px;margin:0 auto;padding:16px 18px 80px}
.stat{display:flex;gap:18px;flex-wrap:wrap;font-size:12.5px;color:var(--muted);margin-bottom:10px}
.stat b{color:var(--ink);font-family:var(--mono)}
.ctl{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:10px 0 12px}
.ctl input[type=search]{flex:1;min-width:240px;padding:7px 11px;border:1px solid var(--line);border-radius:8px;background:var(--panel);color:var(--ink);font:inherit;font-size:12.5px}
.ctl select{padding:6px 9px;border:1px solid var(--line);border-radius:8px;background:var(--panel);color:var(--ink);font:inherit;font-size:12.5px}
.ctl label{font-size:12.5px;color:var(--ink2);display:flex;gap:5px;align-items:center;cursor:pointer;white-space:nowrap}
.ctl input[type=search]:focus,.ctl select:focus{outline:2px solid var(--acc);outline-offset:-1px}
.tw{overflow:auto;border:1px solid var(--line);border-radius:10px;background:var(--panel);max-height:none}
table{border-collapse:separate;border-spacing:0;width:100%;font-size:12.4px}
thead th{position:sticky;top:0;z-index:5;background:var(--panel2);text-align:left;font-weight:600;color:var(--ink2);padding:8px 10px;border-bottom:1px solid var(--line);white-space:nowrap;font-size:12px}
tbody td{padding:6px 10px;border-bottom:1px solid var(--line2);vertical-align:top}
tbody tr:hover{background:var(--panel2)}
tr.grp td{background:var(--line2);font-weight:700;color:var(--ink);padding:7px 10px;position:sticky;top:31px;z-index:4;border-top:1px solid var(--line);font-size:12.6px}
tr.grp .cnt{font-weight:500;color:var(--muted);font-family:var(--mono);margin-left:8px;font-size:11.5px}
.sym{font-family:var(--mono);font-size:12px;color:var(--ink);word-break:break-word}
.sym .sig{color:var(--muted)}
.desc{color:var(--ink2);min-width:320px}
.cal{font-family:var(--mono);font-size:11.4px;color:var(--ink2);word-break:break-word}
.note{color:var(--muted);font-size:11.8px}
.kind{display:inline-block;padding:1px 7px;border-radius:999px;font-size:10.5px;font-weight:600;white-space:nowrap}
.k-mem{background:#eef2f7;color:#475569}
.k-fn {background:#e8f0fe;color:#1d4ed8}
.k-type{background:#f1ebfe;color:#6d28d9}
.k-const{background:#e6f6f0;color:#047857}
.k-glob{background:#fdeee5;color:#c2410c}
.k-ns{background:#eef1f5;color:#64748b}
.k-mac{background:#fdf4e3;color:#a16207}
.k-inc{background:#f1f5f9;color:#94a3b8}
.k-as {background:#fdeaea;color:#b91c1c}
.k-isr{background:#fdeaea;color:#b91c1c}
html[data-theme="dark"] .kind{background:#252c37;color:#cfd7e3}
html[data-theme="dark"] .k-fn{color:#93b4ff}
html[data-theme="dark"] .k-type{color:#c4b5fd}
html[data-theme="dark"] .k-const{color:#6ee7b7}
html[data-theme="dark"] .k-glob{color:#fdba74}
td.cal-none{color:var(--bad);font-weight:600}
td.cal-star,.desc-star{color:var(--warn)}
tr.hidden{display:none}
.foot{margin-top:14px;font-size:12px;color:var(--muted)}
</style>
</head>
<body>
<header class="top">
  <div class="top-in">
    <div class="brand">app 业务层符号表<span>符号 / 解释 / 调用者 · 只读梳理, 未修改任何代码</span></div>
    <button class="tbtn" id="themeBtn">◐ 深/浅</button>
  </div>
</header>
<main>
  <div class="stat" id="stats"></div>
  <div class="ctl">
    <input type="search" id="q" placeholder="搜索符号 / 解释 / 调用者 / 备注…（例如 health、dshot、snapshot、无调用方、★）">
    <select id="ffile"><option value="">全部文件</option></select>
    <select id="fkind"><option value="">全部类别</option></select>
    <label><input type="checkbox" id="fdead"> 只看 无调用方 / ★</label>
    <label><input type="checkbox" id="fstar"> 只看 ISR / 全局量</label>
  </div>
  <div class="tw">
    <table>
      <thead><tr><th>文件</th><th>类别</th><th>符号</th><th>解释</th><th>调用者</th><th>备注</th></tr></thead>
      <tbody id="tb">
'@)

# ---- 分组输出 ----
$filesPresent = $rows | Group-Object file
$ordered = New-Object System.Collections.Generic.List[string]
foreach ($f in $order) { if ($filesPresent.Name -contains $f) { $ordered.Add($f) } }
foreach ($f in $filesPresent.Name) { if (-not $ordered.Contains($f)) { $ordered.Add($f) } }

$total = 0
foreach ($fname in $ordered) {
  $grp = $rows | Where-Object { $_.file -eq $fname }
  $total += $grp.Count
  [void]$sb.Append("      <tr class=""grp"" data-grp=""1"" data-file=""" + (Esc $fname) + """><td colspan=""6"">" + (Esc $fname) + "<span class=""cnt"">" + $grp.Count + " 个符号</span></td></tr>`n")
  foreach ($r in $grp) {
    $calCls = ''
    if ($r.callers -match '无调用方|无读者|无写者|无生产者') { $calCls = ' class="cal-none"' } else { $calCls = '' }
    $star = ($r.desc + $r.callers + $r.note + $r.symbol) -match '★'
    $rowCls = if ($star) { ' class="star"' } else { '' }
    [void]$sb.Append("      <tr$rowCls data-file=""" + (Esc $r.file) + """ data-kind=""" + (Esc $r.kind) + """ data-star=""" + $(if($star){'1'}else{'0'}) + """ data-dead=""" + $(if($r.callers -match '无调用方|无读者|无写者'){'1'}else{'0'}) + """>")
    [void]$sb.Append("<td class=""nw"" style=""white-space:nowrap;color:var(--muted);font-size:11.5px"">" + (Esc $r.file) + "</td>")
    [void]$sb.Append("<td><span class=""kind " + (KindClass $r.kind) + """>" + (Esc $r.kind) + "</span></td>")
    [void]$sb.Append("<td class=""sym"">" + (Esc $r.symbol) + "</td>")
    [void]$sb.Append("<td class=""desc" + $(if($star){' desc-star'}else{''}) + """>" + (Esc $r.desc) + "</td>")
    [void]$sb.Append("<td$calCls class=""cal"">" + (Esc $r.callers) + "</td>")
    [void]$sb.Append("<td class=""note"">" + (Esc $r.note) + "</td>")
    [void]$sb.Append("</tr>`n")
  }
}

[void]$sb.Append(@'
      </tbody>
    </table>
  </div>
  <div class="foot" id="foot"></div>
</main>
<script>
(function(){
  var root=document.documentElement, btn=document.getElementById('themeBtn');
  try{ var t=localStorage.getItem('dlx-theme'); if(t) root.setAttribute('data-theme',t); }catch(e){}
  btn.addEventListener('click',function(){
    var n=root.getAttribute('data-theme')==='dark'?'light':'dark';
    root.setAttribute('data-theme',n);
    try{ localStorage.setItem('dlx-theme',n); }catch(e){}
  });

  var tb=document.getElementById('tb');
  var rows=[].slice.call(tb.querySelectorAll('tr:not(.grp)'));
  var grps=[].slice.call(tb.querySelectorAll('tr.grp'));
  var q=document.getElementById('q'), ffile=document.getElementById('ffile'),
      fkind=document.getElementById('fkind'), fdead=document.getElementById('fdead'), fstar=document.getElementById('fstar');

  // 填充下拉
  var files=[], kinds={}, fileCount={};
  rows.forEach(function(r){
    var f=r.dataset.file, k=r.dataset.kind;
    if(files.indexOf(f)<0) files.push(f);
    kinds[k]=(kinds[k]||0)+1;
    fileCount[f]=(fileCount[f]||0)+1;
  });
  files.forEach(function(f){ var o=document.createElement('option'); o.value=f; o.textContent=f+' ('+fileCount[f]+')'; ffile.appendChild(o); });
  Object.keys(kinds).sort(function(a,b){return kinds[b]-kinds[a];}).forEach(function(k){
    var o=document.createElement('option'); o.value=k; o.textContent=k+' ('+kinds[k]+')'; fkind.appendChild(o);
  });

  document.getElementById('stats').innerHTML =
    '<span>符号总数 <b>'+rows.length+'</b></span>' +
    '<span>文件 <b>'+files.length+'</b></span>' +
    '<span>类别 <b>'+Object.keys(kinds).length+'</b></span>' +
    '<span>无调用方/无读者 <b>'+rows.filter(function(r){return r.dataset.dead==='1';}).length+'</b></span>' +
    '<span>带 ★ 重点 <b>'+rows.filter(function(r){return r.dataset.star==='1';}).length+'</b></span>';

  function apply(){
    var s=(q.value||'').trim().toLowerCase(), f=ffile.value, k=fkind.value,
        dead=fdead.checked, star=fstar.checked, vis=0;
    rows.forEach(function(r){
      var ok = true;
      if(f && r.dataset.file!==f) ok=false;
      if(k && r.dataset.kind!==k) ok=false;
      if(dead && r.dataset.dead!=='1' && r.dataset.star!=='1') ok=false;
      if(star && !(r.dataset.kind==='ISR'||r.dataset.kind==='global')) ok=false;
      if(ok && s) ok = r.textContent.toLowerCase().indexOf(s)>=0;
      r.classList.toggle('hidden', !ok);
      if(ok) vis++;
    });
    // 分组行: 该文件下没有任何可见数据行就藏起来
    grps.forEach(function(g){
      var f2=g.dataset.file, any=false, n=g.nextElementSibling;
      while(n && !n.classList.contains('grp')){ if(!n.classList.contains('hidden')){ any=true; break; } n=n.nextElementSibling; }
      g.classList.toggle('hidden', !any);
    });
    document.getElementById('foot').textContent='当前显示 '+vis+' / '+rows.length+' 个符号';
  }
  [q,ffile,fkind,fdead,fstar].forEach(function(el){ el.addEventListener('input',apply); el.addEventListener('change',apply); });
  apply();
})();
</script>
</body>
</html>
'@)

Set-Content -LiteralPath $out -Value $sb.ToString() -Encoding utf8NoBOM
Write-Host ("写入 {0}" -f $out)
Write-Host ("符号行 {0} 个; 文件 {1} 个" -f $total, $ordered.Count)
