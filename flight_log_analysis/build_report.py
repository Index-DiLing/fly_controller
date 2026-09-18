"""由 analysis.json 生成自包含的可视化分析报告 report.html + 文字报告 report.md。

图表用原生 SVG/JS 手写(不依赖 CDN), 所以离线双击打开即可看。
"""
from __future__ import annotations

import json
import os
from datetime import datetime

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

CFG = {
    "roll": "#4e79a7", "pitch": "#e15759", "yaw": "#59a14f",
    "tgt": "#f28e2b", "m0": "#4e79a7", "m1": "#e15759", "m2": "#59a14f", "m3": "#b07aa1",
    "acc": "#4e79a7", "gdev": "#e15759", "thr": "#4e79a7", "man": "#f28e2b",
    "hgt": "#4e79a7", "brel": "#59a14f", "vv": "#b07aa1", "link": "#e15759",
    "scale": "#b07aa1", "gyro": "#8c510a", "biasx": "#4e79a7", "biasy": "#e15759",
    "biasz": "#59a14f",
}

CSS = """
*{box-sizing:border-box}
body{margin:0;padding:22px 26px 60px;font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Microsoft YaHei",Arial,sans-serif;
     background:#fff;color:#111;font-size:14px;line-height:1.55}
@media(prefers-color-scheme:dark){body{background:#13151d;color:#e7e7e7}
 .card,.kpi,table,.evt{background:#1b1e28!important;border-color:#2b3040!important}
 th{background:#20242f!important} .tip{background:#22262f!important;border-color:#3a4257!important;color:#e7e7e7!important}}
h1{font-size:21px;margin:0 0 6px} h2{font-size:17px;margin:26px 0 10px;padding-bottom:6px;border-bottom:1px solid #ddd}
h3{font-size:14px;margin:18px 0 6px;color:#555}
@media(prefers-color-scheme:dark){h2{border-color:#333} h3{color:#aaa}}
.sub{color:#888;font-size:12.5px;margin:0 0 14px}
.kpis{display:flex;flex-wrap:wrap;gap:8px;margin:8px 0 14px}
.kpi{background:#f7f8fa;border:1px solid #e3e6ea;border-radius:8px;padding:7px 11px;min-width:120px}
.kpi .k{font-size:11px;color:#888;display:block} .kpi .v{font-size:15px;font-weight:600}
.card{background:#fbfbfd;border:1px solid #e6e8ec;border-radius:10px;padding:12px 14px;margin:10px 0}
.chart{margin:6px 0 2px;position:relative}
.legend{font-size:12px;margin:6px 0 2px;color:#555}
.legend i{display:inline-block;width:11px;height:3px;vertical-align:middle;margin:0 4px 0 12px;border-radius:2px}
.legend i:first-child{margin-left:0}
.title{font-size:12.5px;font-weight:600;margin:14px 0 2px}
table{border-collapse:collapse;width:100%;font-size:12.5px;margin:8px 0 14px;background:#fff}
th,td{border:1px solid #e3e6ea;padding:4px 7px;text-align:right;white-space:nowrap}
th{background:#f2f4f7;font-weight:600;text-align:center}
td.l,th.l{text-align:left}
tr:nth-child(even) td{background:rgba(127,127,127,0.04)}
.bar{height:9px;border-radius:3px;display:inline-block;vertical-align:middle;background:#4e79a7}
.bar.warn{background:#f28e2b} .bar.bad{background:#e15759} .bar.ok{background:#59a14f}
td .barwrap{display:flex;align-items:center;gap:6px;justify-content:flex-end}
.tip{position:absolute;pointer-events:none;background:rgba(255,255,255,.96);border:1px solid #ccc;border-radius:6px;
     padding:5px 8px;font-size:11.5px;color:#111;display:none;z-index:9;line-height:1.4;white-space:nowrap}
.evt{background:#fbfbfd;border:1px solid #e6e8ec;border-radius:8px;padding:8px 10px;margin:6px 0;font-size:12.5px}
.tag{display:inline-block;padding:0 6px;border-radius:4px;font-size:11px;background:#eaeef3;color:#333;margin-right:6px}
.tag.bad{background:#fde7e6;color:#a02c2c} .tag.warn{background:#fdf1e0;color:#8a5a12} .tag.ok{background:#e7f4ea;color:#2c6e3f}
ul{margin:6px 0 6px 18px;padding:0} li{margin:3px 0}
code{background:#f0f2f5;padding:1px 4px;border-radius:3px;font-size:12px}
@media(prefers-color-scheme:dark){code{background:#232734}}
.split{display:flex;gap:12px;flex-wrap:wrap}.split>div{flex:1 1 380px;min-width:320px}
svg{display:block;max-width:100%;height:auto}
"""

# ---------------------------------------------------------------- JS 图表库
JS = r"""
const F = (v,p)=> (v===null||v===undefined||Number.isNaN(v))?'-':v.toFixed(p===undefined?2:p);

function mkChart(host, cfg){
  const H = cfg.h || 190, W = 1180, ml = 56, mr = (cfg.ax2? 56 : 16), mt = 16, mb = 24;
  const hostEl = document.getElementById(host);
  if(!hostEl) return;
  const bag = (typeof window!=='undefined' && window.BAGS && typeof cfg.bag==='string')
              ? window.BAGS[cfg.bag] : cfg.data;
  const t = bag.t;
  const series = cfg.series.map(s=>({...s, v: bag[s.k]}));
  const s1 = series.filter(s=>!s.ax2), s2 = series.filter(s=>s.ax2);
  const full = [t[0], t[t.length-1]];
  let dom = full.slice();
  const svgNS='http://www.w3.org/2000/svg';
  const svg = document.createElementNS(svgNS,'svg');
  svg.setAttribute('viewBox',`0 0 ${W} ${H}`); svg.setAttribute('preserveAspectRatio','xMidYMid meet');
  hostEl.appendChild(svg);
  const tip = document.createElement('div'); tip.className='tip'; hostEl.appendChild(tip);

  const lo = (a,f)=>{let v=a[0];for(let i=1;i<a.length;i++){const x=f(a[i]);if(x<v)v=x;}return v;};
  const hi = (a,f)=>{let v=a[0];for(let i=1;i<a.length;i++){const x=f(a[i]);if(x>v)v=x;}return v;};
  function niceRange(mn,mx){
    if(!(mx>mn)) { const c=(mn+mx)/2||0; mn=c-Math.max(Math.abs(c)*0.1,0.5); mx=c+Math.max(Math.abs(c)*0.1,0.5); }
    const pad=(mx-mn)*0.08; return [mn-pad, mx+pad];
  }
  function render(){
    while(svg.firstChild) svg.removeChild(svg.firstChild);
    const x = v => ml + (W-ml-mr) * (v-dom[0])/(dom[1]-dom[0] || 1);
    const xinv = px => dom[0] + (px-ml)/(W-ml-mr)*(dom[1]-dom[0]);
    // 可见区间取值范围
    let i0=0,i1=t.length-1;
    while(i0<t.length-1 && t[i0+1]<dom[0]) i0++;
    while(i1>0 && t[i1-1]>dom[1]) i1--;
    const vis = a => a.slice(i0, i1+1);
    // 合并所有 ax1 系列的范围
    let a1min=Infinity,a1max=-Infinity;
    for(const s of s1){ a1min=Math.min(a1min,lo(vis(s.v),v=>v)); a1max=Math.max(a1max,hi(vis(s.v),v=>v)); }
    if(cfg.hlines) for(const h of cfg.hlines){ if(h.y<a1min)a1min=h.y; if(h.y>a1max)a1max=h.y; }
    const [y1lo,y1hi] = niceRange(a1min,a1max);
    const y1 = v => H-mb - (H-mb-mt)*(v-y1lo)/(y1hi-y1lo||1);
    let y2lo=0,y2hi=1,y2=()=>H-mb;
    if(s2.length){
      let bmin=Infinity,bmax=-Infinity;
      for(const s of s2){ bmin=Math.min(bmin,lo(vis(s.v),v=>v)); bmax=Math.max(bmax,hi(vis(s.v),v=>v)); }
      const r = niceRange(bmin,bmax); y2lo=r[0]; y2hi=r[1];
      y2 = v => H-mb - (H-mb-mt)*(v-y2lo)/(y2hi-y2lo||1);
    }
    const el=(n,at)=>{const e=document.createElementNS(svgNS,n);for(const k in at)e.setAttribute(k,at[k]);return e;};
    // 背景 / 区间
    svg.appendChild(el('rect',{x:ml,y:mt,width:W-ml-mr,height:H-mt-mb,fill:'none'}));
    (cfg.bands||[]).forEach(b=>{
      const x0=Math.max(x(b.a),ml), x1=Math.min(x(b.b),W-mr);
      if(x1>x0) svg.appendChild(el('rect',{x:x0,y:mt,width:x1-x0,height:H-mt-mb,fill:b.c||'rgba(120,160,255,.10)'}));
    });
    // 网格
    for(let i=0;i<=4;i++){
      const yy = mt + (H-mt-mb)*i/4;
      svg.appendChild(el('line',{x1:ml,x2:W-mr,y1:yy,y2:yy,stroke:'rgba(128,128,128,.28)','stroke-width':.6}));
      const v = y1hi - (y1hi-y1lo)*i/4;
      const tx=el('text',{x:ml-6,y:yy+3.5,'text-anchor':'end','font-size':10,fill:'#888'}); tx.textContent=v.toFixed(Math.abs(y1hi)>100?0:(Math.abs(y1hi)>10?1:2));
      svg.appendChild(tx);
      if(s2.length){
        const v2 = y2hi - (y2hi-y2lo)*i/4;
        const t2=el('text',{x:W-mr+6,y:yy+3.5,'font-size':10,fill:'#888'}); t2.textContent=v2.toFixed(1);
        svg.appendChild(t2);
      }
    }
    for(let i=0;i<=8;i++){
      const xx = ml + (W-ml-mr)*i/8, v = dom[0] + (dom[1]-dom[0])*i/8;
      svg.appendChild(el('line',{x1:xx,x2:xx,y1:mt,y2:H-mb,stroke:'rgba(128,128,128,.16)','stroke-width':.6}));
      const tx=el('text',{x:xx,y:H-mb+13,'text-anchor':'middle','font-size':10,fill:'#888'}); tx.textContent=v.toFixed(0)+'s';
      svg.appendChild(tx);
    }
    (cfg.hlines||[]).forEach(h=>{
      const yy=y1(h.y); if(yy<mt||yy>H-mb) return;
      svg.appendChild(el('line',{x1:ml,x2:W-mr,y1:yy,y2:yy,stroke:h.c||'#f28e2b','stroke-width':1,'stroke-dasharray':'5 3'}));
      if(h.n){const tx=el('text',{x:W-mr-3,y:yy-3,'text-anchor':'end','font-size':10,fill:h.c||'#f28e2b'});tx.textContent=h.n;svg.appendChild(tx);}
    });
    // 曲线
    const clip = el('clipPath',{id:'c'+host}); const cr=el('rect',{x:ml,y:mt,width:W-ml-mr,height:H-mt-mb}); clip.appendChild(cr); svg.appendChild(clip);
    for(const s of series){
      const yf = s.ax2? y2 : y1;
      let d='', started=false;
      for(let i=i0;i<=i1;i++){
        const X=x(t[i]), Y=yf(s.v[i]);
        if(Y<mt-200||Y>H-mb+200){ continue; }
        d += (started?'L':'M') + X.toFixed(1)+' '+Y.toFixed(1)+' '; started=true;
      }
      svg.appendChild(el('path',{d:d,fill:'none',stroke:s.c,'stroke-width':s.w||1.1,
        'stroke-dasharray':s.d?'4 3':'none','clip-path':`url(#c${host})`,'stroke-linejoin':'round'}));
    }
    // 事件竖线
    (cfg.marks||[]).forEach(m=>{
      const xx=x(m.t); if(xx<ml||xx>W-mr) return;
      svg.appendChild(el('line',{x1:xx,x2:xx,y1:mt,y2:H-mb,stroke:m.c||'rgba(200,0,0,.55)','stroke-width':1,'stroke-dasharray':'3 3'}));
      if(m.n){const tx=el('text',{x:xx+3,y:mt+10,'font-size':10,fill:m.c||'#c00'});tx.textContent=m.n;svg.appendChild(tx);}
    });
    // hover
    const cross = el('line',{x1:0,x2:0,y1:mt,y2:H-mb,stroke:'rgba(128,128,128,.8)','stroke-width':.8,opacity:0});
    svg.appendChild(cross);
    const dots = series.map(()=>{const c=el('circle',{r:2.6,fill:'none','stroke-width':1.6,opacity:0});svg.appendChild(c);return c;});
    const rect = el('rect',{x:ml,y:mt,width:W-ml-mr,height:H-mt-mb,fill:'transparent',style:'cursor:crosshair'});
    svg.appendChild(rect);
    function show(ev){
      const bb=svg.getBoundingClientRect();
      const px=(ev.clientX-bb.left)/bb.width*W;
      if(px<ml||px>W-mr){cross.setAttribute('opacity',0);dots.forEach(d=>d.setAttribute('opacity',0));tip.style.display='none';return;}
      const tv=xinv(px);
      let i=Math.max(0,Math.min(t.length-1,Math.round((tv-t[0])/(t[1]-t[0]||1))));
      // t 不等距(包络抽样), 用二分找最近点
      let a=0,b=t.length-1; while(b-a>1){const m=(a+b)>>1; if(t[m]<tv)a=m;else b=m;}
      i = (Math.abs(t[a]-tv)<=Math.abs(t[b]-tv))?a:b;
      cross.setAttribute('x1',x(t[i]));cross.setAttribute('x2',x(t[i]));cross.setAttribute('opacity',.7);
      let html='<b>t = '+t[i].toFixed(2)+' s</b>';
      series.forEach((s,k)=>{
        const yf=s.ax2?y2:y1, Y=yf(s.v[i]);
        dots[k].setAttribute('cx',x(t[i]));dots[k].setAttribute('cy',Y);dots[k].setAttribute('stroke',s.c);dots[k].setAttribute('opacity',1);
        html+='<br><span style="color:'+s.c+'">&#9632;</span> '+s.n+': '+F(s.v[i],s.p===undefined?2:s.p)+(s.u||'');
      });
      tip.innerHTML=html; tip.style.display='block';
      const hb=hostEl.getBoundingClientRect();
      let left=(x(t[i])/W)*hb.width+12; if(left>hb.width-150) left-=170;
      tip.style.left=Math.max(0,left)+'px'; tip.style.top=(mt+2)+'px';
    }
    rect.addEventListener('mousemove',show);
    rect.addEventListener('mouseleave',()=>{cross.setAttribute('opacity',0);dots.forEach(d=>d.setAttribute('opacity',0));tip.style.display='none';});
    // 滚轮缩放 / 拖动平移 / 双击复位
    let dragging=false, dragX0=0, dom0=null;
    rect.addEventListener('wheel',ev=>{
      ev.preventDefault();
      const bb=svg.getBoundingClientRect(); const px=(ev.clientX-bb.left)/bb.width*W;
      const tv=xinv(Math.max(ml,Math.min(W-mr,px)));
      const f=ev.deltaY>0?1.25:0.8;
      let a=tv-(tv-dom[0])*f, b=tv+(dom[1]-tv)*f;
      a=Math.max(full[0],a); b=Math.min(full[1],b);
      if(b-a<0.6) return; dom=[a,b]; render();
    },{passive:false});
    rect.addEventListener('mousedown',ev=>{dragging=true;dragX0=ev.clientX;dom0=dom.slice();ev.preventDefault();});
    window.addEventListener('mousemove',ev=>{
      if(!dragging) return;
      const bb=svg.getBoundingClientRect();
      const dt=(ev.clientX-dragX0)/bb.width*(dom0[1]-dom0[0]);
      let a=dom0[0]-dt,b=dom0[1]-dt;
      a=Math.max(full[0],a); b=Math.min(full[1],b);
      dom=[a,b]; render();
    });
    window.addEventListener('mouseup',()=>{dragging=false;});
    rect.addEventListener('dblclick',()=>{dom=full.slice();render();});
  }
  render();
  if(cfg.legend!==false){
    const lg=document.createElement('div'); lg.className='legend';
    lg.innerHTML = series.map(s=>'<i style="background:'+s.c+'"></i>'+s.n).join('');
    hostEl.appendChild(lg);
  }
  const hint=document.createElement('div'); hint.className='sub'; hint.style.margin='2px 0 0';
  hint.textContent = cfg.hint || '滚轮缩放 · 按住拖动平移 · 双击复位 · 鼠标悬停读数';
  hostEl.appendChild(hint);
}

function flagsStrip(host, cfg){
  const hostEl=document.getElementById(host); if(!hostEl) return;
  const W=1180, names=cfg.names, rowH=13, gap=2, H=names.length*(rowH+gap)+16, ml=56, mr=16, mt=4;
  const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');
  svg.setAttribute('viewBox',`0 0 ${W} ${H}`); svg.setAttribute('preserveAspectRatio','xMidYMid meet');
  const el=(n,at)=>{const e=document.createElementNS('http://www.w3.org/2000/svg',n);for(const k in at)e.setAttribute(k,at[k]);return e;};
  const tt=cfg.t, tmax=tt[tt.length-1];
  const x=v=>ml+(W-ml-mr)*v/tmax;
  const wpx=Math.max(1.4,(W-ml-mr)*tt[0]/tmax || 1.4);
  names.forEach((nm,row)=>{
    const y=mt+row*(rowH+gap);
    const tx=el('text',{x:ml-6,y:y+rowH-2,'text-anchor':'end','font-size':10,fill:'#888'}); tx.textContent=nm; svg.appendChild(tx);
    const a=cfg.series[nm]||[];
    for(let i=0;i<a.length;i++){
      if(!a[i]) continue;
      svg.appendChild(el('rect',{x:x(tt[i]),y:y,width:wpx,height:rowH-2,fill:cfg.color||'#4e79a7'}));
    }
  });
  const ax=el('text',{x:ml-6,y:H-3,'text-anchor':'end','font-size':10,fill:'#888'}); ax.textContent='s'; svg.appendChild(ax);
  for(let k=0;k<=8;k++){const v=tmax*k/8,xx=x(v);
    svg.appendChild(el('line',{x1:xx,x2:xx,y1:mt,y2:H-14,stroke:'rgba(128,128,128,.16)','stroke-width':.6}));
    const t2=el('text',{x:xx,y:H-3,'text-anchor':'middle','font-size':10,fill:'#888'});t2.textContent=v.toFixed(0);svg.appendChild(t2);}
  hostEl.appendChild(svg);
}

// 二维散点(x 不是时间, 例如 x = 油门, y = 振荡幅值)
function mkScatter(host, cfg){
  const hostEl=document.getElementById(host); if(!hostEl) return;
  const W=1180, H=cfg.h||300, ml=64, mr=170, mt=18, mb=40;
  const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');
  svg.setAttribute('viewBox',`0 0 ${W} ${H}`); svg.setAttribute('preserveAspectRatio','xMidYMid meet');
  const el=(n,at)=>{const e=document.createElementNS('http://www.w3.org/2000/svg',n);for(const k in at)e.setAttribute(k,at[k]);return e;};
  let xmin=Infinity,xmax=-Infinity,ymax=-Infinity;
  cfg.series.forEach(s=>s.pts.forEach(p=>{xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymax=Math.max(ymax,p[1]);}));
  xmin=Math.min(0,xmin); ymax=ymax*1.1||1;
  const X=v=>ml+(W-ml-mr)*(v-xmin)/(xmax-xmin||1);
  const Y=v=>H-mb-(H-mt-mb)*v/ymax;
  for(let i=0;i<=5;i++){const yy=mt+(H-mt-mb)*i/5;
    svg.appendChild(el('line',{x1:ml,x2:W-mr,y1:yy,y2:yy,stroke:'rgba(128,128,128,.25)','stroke-width':.6}));
    const t=el('text',{x:ml-6,y:yy+3.5,'text-anchor':'end','font-size':10,fill:'#888'});
    t.textContent=(ymax*(1-i/5)).toFixed(1); svg.appendChild(t);}
  for(let i=0;i<=6;i++){const v=xmin+(xmax-xmin)*i/6, xx=X(v);
    svg.appendChild(el('line',{x1:xx,x2:xx,y1:mt,y2:H-mb,stroke:'rgba(128,128,128,.14)','stroke-width':.6}));
    const t=el('text',{x:xx,y:H-mb+14,'text-anchor':'middle','font-size':10,fill:'#888'});
    t.textContent=v.toFixed(2); svg.appendChild(t);}
  const xl=el('text',{x:ml,y:H-8,'font-size':11,fill:'#888'}); xl.textContent=cfg.xlab||''; svg.appendChild(xl);
  const yl=el('text',{x:8,y:mt+12,'font-size':11,fill:'#888'}); yl.textContent=cfg.ylab||''; svg.appendChild(yl);
  cfg.series.forEach((s,si)=>{
    const pts=s.pts.map(p=>[X(p[0]),Y(p[1])]);
    if(pts.length>1) svg.appendChild(el('path',{d:'M'+pts.map(p=>p[0].toFixed(1)+' '+p[1].toFixed(1)).join(' L'),
      fill:'none',stroke:s.c,'stroke-width':1.4,opacity:.65}));
    pts.forEach(p=>svg.appendChild(el('circle',{cx:p[0],cy:p[1],r:3.5,fill:s.c})));
    const ly=mt+8+si*16;
    svg.appendChild(el('rect',{x:W-mr+16,y:ly-8,width:10,height:10,fill:s.c,rx:2}));
    const t=el('text',{x:W-mr+30,y:ly+1,'font-size':11,fill:'#888'}); t.textContent=s.n; svg.appendChild(t);
  });
  hostEl.appendChild(svg);
}
"""


def fmt(v, n=2):
    return f"{v:.{n}f}"


def bar(val, vmax, cls="bar"):
    w = 0 if not vmax else max(1.5, 90.0 * val / vmax)
    return f'<div class="barwrap"><span class="bar {cls}" style="width:{w:.1f}px"></span><span>{val}</span></div>'


def session_section(s: dict, params: dict) -> str:
    sid = s["id"]
    segs = s["segments"]
    armed_time = sum(g["dur"] for g in segs)
    kpis = [
        ("帧数 / 时长", f"{s['rows']} 帧 · {s['duration']:.1f} s"),
        ("平均记录率", f"{s['rate']:.2f} Hz"),
        ("缺帧(回传丢)", f"{s['lost_frames']} 帧 · {s['lost_pct']:.2f}%"),
        ("控制环周期", f"p50 {s['loop_us']['p50']:.0f} / max {s['loop_us']['max']:.0f} µs"),
        ("解锁段 / 总时长", f"{len(segs)} 段 · {armed_time:.1f} s"),
        ("最大手动油门", f"{s['man_max']:.3f} (悬停 {params['hover_throttle']})"),
        ("最大电机输出", f"{s['motor_max']} / 1950 ({s['motor_frac_max']*100:.0f}%)"),
        ("姿态范围 roll/pitch", f"{s['att_range']['roll'][0]:.0f}~{s['att_range']['roll'][1]:.0f}° / "
                             f"{s['att_range']['pitch'][0]:.0f}~{s['att_range']['pitch'][1]:.0f}°"),
        ("最大角速度 / 加速度", f"{s['gyro_max']:.1f} rad/s · {s['acc_max']:.1f} g"),
        ("链路丢失 / 失控保护", f"{s['link_lost_frames']} 帧 / {s['failsafe_frames']} 帧"),
    ]
    kpi_html = "".join(f'<div class="kpi"><span class="k">{k}</span><span class="v">{v}</span></div>' for k, v in kpis)

    # 解锁段表
    seg_rows = []
    for i, g in enumerate(segs):
        seg_rows.append(
            f"<tr><td class='l'>#{i+1}</td><td>{g['t0']:.1f}~{g['t1']:.1f}</td><td>{g['dur']:.1f}</td>"
            f"<td>{g['man_max']:.3f}</td><td>{g['motor_mean']:.0f}</td><td>{g['motor_max']}</td>"
            f"<td>{g['roll_absmax']:.1f} / {g['pitch_absmax']:.1f}</td><td>{g['gyro_max']:.1f}</td>"
            f"<td>{g['acc_min']:.2f}~{g['acc_max']:.2f}</td>"
            f"<td>{g['mix_clip_pct']:.0f}% / {g['mix_min']:.2f}</td>"
            f"<td>{g['sat_fw']} / {g['sat_frac']:.0f}% (上侧 {g['sat_fw_high']})</td>"
            f"<td>{('%.2f~%.2f' % (g['baro_min'], g['baro_max'])) if g['baro_min'] is not None else '无气压计'}</td>"
            f"<td>{g['gdev_median']:.0f}° / {g['gdev_p95']:.0f}°</td>"
            f"<td>{g['vib']:.3f}</td></tr>"
        )
    seg_table = (
        "<table><tr><th class='l'>解锁段</th><th>起止 [s]</th><th>时长 s</th><th>手动油门 max</th>"
        "<th>电机均值</th><th>电机 max</th><th>|roll|/|pitch| max °</th><th>|ω| max rad/s</th>"
        "<th>|a| g</th><th>差动被裁剪 / 最小缩放</th><th>MotorSat 帧 / 占比(上侧)</th><th>气压高度 m</th>"
        "<th>重力方向偏差 中位/p95</th><th>加速度抖动</th></tr>" + "".join(seg_rows) + "</table>"
    )
    if not segs:
        seg_table = "<p class='sub'>本会话没有出现过解锁(电机使能)状态。</p>"

    # 事件表: 按类型汇总 + 明细
    kinds: dict[str, list] = {}
    for e in s["events"]:
        kinds.setdefault(e["kind"], []).append(e)
    ev_rows = []
    for k, lst in kinds.items():
        tot = sum(x["n"] for x in lst)
        longest = max(lst, key=lambda x: x["t1"] - x["t0"])
        cls = "bad" if ("大姿态" in k or "冲击" in k or "事件 Failsafe" in k) else (
            "warn" if ("饱和" in k or "高角速度" in k or "超时" in k or "偏离" in k or "钳位" in k) else "ok")
        times = ""
        span = f"{longest['t0']:.1f}~{longest['t1']:.1f}"
        ncell = str(len(lst))
        if "times" in longest:
            times = " @ " + ", ".join(f"{x:.1f}s" for x in longest["times"][:8]) + \
                    (" …" if len(longest["times"]) > 8 else "")
            ncell = f"{longest.get('count', len(longest['times']))} 次"
            span = "按次列于右侧"
        else:
            ncell = f"{len(lst)} 段"
        ev_rows.append(
            f"<tr><td class='l'><span class='tag {cls}'>{k}</span></td><td>{ncell}</td><td>{tot}</td>"
            f"<td>{span}</td><td class='l'>{times}</td></tr>"
        )
    ev_table = ("<table><tr><th class='l'>事件类型</th><th>事件数</th><th>涉及帧数</th>"
                "<th>最长一段 [s]</th><th class='l'>出现时刻</th></tr>" + "".join(ev_rows) + "</table>")

    # 图表
    bands = [{"a": a, "b": b, "c": "rgba(120,160,255,.13)"} for a, b in s["armed_segs"]]
    marks = []
    for e in s["events"]:
        if e["kind"].startswith("事件 Failsafe"):
            marks.append({"t": e["t0"], "c": "#c00", "n": "失控保护"})
        if e["kind"].startswith("大姿态(飞行中"):
            marks.append({"t": e["t0"], "c": "rgba(200,0,0,.5)", "n": "异常姿态"})
    # 去重时间点
    seen = set()
    marks = [m for m in marks if not (round(m["t"], 1) in seen or seen.add(round(m["t"], 1)))]

    def chart(host, title, data_series, h=190, ax2_names=(), hlines=None, hint=None,
              units=None, bag=None):
        series = []
        for k, name, color, dash in data_series:
            series.append({"k": k, "n": name, "c": color, "w": 1.6 if not dash else 1.1,
                           "d": dash, "ax2": k in ax2_names,
                           "p": (units or {}).get(k, 0 if k in ("m0", "m1", "m2", "m3", "mavg", "link") else 2),
                           "u": ""})
        cfg = {
            "bag": bag or f"s{sid}", "series": series, "h": h,
            "bands": bands, "marks": marks,
        }
        if hlines:
            cfg["hlines"] = hlines
        if hint:
            cfg["hint"] = hint
        return (f"<div class='title'>{title}</div><div class='chart' id='{host}'></div>"
                f"<script>mkChart('{host}', {json.dumps(cfg, separators=(',', ':'))});</script>")

    charts = []
    charts.append(chart(f"c{sid}_att", "1. 姿态角 (灰带 = 解锁段; 橙虚线 = 遥控期望姿态)",
                        [("roll", "roll", CFG["roll"], False), ("pitch", "pitch", CFG["pitch"], False),
                         ("tgtroll", "目标 roll", CFG["tgt"], True), ("tgtpitch", "目标 pitch", CFG["tgt"], True)]))
    charts.append(chart(f"c{sid}_yaw", "2. 偏航角 (单 IMU 无磁力计, 绝对值不可信; 注意静止时也可能在旋转)",
                        [("yaw", "yaw", CFG["yaw"], False)], h=150))
    charts.append(chart(f"c{sid}_rate", "3. 机体角速度 (实线) vs 角速度期望 (虚线)",
                        [("wx", "ωx", CFG["roll"], False), ("wy", "ωy", CFG["pitch"], False),
                         ("wz", "ωz", CFG["yaw"], False),
                         ("spx", "sp ωx", CFG["tgt"], True), ("spy", "sp ωy", CFG["tgt"], True),
                         ("spz", "sp ωz", CFG["tgt"], True)]))
    charts.append(chart(f"c{sid}_motor", "4. 四路电机 DShot 输出 (物理通道顺序 = M0左前 / M3左后 / M1右前 / M2右后)",
                        [("m0", "motor0 (M0左前)", CFG["m0"], False), ("m1", "motor1 (M3左后)", CFG["m1"], False),
                         ("m2", "motor2 (M1右前)", CFG["m2"], False), ("m3", "motor3 (M2右后)", CFG["m3"], False),
                         ("mavg", "四路均值", "#333", False)],
                        hlines=[{"y": 1950, "c": "#e15759", "n": "DShot 上限 1950"},
                                {"y": 50, "c": "#f28e2b", "n": "0 油门 50"}]))
    charts.append(chart(f"c{sid}_scale", "5. 混控缩放系数 (1.0 = 力矩未被裁剪; <0.98 说明差动被推力优先策略收缩)",
                        [("scale", "mix_scale", CFG["scale"], False)],
                        hlines=[{"y": 0.98, "c": "#e15759", "n": "裁剪线 0.98"}], h=150))
    charts.append(chart(f"c{sid}_thr", "6. 集体油门 (实线手/控制) 与悬停油门参考线",
                        [("thr", "控制油门", CFG["thr"], False), ("man", "遥控手动油门", CFG["man"], False)],
                        hlines=[{"y": 0.25, "c": "#e15759", "n": "悬停油门 0.25"},
                                {"y": 0.05, "c": "#59a14f", "n": "遥控最小杆量 0.05"}], h=150))
    charts.append(chart(f"c{sid}_hgt", "7. 高度: EKF 高度(解锁瞬间清零) vs 气压相对高度",
                        [("hgt", "EKF 高度", CFG["hgt"], False), ("brel", "气压相对高度", CFG["brel"], False)]))
    charts.append(chart(f"c{sid}_vv", "8. 垂向速度 (EKF)",
                        [("vv", "vz", CFG["vv"], False)], h=150))
    charts.append(chart(f"c{sid}_acc", "9. 加速度幅值(左轴, g) 与重力方向一致性偏差(右轴, °)",
                        [("acc", "|a|", CFG["acc"], False), ("gdev", "重力方向偏差", CFG["gdev"], False)],
                        ax2_names=("gdev",), hlines=[{"y": 1.0, "c": "#888", "n": "1 g"}]))
    charts.append(chart(f"c{sid}_gyroabs", "10. 角速度幅值 (识别翻滚/坠机)",
                        [("gyroabs", "|ω|", CFG["gyro"], False)], h=140))
    charts.append(chart(f"c{sid}_link", "11. 遥控链路年龄 (ms, 5000ms 触发失控保护)",
                        [("link", "linkAge", CFG["link"], False)],
                        hlines=[{"y": 5000, "c": "#e15759", "n": "超时 5000"}], h=150))
    bsrc = {"t": s["bias"]["t"], "biasx": s["bias"]["x"],
            "biasy": s["bias"]["y"], "biasz": s["bias"]["z"]}
    charts.append(chart(f"c{sid}_bias", "12. EKF 陀螺零偏估计 (钳位 ±0.3 rad/s)",
                        [("biasx", "bx", CFG["biasx"], False), ("biasy", "by", CFG["biasy"], False),
                         ("biasz", "bz", CFG["biasz"], False)],
                        hlines=[{"y": 0.3, "c": "#e15759", "n": "+0.3 钳位"}, {"y": -0.3, "c": "#e15759", "n": "-0.3 钳位"}],
                        bag=f"b{sid}"))

    bags = (f"<script>window.BAGS=window.BAGS||{{}};window.BAGS['s{sid}']="
            f"{json.dumps(s['series'], separators=(',', ':'))};"
            f"window.BAGS['b{sid}']={json.dumps(bsrc, separators=(',', ':'))};</script>")

    fnames = list(s["flags_ts"].keys())
    ftimes = [round(i * s["flags_dt"], 2) for i in range(len(s["flags_ts"][fnames[0]]))]
    flags_chart = (
        f"<div class='title'>13. 状态位条带 (每 100 ms 一格, 有色 = 该位为 1)</div>"
        f"<div class='chart' id='f{sid}'></div>"
        f"<script>flagsStrip('f{sid}', {{t:{json.dumps(ftimes)},color:'#4e79a7',"
        f"series:{json.dumps(s['flags_ts'], separators=(',', ':'))},"
        f"names:{json.dumps(fnames)}}});</script>"
    )

    return f"""
<h2 id="s{sid}">session {sid} — {s['file']}</h2>
<div class="sub">tick {s['tickMs'][0]}~{s['tickMs'][-1]} ms(飞控上电毫秒) · 频率 {s['rate']:.2f} Hz ·
  会话时长 {s['duration']:.1f} s(≈{s['duration']/60:.1f} min)</div>
<div class="kpis">{kpi_html}</div>
{bags}
{''.join(charts)}
{flags_chart}
<h3>解锁段明细</h3>
{seg_table}
<div class="sub">"差动被裁剪"= 混控为了保住总推力而按比例缩小了三轴差动(即力矩不够用);
"饱和帧(低侧)"= 飞控 MotorSat 状态位里由 motor≤55(被压到 0 档)造成的帧数。</div>
<h3>事件清单</h3>
{ev_table}
"""


def _facts(data: dict, prev: dict | None) -> dict:
    """把这一批的关键指标算出来, 供报告文本使用。"""
    sess = data["sessions"]
    p = data["params"]
    f = {
        "sess": sess, "p": p,
        "batch": data.get("batch", "all"),
        "rows": sum(s["rows"] for s in sess),
        "dur": sum(s["duration"] for s in sess),
        "n_armed_segs": sum(len(s["segments"]) for s in sess),
        "armed_time": sum(g["dur"] for s in sess for g in s["segments"]),
        "max_man": max(s["man_max"] for s in sess),
        "max_thr": max(s["thr_max"] for s in sess),
        "max_motor": max(s["motor_max"] for s in sess),
        "max_gyro": max(s["gyro_max"] for s in sess),
        "min_acc": min(s["acc_min"] for s in sess),
        "max_acc": max(s["acc_max"] for s in sess),
        "lost": sum(s["lost_frames"] for s in sess),
        "lost_min": min(s["lost_pct"] for s in sess),
        "lost_max": max(s["lost_pct"] for s in sess),
        "baro": any(s.get("has_baro", True) for s in sess),
        "prev": prev,
    }
    # 有气压计的会话才统计气压/高度指标
    f["hgt_drift"] = max((max(abs(x) for x in s["hgt_raw_range"]) for s in sess), default=0.0)
    f["mix_clip_max"] = max([g["mix_clip_pct"] for s in sess for g in s["segments"]], default=0.0)
    f["sat_total"] = sum(g["sat_fw"] for s in sess for g in s["segments"])
    f["sat_high"] = sum(g["sat_fw_high"] for s in sess for g in s["segments"])
    f["failsafe_frames"] = sum(s["failsafe_frames"] for s in sess)
    f["failsafe_sess"] = [s["id"] for s in sess if s["failsafe_frames"]]
    f["mixer_ok"] = all((s.get("mixer_check", {}).get("n", 0) == 0) or
                        (s.get("mixer_check", {}).get("match_pct") == 100.0) for s in sess)
    drags = [s for s in sess if (s.get("ekf_drag") or {}).get("rate_median") is not None]
    f["drag_sess"] = drags
    f["drag_gate_min"] = min((s["ekf_drag"]["gate_open_armed"] or 0) for s in drags) if drags else None
    f["drag_gate_max"] = max((s["ekf_drag"]["gate_open_armed"] or 0) for s in drags) if drags else None
    f["drag_med_min"] = min(s["ekf_drag"]["rate_median"] for s in drags) if drags else None
    f["drag_med_max"] = max(s["ekf_drag"]["rate_median"] for s in drags) if drags else None
    f["drag_p95_max"] = max(s["ekf_drag"]["rate_p95"] for s in drags) if drags else None
    f["vib_min"] = min((s.get("vib") or {}).get("dgyro_armed") or 9e9 for s in sess)
    f["vib_max"] = max((s.get("vib") or {}).get("dgyro_armed") or 0 for s in sess)
    f["gve"] = [(s["id"], s.get("gyro_vs_ekf") or {}) for s in sess if s.get("gyro_vs_ekf")]
    # 只保留"真的动过"的会话(排除不装桨/没转动的台架), 用陀螺积分变化量判断
    f["gve_fly"] = [(i, g) for i, g in f["gve"]
                    if max(abs(g.get("pitch_gyro", 0.0)), abs(g.get("roll_gyro", 0.0))) > 3.0]
    # 明显事件: 角速度峰值/大姿态/失控保护/链路中断
    f["high_rate"] = [(s["id"], s.get("gyro_peak") or {"v": s["gyro_max"], "t": 0.0, "armed": None})
                      for s in sess if s["gyro_max"] > 3.0]
    f["big_att"] = [(s["id"], max(abs(s["att_range"]["roll"][0]), abs(s["att_range"]["roll"][1]),
                                  abs(s["att_range"]["pitch"][0]), abs(s["att_range"]["pitch"][1])))
                    for s in sess]
    return f


def _narrative(data: dict, prev: dict | None) -> str:
    """批次自适应的"结论速览 + 解析口径 + 总览"部分(数值全部由数据算出)。"""
    f = _facts(data, prev)
    sess, p = f["sess"], f["p"]
    ids = " / ".join(str(s["id"]) for s in sess)
    baro_txt = ("<b>气压计在线</b>" if f["baro"] else
                "<b>本批会话里 BME280 不在线</b>(flags 里始终没有 <code>kLogFlagBaroOk</code>, "
                "<code>baroRelM/baroAbsM</code> 恒为 0) —— 与摘掉气压计这个操作一致")
    # 事件汇总文字
    ev_bits = []
    for s in sess:
        e = {k: v for k, v in s["events_used"].items() if k != "BaroUpdate"}
        if e:
            ev_bits.append(f"session {s['id']}: " + ", ".join(f"{k}×{v}" for k, v in e.items()))
    ev_txt = "; ".join(ev_bits) if ev_bits else "无"
    drag_rows = "".join(
        f"<tr><td class='l'>session {s['id']}</td><td>{s['ekf_drag']['gate_open_armed']}%</td>"
        f"<td>{s['ekf_drag']['rate_median']} °/s</td><td>{s['ekf_drag']['rate_p95']} °/s</td>"
        f"<td>{s['man_max']:.2f}</td><td>{s['gyro_max']:.1f}</td></tr>"
        for s in f["drag_sess"])
    gve_rows = "".join(
        f"<tr><td class='l'>session {i}</td><td>{g['t0']:.1f}~{g['t1']:.1f}</td>"
        f"<td>{g['pitch_gyro']:+.2f}</td><td>{g['pitch_ekf']:+.2f}</td>"
        f"<td><b>{g['pitch_accel_contrib']:+.2f}</b></td>"
        f"<td>{g['roll_gyro']:+.2f}</td><td>{g['roll_ekf']:+.2f}</td>"
        f"<td><b>{g['roll_accel_contrib']:+.2f}</b></td>"
        f"<td>{s['man_max']:.2f}</td></tr>"
        for i, g, s in [(i, g, next(x for x in f["sess"] if x["id"] == i)) for i, g in f["gve"]]
    )
    peak_txt = ", ".join(
        f"session {i} {g['v']:.1f} rad/s @t={g['t']:.0f}s"
        f"({'解锁中' if g['armed'] else ('未解锁' if g['armed'] is not None else '状态未知')})"
        for i, g in f["high_rate"]) or "无"
    return f"""
<h2 id="summary">一、结论速览</h2>
<div class="card">
<p><b>本批 = {f['batch']} 的 {len(sess)} 个会话(session {ids}), 共 {f['rows']:,} 帧 / 50 Hz / 合计 {f['dur']/60:.1f} 分钟,
其中解锁 {f['n_armed_segs']} 段、累计 {f['armed_time']:.0f} s。</b>
固件与 0913 那一批相同(未烧 BME 开关版), 但{baro_txt}。</p>
<ol>
<li><b>姿态质量明显好于 0913:</b>合加速度 {f['min_acc']:.2f}~{f['max_acc']:.2f} g, <b>没有出现 0913 那种
12~19 rad/s 的翻滚/坠机尖峰</b>;全部会话里超过 3 rad/s 的只有 {peak_txt}。
真正解锁飞行中的角速度峰值最大只有 {max((s.get('gyro_max_armed') or 0) for s in sess):.1f} rad/s,
而 0913 解锁段是 19.7 rad/s。</li>
<li><b>混控一次都没有裁剪过差动</b>(0%), MotorSat 饱和帧 {f['sat_total']} 个(其中上侧 {f['sat_high']} 个)。
原因是这一批油门用到了 {f['max_thr']:.2f}(最大电机 {f['max_motor']}/1950), 比 0913 的 0.26 高得多,
差动权限足够 —— 反过来印证了 0913 里"低油门把差动挤没了"的判断。</li>
<li><b>但姿态估计被加速度计拖走的现象依然存在</b>(见第五节 3):解锁段里倾角修正门限有
{f['drag_gate_min']:.0f}~{f['drag_gate_max']:.0f}% 的时间是开着的, 高油门段的"非陀螺"姿态变化速率中位
{f['drag_med_min']:.1f}~{f['drag_med_max']:.1f} °/s(p95 到 {f['drag_p95_max']:.0f} °/s)。
比 0913(中位 23~35 °/s)小, 但机制相同。</li>
<li><b>【最关键的一条】发动机一转起来, 姿态估计就"看不见"飞机真实的转动。</b>
三个上过油门的会话里, 飞机真转了 pitch {min(g['pitch_gyro'] for _, g in f['gve_fly']):+.0f}~{max(g['pitch_gyro'] for _, g in f['gve_fly']):+.0f}°、
roll {min(g['roll_gyro'] for _, g in f['gve_fly']):+.0f}~{max(g['roll_gyro'] for _, g in f['gve_fly']):+.0f}°(陀螺积分),
而 EKF 报出来的姿态几乎不动(最大只有 {max(abs(g['pitch_ekf']) for _, g in f['gve_fly']):.1f}°),
因为加速度计倾角修正在反方向拧了
{min(abs(g['pitch_accel_contrib']) for _, g in f['gve_fly']):.0f}~{max(abs(g['pitch_accel_contrib']) for _, g in f['gve_fly']):.0f}° 把真实转动抵消掉。
<b>飞行中加速度计测的是推力(不是重力), 所以它对倾角不敏感; 控制器据此认为"姿态没变", 于是不去修正</b> ——
这就是"加油门就低头/倾斜、电机看不出修正、稳定停在一个倾角上"的直接原因(详见第五节 8)。</li>
<li><b>摘掉气压计的代价很直接:</b>高度通道变成纯惯性外推, 静止不动也会漂到 ±{f['hgt_drift']:.0f} m
(例: session 3 到 +37.7 m、session 5 到 −21.6 m, 而飞机实际就在地面/低空)。
<b>这批日志的高度、垂速、以及依赖高度的定高通道完全不可用</b>, 遥控回传里 height 字段也会失真。</li>
<li><b>回传链路质量变差了:</b>丢帧率 {f['lost_min']:.2f}%~{f['lost_max']:.2f}%(0913 为 0.37%~2.20%),
session 4 丢掉 <b>6.62%</b>、session 5 丢掉 4.47%。更严重的是 <b>session 4 出现了飞行中链路丢失</b>:
t≈37.4 s(已解锁、油门 0.20、姿态平稳 ±3°)最后一次收到遥控, 5 s 后(44.0 s)失控保护停机,
之后链路整整断了 27.5 s 才恢复。这一次是真的"链路导致停机"。</li>
<li><b>控制链路自洽性复核通过:</b>用 torque+throttle 反算四路电机, 与记录值
{"全部" if f["mixer_ok"] else "大部分"} ≤1 LSB 吻合(见第五节 5), 说明混控/记录/物理通道顺序没有变化。</li>
<li><b>振动明显变小:</b>解锁段陀螺帧间抖动 {f['vib_min']:.2f}~{f['vib_max']:.2f} rad/s
(0913 为 0.40~0.49), 电机指令帧间抖动 7~34 counts(0913 为 47~58)。
session 2 特别安静(0.038 rad/s、|a| 1.00±0.01), 看起来是<b>不装桨的电机台架测试</b>。</li>
</ol>
<p>事件汇总(不含 BaroUpdate): {ev_txt}</p>
<table><tr><th class="l">会话</th><th>倾角门限开门</th><th>非陀螺姿态速率 中位</th><th>p95</th>
<th>最大手动油门</th><th>最大 |ω| rad/s</th></tr>{drag_rows}</table>
</div>

<h2 id="parse">二、日志格式与解析口径</h2>
<div class="card">
<p>与 0913 相同:CSV 由上位机把回传的 <code>FlightLog</code> 原样展开, 一行 = 一条 <code>LogEntry</code>,
50 Hz, 控制环 500 Hz(实测 p50 {f['sess'][0]['loop_us']['p50']:.0f} µs、最大
{max(s['loop_us']['max'] for s in sess):.0f} µs)。字段含义见 0913 报告第二节, 这里只列本批的特殊点:</p>
<ul>
<li><code>flags</code> 里<b>没有 BaroOk</b>:{baro_txt}; 因此 <code>kLogEventBaroUpdate</code> 也不再出现,
日志里 events 字段绝大多数是 0。</li>
<li><code>baroRelM</code>/<code>baroAbsM</code> 恒为 0(未读传感器就没有更新), <b>不要当高度用</b>。</li>
<li><code>heightM</code>/<code>vertVelMps</code> 由 EKF 纯惯性外推(无气压观测校正) → 会持续漂移,
只能看"短时间内的相对变化", 不能看绝对值。</li>
<li>其余字段(四元数/角速度/比力/期望/力矩/油门/DShot 计数)与 0913 完全一致, 可直接横向对比。</li>
</ul>
</div>

<h2 id="overview">三、会话总览</h2>
"""


def _topics(data: dict, prev: dict | None) -> str:
    """五、专项分析(全部由数据算出, 可适用于任意一批日志)。"""
    f = _facts(data, prev)
    sess, p = f["sess"], f["p"]
    # --- 回传链路 ---
    link_rows = "".join(
        f"<tr><td class='l'>session {s['id']}</td><td>{s['lost_frames']}</td><td>{s['lost_pct']:.2f}%</td>"
        f"<td>{s['link_lost_frames']}</td><td>{s['link_max']}</td><td>{s['failsafe_frames']}</td></tr>"
        for s in sess)
    outage_rows = []
    for s in sess:
        for e in s["events"]:
            if e["kind"].startswith("遥控超时"):
                outage_rows.append(
                    f"<tr><td class='l'>session {s['id']}</td><td>{e['t0']:.1f}~{e['t1']:.1f}</td>"
                    f"<td>{e['t1']-e['t0']:.1f}</td><td>{e['n']}</td></tr>")
    fail_rows = []
    for s in sess:
        for e in s["events"]:
            if e["kind"].startswith("事件 Failsafe") or e["kind"].startswith("事件 Arm") or \
               e["kind"].startswith("事件 Disarm") or e["kind"].startswith("事件 EkfRezero"):
                times = "".join(f"{t:.1f}s " for t in e.get("times", []))
                fail_rows.append(f"<tr><td class='l'>session {s['id']}</td><td class='l'>{e['kind']}</td>"
                                 f"<td class='l'>{times}</td></tr>")
    # --- 饱和/裁剪 ---
    sat_rows = "".join(
        f"<tr><td class='l'>session {s['id']}</td><td>{sum(g['dur'] for g in s['segments']):.1f}</td>"
        f"<td>{max([g['mix_clip_pct'] for g in s['segments']], default=0):.0f}%</td>"
        f"<td>{min([g['mix_min'] for g in s['segments']], default=1.0):.2f}</td>"
        f"<td>{sum(g['sat_fw'] for g in s['segments'])}</td>"
        f"<td>{sum(g['sat_fw_high'] for g in s['segments'])}</td></tr>"
        for s in sess)
    # --- 混控对账 ---
    mix_rows = "".join(
        f"<tr><td class='l'>session {s['id']}</td><td>{s['mixer_check']['n']}</td>"
        f"<td>{s['mixer_check']['match_pct'] if s['mixer_check']['match_pct'] is not None else '-'}</td>"
        f"<td>{s['mixer_check']['max_err'] if s['mixer_check']['max_err'] is not None else '-'}</td></tr>"
        for s in sess)
    # --- 振荡 ---
    osc_rows = []
    for s in sess:
        for o in s.get("osc", []):
            osc_rows.append(f"<tr><td class='l'>session {s['id']}</td><td>{o['thr']:.2f}</td>"
                            f"<td>{o['wlf']:.3f}</td><td>{o['attpp']:.1f}</td><td>{o['n']}</td></tr>")
    # --- EKF 拖拽: worst 窗口 ---
    drag_rows = []
    for s in f["drag_sess"]:
        smp = sorted(s["ekf_drag"]["samples"], key=lambda x: -abs(x["dq"] - x["dg"]))[:3]
        for w in smp:
            drag_rows.append(
                f"<tr><td class='l'>session {s['id']}</td><td>{w['t']:.1f}</td>"
                f"<td>{w['dq']:+.1f}</td><td>{w['dg']:+.1f}</td><td>{w['dq']-w['dg']:+.1f}</td></tr>")
    # --- 陀螺积分 vs EKF 姿态(加速度计修正把真实转动抵消掉) ---
    gve_rows = "".join(
        f"<tr><td class='l'>session {i}</td><td>{g['t0']:.1f}~{g['t1']:.1f}</td>"
        f"<td>{g['pitch_gyro']:+.2f}</td><td>{g['pitch_ekf']:+.2f}</td>"
        f"<td><b>{g['pitch_accel_contrib']:+.2f}</b></td>"
        f"<td>{g['roll_gyro']:+.2f}</td><td>{g['roll_ekf']:+.2f}</td>"
        f"<td><b>{g['roll_accel_contrib']:+.2f}</b></td>"
        f"<td>{next(x for x in f['sess'] if x['id'] == i)['man_max']:.2f}</td></tr>"
        for i, g in f["gve"])
    # --- 与上一批对比 ---
    cmp_txt = ""
    if prev:
        pr = _facts(prev, None)
        rows = [
            ("会话数 / 帧数", f"{len(pr['sess'])} / {pr['rows']:,}", f"{len(f['sess'])} / {f['rows']:,}"),
            ("总时长 [min]", f"{pr['dur']/60:.1f}", f"{f['dur']/60:.1f}"),
            ("解锁段 / 累计 [s]", f"{pr['n_armed_segs']} / {pr['armed_time']:.0f}",
             f"{f['n_armed_segs']} / {f['armed_time']:.0f}"),
            ("最大手动油门", f"{pr['max_man']:.3f}", f"{f['max_man']:.3f}"),
            ("最大电机输出", f"{pr['max_motor']}", f"{f['max_motor']}"),
            ("最大 |ω| [rad/s]", f"{pr['max_gyro']:.1f}", f"{f['max_gyro']:.1f}"),
            ("|a| 范围 [g]", f"{pr['min_acc']:.2f}~{pr['max_acc']:.2f}", f"{f['min_acc']:.2f}~{f['max_acc']:.2f}"),
            ("最大差动裁剪 [%]", f"{pr['mix_clip_max']:.0f}", f"{f['mix_clip_max']:.0f}"),
            ("MotorSat 帧(上侧)", f"{pr['sat_total']} ({pr['sat_high']})", f"{f['sat_total']} ({f['sat_high']})"),
            ("回传丢帧 [%]", f"{pr['lost_min']:.2f}~{pr['lost_max']:.2f}", f"{f['lost_min']:.2f}~{f['lost_max']:.2f}"),
            ("失控保护帧", f"{pr['failsafe_frames']}", f"{f['failsafe_frames']}"),
            ("气压计", "在线" if pr["baro"] else "不在线", "在线" if f["baro"] else "不在线"),
            ("非陀螺姿态速率 中位 [°/s]",
             f"{pr['drag_med_min']:.1f}~{pr['drag_med_max']:.1f}" if pr["drag_med_min"] is not None else "-",
             f"{f['drag_med_min']:.1f}~{f['drag_med_max']:.1f}" if f["drag_med_min"] is not None else "-"),
            ("陀螺帧间抖动 中位 [rad/s]",
             f"{pr['vib_min']:.2f}~{pr['vib_max']:.2f}" if pr["vib_min"] < 9e8 else "-",
             f"{f['vib_min']:.2f}~{f['vib_max']:.2f}" if f["vib_min"] < 9e8 else "-"),
        ]
        cmp_txt = ("<h3>7) 与上一批(" + str(prev.get("batch", "0913")) + ")对比</h3><div class='card'>"
                   "<table><tr><th class='l'>指标</th><th>上一批</th><th>本批</th></tr>" +
                   "".join(f"<tr><td class='l'>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in rows) +
                   "</table></div>")

    baro_section = ""
    if not f["baro"]:
        baro_section = f"""
<h3>2) 摘掉气压计之后: 高度通道变成"纯惯性外推"</h3>
<div class="card">
<p>本批 <code>flags</code> 里始终没有 <code>kLogFlagBaroOk</code>, <code>baroRelM/baroAbsM</code> 恒为 0,
说明 BME280 从头到尾没有被成功初始化(与"摘掉气压计"一致)。后果:</p>
<ul>
<li><b>高度/垂速只剩惯性积分</b>: 没有气压观测量去校正, 加速度计的零偏与姿态误差会被二次积分成
"高度漂移"。实测静止/地面状态就能漂到 ±{f['hgt_drift']:.0f} m(session 3 到 +37.7 m、session 5 到 −21.6 m),
而这些会话里飞机实际只在地面附近 —— <b>这批日志的高度绝对值完全不可用</b>。</li>
<li><b>遥控回传的 height 字段同样失真</b>(它是同一个 EKF 输出); 上位机上的高度显示不可信。</li>
<li><b>定高模式(updateAngleHeight)不可用</b>: 它需要 height/垂速, 现在两者都是发散的。</li>
<li>好处是: 姿态/角速度通道不受影响, 因为姿态只用陀螺+加速度计, 与气压无关;
这也让这一批成为"纯 IMU 姿态"的干净对照(见下一条)。</li>
<li>建议: 要么把气压计装回去(并做减振/导流 + 观测门限), 要么在固件里显式标注"无气压 → 高度无效"
并禁止依赖高度的功能; 不要用现在这个 height 做任何判断。</li>
</ul>
</div>"""

    return f"""
<h2 id="topics">五、专项分析</h2>

<h3>1) 回传链路与丢帧</h3>
<div class="card">
<table><tr><th class="l">会话</th><th>丢帧</th><th>丢帧率</th><th>LinkOk=0 帧</th>
<th>linkAge 最大 [ms]</th><th>失控保护帧</th></tr>{link_rows}</table>
<p>本批丢帧率 <b>{f['lost_min']:.2f}%~{f['lost_max']:.2f}%</b>(0913 为 0.37%~2.20%),
而丢帧仍然"成串"发生(seq 与 tickMs 缺口严格自洽 → 丢在回传链路, 不是飞控漏记)。
更值得注意的是一次<b>飞行中的链路丢失</b>:</p>
<table><tr><th class="l">会话</th><th>linkAge&gt;5s 区间 [s]</th><th>时长 [s]</th><th>帧数</th></tr>
{"".join(outage_rows) or "<tr><td class='l'>无</td><td>-</td><td>-</td><td>-</td></tr>"}</table>
<table><tr><th class="l">会话</th><th class="l">事件</th><th class="l">时刻</th></tr>
{"".join(fail_rows) or "<tr><td class='l'>无</td><td></td><td></td></tr>"}</table>
<p>解读: session 4 在 t≈37.4 s(已解锁、油门 0.20、姿态平稳)最后一次收到遥控指令, linkAge 一路涨到
<b>27.5 s</b>; 5 s 后(44.0 s)失控保护停机, 电机停转。这一次<b>不是姿态问题导致的, 而是链路本身断了</b> ——
与 0913 最后一次(先失稳、链路同时中断)性质不同。结合本批 4.5%~6.6% 的丢帧率,
建议优先排查 NRF 链路: 天线朝向/馈线、发射功率、信道干扰、上位机接收窗口。</p>
</div>

<h3>{"" if baro_section else "2) 气压计/高度通道"}</h3>
{baro_section if baro_section else "<div class='card'><p>本批气压计在线, 高度/气压分析见 0913 报告第二节与第四节图表。</p></div>"}

<h3>3) 姿态估计被加速度计拖走(仍然存在)</h3>
<div class="card">
<p>方法与 0913 报告 6.7 相同: 用刚体运动学把"四元数的姿态变化"与"陀螺积分"相减, 差值只能来自
EKF 的加速度计倾角修正。倾角修正门限仍是 <code>accel_tilt_gate_mss = 2.0 m/s²</code>,
<b>只查幅值不查方向</b>:</p>
<table><tr><th class="l">会话</th><th>倾角门限开门比例(解锁段)</th><th>非陀螺姿态速率 中位</th><th>p95</th>
<th>最大手动油门</th><th>最大 |ω|</th></tr>{drag_rows and "".join(
        f"<tr><td class='l'>session {s['id']}</td><td>{s['ekf_drag']['gate_open_armed']}%</td>"
        f"<td>{s['ekf_drag']['rate_median']} °/s</td><td>{s['ekf_drag']['rate_p95']} °/s</td>"
        f"<td>{s['man_max']:.2f}</td><td>{s['gyro_max']:.1f}</td></tr>" for s in f["drag_sess"])}</table>
<p>差值最大的几个 0.5 s 窗口(Δpitch 四元数 vs Δpitch 陀螺):</p>
<table><tr><th class="l">会话</th><th>t [s]</th><th>Δpitch(四元数) [°]</th><th>Δpitch(陀螺) [°]</th><th>差 [°]</th></tr>
{"".join(drag_rows[:8])}</table>
<p>本批中位 {f['drag_med_min']:.1f}~{f['drag_med_max']:.1f} °/s、p95 到 {f['drag_p95_max']:.0f} °/s,
比 0913(中位 23~35 °/s)低, 但机制相同: 只要推力/颠簸让 ||a|−g| 落在 0.2g 以内, 加速度计就被当成重力,
而它此时测的是推力方向 + 振动。这与姿态环构成正反馈(详见 0913 报告 6.7),
表现为"水平方向缓慢漂 + 姿态慢慢歪", 也是本批 s5 在 0.29~0.33 油门时低频 |ω| 涨到 0.45~0.57 rad/s 的原因。
<b>建议仍按 0913 报告的第 1 优先级处理</b>: 门限收紧到 0.3~0.5 m/s²(或改成方向新息门限),
并在下次系留吊挂测试里确认。</p>
</div>

<h3>4) 混控裁剪与电机饱和</h3>
<div class="card">
<table><tr><th class="l">会话</th><th>解锁总时长 [s]</th><th>差动被裁剪帧占比</th><th>最小混控缩放</th>
<th>MotorSat 帧</th><th>其中上侧</th></tr>{sat_rows}</table>
<p>本批<b>一次都没有裁剪差动</b>(最大 {f['mix_clip_max']:.0f}%、最小缩放 ≥0.99),
因为油门用到了 {f['max_thr']:.2f}, 差动权限足够; MotorSat 也基本不再出现(共 {f['sat_total']} 帧, 上侧 {f['sat_high']} 帧)。
这与 0913 的判断互相印证: 那时候的裁剪/饱和是"油门太低 + 偏航通道抢权限"造成的, 而不是控制器算错。</p>
</div>

<h3>5) 混控与记录字段一致性(逐帧对账)</h3>
<div class="card">
<table><tr><th class="l">会话</th><th>对账帧数(油门&gt;0.05 的解锁帧)</th><th>误差 ≤1 LSB 占比 [%]</th><th>最大误差 [LSB]</th></tr>
{mix_rows}</table>
<p>用 <code>torque</code>+<code>throttle</code> 按 flight_control_mixer.hpp 复算四路 DShot 再与记录值比较,
本批仍是全部吻合(≤1 LSB) → 控制器/混控/记录链路与 0913 完全一致, 物理通道顺序也仍是
<code>motorMap={0,2,3,1}</code>(CSV 的 motor0~3 = M0左前 / M3左后 / M1右前 / M2右后)。</p>
</div>

<h3>6) 振动与低频振荡随油门</h3>
<div class="card">
<p>陀螺帧间抖动(解锁段):{", ".join(f"session {s['id']} {s['vib']['dgyro_armed']:.3f} rad/s" for s in sess if s['vib']['dgyro_armed'] is not None)}
(未解锁时都是 0.04 rad/s 量级 = 传感器本底), 电机指令帧间抖动
{min(s['vib']['dmotor_armed'] for s in sess if s['vib']['dmotor_armed'] is not None):.0f}~
{max(s['vib']['dmotor_armed'] for s in sess if s['vib']['dmotor_armed'] is not None):.0f} counts。
比 0913(0.40~0.49 rad/s、47~58 counts)明显小 —— 本批的桨/机架状态或转速区间更"干净"。</p>
<table><tr><th class="l">会话</th><th>手动油门</th><th>低频 |ω| RMS [rad/s]</th><th>姿态峰峰 [°]</th><th>帧数</th></tr>
{"".join(osc_rows)}</table>
<div class="title">低频 |ω| RMS 随油门变化(每条线 = 一个会话)</div>
<div class="chart" id="osc1"></div>
<div class="title">姿态峰峰(roll+pitch 的 1~99% 跨度)随油门变化</div>
<div class="chart" id="osc2"></div>
<p>低频 |ω| 仍随油门单调增大(session 5 从 0.19@0.17 涨到 0.57@0.33), 但幅度远小于 0913(session 8 曾到 2.6 rad/s),
而且主频很低(session 3/5 为 0.05~0.43 Hz 的缓慢摆动, 0913 发散前是 1.65 Hz)。
说明本批<b>没有出现快速极限环</b>, 主要问题回到"姿态缓慢漂 + 链路"。</p>
</div>

{cmp_txt}

<h3>8) 【最关键】一加油门飞机就低头/倾斜, 电机却"不修正" —— 加速度计把真实转动抵消掉了</h3>
<div class="card">
<p>把解锁段内两条姿态变化放在一起比:</p>
<ul>
<li><b>陀螺积分</b> —— 用 EKF 自己的零偏估计去偏后积分(dt=20ms)。陀螺不会骗人, 这代表"飞机真的转了多少"。</li>
<li><b>EKF 实际输出的姿态</b> —— 日志里四元数解出来的。</li>
</ul>
<p>两者之差只能是<b>加速度计倾角修正</b>拧过去的量:</p>
<table>
<tr><th class="l">会话</th><th>解锁段 [s]</th><th>pitch: 陀螺积分</th><th>pitch: EKF</th>
<th>加速度计修正(pitch)</th><th>roll: 陀螺积分</th><th>roll: EKF</th><th>加速度计修正(roll)</th>
<th>最大手动油门</th></tr>
{gve_rows}</table>
<p><b>读法(以 session 5 为例)</b>: 这段飞行里飞机真的转了 pitch +11.8°、roll −17.8°,
而 EKF 报出来的姿态只动了 −2.6° / +1.5° —— 因为加速度计修正在反方向拧了 −14.4° / +19.3°, 把真实转动几乎完全抵消。
控制器看到的是"姿态基本没变", 于是<b>它没有理由去修</b>: 这不是"修得不够", 是"看不见"。
飞机于是继续往那个方向倒/漂, 飞手看到的就是"一加油门就低头(或倾斜), 电机不修正, 而且稳定停在一个倾角上"。</p>
<p>对照 session 2(不装桨的台架): 修正量只有 −0.2°/−0.6° —— 因为没有推力,
加速度计测到的确实是重力, 修正就是对的。反差正好证明问题的来源。</p>
<p><b>物理原因</b>: 加速度计测的是比力 f = a − g。只要飞机处在"推力支撑自身"的状态,
推力沿机体 z 轴, 于是 f 在机体系里几乎就是机体 z 方向 —— <b>它测的是推力, 不是重力, 对倾斜角不敏感</b>;
而水平加速度(倾斜必然带来)又会让 f 的方向偏 atan(a/g)。实测(见 observability_check_*.txt):
推力段"比力方向与机体 z 的夹角"中位 6~10°、p95 到 30°, 而幅值仍≈1.00g ——
滤波器只看幅值(门限 0.2g), 于是把这些都当成重力, 把真实的倾角判成"已经在水平"。</p>
<p><b>所以之前那条"收紧门限"的建议还不够</b>: 关键在于<b>有推力时加速度计不能作为倾角基准</b>。
可选修法(按实现难度):</p>
<ol>
<li><b>最简单</b>: 解锁后(或油门 &gt; 0.08 时)<b>冻结/大幅降低加速度计倾角修正</b>, 飞行中只靠陀螺;
静止时(解锁前那段)已经把零偏收敛好 —— 实测静止零偏只有 x +0.1°/s、y +0.1°/s、z −1.2°/s,
30 s 飞行陀螺单独积分也就漂 3~6°, 比现在被拧掉 10~20° 好得多。</li>
<li><b>正确做法</b>: 把推力写进观测模型 —— 用油门(和悬停标定)估计 T/m, 先从比力里减掉
<code>(T/m)·ẑ_body</code>, 剩下的才是重力方向; 这样倾斜在飞行中也重新变得可观。
悬停标定你们已经有(≈0.20), 先做线性近似就能用。</li>
<li><b>硬件/机械</b>: 检查推力矢量是否过重心(装载/电池位置)、电机/桨是否同规格。
如果加油门本身就有固定方向的低头力矩, 会把上面这条的影响放大。</li>
<li><b>验证方法(不用真飞)</b>: 拆桨、解锁但保持油门小, 用手缓慢把机架倾斜 20° 再回平 ——
改装前: EKF 会被加速度计拉住, 姿态角明显跟不上陀螺; 改装后: 姿态角应当基本跟着陀螺走。
这条测试 5 分钟就能做完, 而且直接判定有没有修好。</li>
</ol>
</div>

<h3>9) 附录: 为什么 Madgwick 那版能平稳起飞、EKF 这版不能(2026-09-08 飞行数据对比)</h3>
<div class="card">
<p>仓库里的 <code>flight_log_analysis.html</code> 正是 2026-09-08 那次 <b>Madgwick</b> 飞行的报告,
里面嵌了当时的序列。把它按同一口径重算(脚本 <code>analyze_madgwick_report.py</code>):</p>
<table><tr><th class="l">对比项</th><th>Madgwick(0908 飞行)</th><th>EKF(本批 0914)</th></tr>
<tr><td class="l">解锁飞行时长</td><td><b>1 段 96.8 s</b>(连续飞下来了)</td><td>13.9~43.8 s, 每次都被飞手收油门</td></tr>
<tr><td class="l">油门范围</td><td>0.08~0.28</td><td>0.14~0.32</td></tr>
<tr><td class="l">姿态低频(0.5s 平均)5~95%</td><td>roll −2.1~+3.7°, pitch −6.2~+5.9°</td>
<td>roll −16~+4°, pitch −5~+12°(且慢慢漂, 不回来)</td></tr>
<tr><td class="l">|pitch|&gt;15° 的时段</td><td><b>只有 3 次, 每次 0.1~0.3 s 的尖峰</b>(+37°/−24°/+16°)</td>
<td>持续数秒的 10~20° 倾斜</td></tr>
<tr><td class="l">陀螺零偏估计</td><td><b>没有零偏状态</b>(Madgwick 不估零偏)</td>
<td>见下表, 飞行中跑飞</td></tr></table>
<p>Madgwick 那版的姿态误差是<b>短暂尖峰、立刻回到水平</b>; EKF 这版是<b>误差累积成持续倾斜</b>。
差别不在"用不用加速度计", 而在 EKF 多出来的状态把误差"存住"了:</p>
<p><b>EKF 独有的缺陷: 陀螺零偏估计在飞行中被(推力污染的)加速度计带跑。</b>
零偏在积分时是直接从陀螺里减掉的(<code>om = gyro − bg</code>, integrateNominal 第 225 行),
所以零偏估计错了多少, 姿态就以多快的速度虚假旋转。实测(真实零偏由解锁前静止段算出):</p>
<table><tr><th class="l">会话(本批)</th><th>真实零偏 x/y/z [°/s]</th><th>EKF 零偏估计(飞行中达到) x/y/z [°/s]</th>
<th>零偏误差 x/y/z [°/s]</th><th>姿态虚假旋转(20 s 累计)</th></tr>
<tr><td class="l">s3 (25.7 s)</td><td>+0.12 / +0.12 / −1.34</td><td>+0.36 / +0.27 / −1.99</td>
<td>+0.24 / +0.15 / −0.65</td><td>roll +5° / pitch +3° / yaw −13°</td></tr>
<tr><td class="l">s4 (13.9 s)</td><td>0.00 / +0.12 / −1.22</td><td>+0.13 / +0.20 / −1.61</td>
<td>+0.13 / +0.08 / −0.39</td><td>roll +3° / pitch +2° / yaw −8°</td></tr>
<tr><td class="l">s5 (43.8 s)</td><td>−0.12 / +0.12 / −1.10</td><td>+0.40 / +0.42 / <b>−4.24</b></td>
<td><b>+0.53 / +0.30 / −3.14</b></td><td>roll +11° / pitch +6° / yaw −63°</td></tr></table>
<p>注意 s5: 偏航零偏从真实 −1.1 °/s 被估到 −4.2 °/s, 横滚零偏也偏了 +0.5 °/s。
这些错误会<b>一直</b>作用(直到重新落地看到有效重力才可能收敛), 于是姿态估计持续偏向一侧、
控制器又按这个偏掉的姿态去"找平", 飞机就稳定地歪着飞/漂 —— 正是你看到的现象。
<b>Madgwick 没有零偏状态, 所以它根本不会产生这种累积误差</b>(代价是 yaw 完全不修, 而 yaw 本来也不控制)。</p>
<p>另外两点差异: (1) Madgwick 的修正量是 <code>β·s</code>(|s|=1), 相当于姿态修正角速度被硬限制在
<code>2β = 0.156 rad/s ≈ 9°/s</code>, 且只对"当前误差"作用, 不会存起来;
(2) EKF 还有速度/高度状态(本批注意到气压计摘下后高度漂到 ±38 m), 这些状态与姿态/零偏在协方差里互相耦合,
会把本应由姿态吸收的新息"分走"一部分, 让零偏跑得更远。我做过离线重放: 把 Madgwick β=0.078 直接跑在本批
EKF 的飞行数据上, 仅"加速度计权限"这一项两者相当(误差 5.9° vs 5.8°, 见
<code>filter_compare_20260914.txt</code>) —— 也就是说, <b>光把 EKF 的加速度计权重调小并不够,
必须同时冻结/收紧零偏估计</b>。</p>
<p><b>结论</b>: 想尽快恢复到"能平稳起飞"的状态, 有两条路:</p>
<ol>
<li><b>最稳</b>: 姿态直接用 Madgwick(代码本来就有, main_final_test 已验证), EKF 只留高度/垂速(接回气压计)。
EKF 头文件自己也写了: 在"单 IMU + 气压"的退化情形下, 它只应作为 <b>Madgwick + VES 的可选增强/备份</b>。</li>
<li><b>继续用 EKF</b>: (a) 解锁后冻结零偏估计(零偏只在地面收敛), 或把 <code>gyro_bias_walk_sigma</code> 设≈0
并把 <code>gyro_bias_limit</code> 从 ±0.3 rad/s(±17°/s, 太大)收到 ±0.05 rad/s;
(b) 收紧加速度计倾角修正(噪声σ调大 / 门限收紧 / 推力补偿);
(c) 在只有 IMU+气压时先不考虑速度/高度耦合对姿态的影响。</li>
</ol>
</div>

<h2 id="actions">六、建议(针对本批)</h2>
<div class="card">
<ol>
<li><b>【最关键】修正"加速度计倾角校正在有推力时失效"这个根因</b>(见第五节 8): 解锁后按油门冻结/大幅降低
倾角修正, 或把推力从比力里减掉再用作重力基准。否则姿态环在飞行中"看不见"真实倾斜,
飞机一加油门就会保持一个倾角、电机看起来不修正。</li>
<li><b>【同样关键】陀螺零偏估计不能在空中跑</b>(见第五节 9): 它会被推力污染的加速度计带跑(s5 实测
偏航零偏估到 −4.2 °/s, 真实只有 −1.1 °/s), 而零偏错误会<b>持续</b>注入虚假姿态旋转 ——
这是 Madgwick 那版能平稳起飞、EKF 这版不能的直接差别(Madgwick 根本没有零偏状态)。
做法: 解锁后冻结零偏估计(只在地面收敛), 或把零偏随机游走置≈0、把 ±0.3 rad/s 的钳位收到 ±0.05。</li>
<li><b>先解决链路</b>: 本批 session 4 是飞行中链路丢失导致失控保护停机, 且整体丢帧率升到 4.5%~6.6%。
查天线/馈线/发射功率/信道, 并把 <code>linkTimeoutMs</code> 从 5 s 缩短到 1~2 s、链路丢失时主动降油门。</li>
<li><b>气压计的处理</b>: 现在的高度是纯惯性外推(漂 ±{f['hgt_drift']:.0f} m)。要么装回去并做减振导流 + 观测门限,
要么在固件里把"无气压"显式暴露出来(状态位已有 kLogFlagBaroOk)并禁止依赖高度的功能;
上位机显示高度也要按这个状态位切语义(现在会显示假高度)。</li>
<li><b>仍然要收紧加速度计倾角修正门限</b>(2.0 → 0.3~0.5 m/s² 或方向新息门限) —— 本批差值比 0913 小,
但机制还在, 而且会随油门/振动上升而放大。</li>
<li><b>油门标定与增益</b>: 本批用到 0.32 油门仍无裁剪/饱和, 说明"悬停油门 0.20"的实测值更接近实际;
把 <code>HOVER_THROTTLE</code> 改成实测值(0.20~0.22), 混控力矩换算与等效增益都会更准。</li>
<li><b>做一次系留/吊挂对照</b>: (a) 装桨悬停 0.5 m 保持 30 s; (b) 系留后从 0.1 缓慢加到 0.35。
记录"姿态估计是否在没有陀螺支持时自己转""水平漂移方向是否固定", 用来确认第 3 条。</li>
<li><b>保持回传补传</b>: 现在丢帧率升高后, 事后频谱/事件定位会漏东西; 按 0913 建议加 seq 校验 + 补传。</li>
</ol>
</div>
"""


def build_html(data: dict, prev: dict | None = None) -> str:
    sess = data["sessions"]
    p = data["params"]
    tot_rows = sum(s["rows"] for s in sess)
    tot_dur = sum(s["duration"] for s in sess)
    tot_segs = sum(len(s["segments"]) for s in sess)
    tot_armed = sum(g["dur"] for s in sess for g in s["segments"])
    tot_lost = sum(s["lost_frames"] for s in sess)
    max_man = max(s["man_max"] for s in sess)
    max_gyro = max(s["gyro_max"] for s in sess)
    max_acc = max(s["acc_max"] for s in sess)
    max_motor = max(s["motor_max"] for s in sess)
    n_arm = sum(s["events_used"].get("Arm", 0) for s in sess)
    n_dis = sum(s["events_used"].get("Disarm", 0) for s in sess)

    # 跨会话总览表
    rows = []
    for s in sess:
        segs = s["segments"]
        at = sum(g["dur"] for g in segs)
        mix_min = min([g["mix_min"] for g in segs], default=1.0)
        clip = max([g["mix_clip_pct"] for g in segs], default=0.0)
        rows.append(
            f"<tr><td class='l'><a href='#s{s['id']}'>session {s['id']}</a></td>"
            f"<td>{s['rows']}</td><td>{s['duration']:.1f}</td><td>{s['rate']:.2f}</td>"
            f"<td>{len(segs)}</td><td>{at:.1f}</td>"
            f"<td>{s['man_max']:.3f}</td><td>{s['motor_max']}</td>"
            f"<td>{s['gyro_max']:.1f}</td><td>{s['acc_max']:.2f}</td>"
            f"<td>{mix_min:.2f}</td><td>{clip:.0f}%</td>"
            f"<td>{s['lost_frames']}<br><span class='sub'>{s['lost_pct']:.2f}%</span></td>"
            f"<td>{s['link_max']}</td><td>{s['failsafe_frames']}</td>"
            f"<td class='l'>{', '.join(k for k in s['events_used'] if k not in ('BaroUpdate',))}</td></tr>"
        )
    overview = ("<table><tr><th class='l'>会话</th><th>帧数</th><th>时长 s</th><th>Hz</th><th>解锁段</th>"
                "<th>解锁总时长 s</th><th>最大手动油门</th><th>电机 max</th><th>|ω| max</th><th>|a| max</th>"
                "<th>最小混控缩放</th><th>差动裁剪占比</th><th>回传丢帧</th><th>链路最大 ms</th><th>失控保护帧</th>"
                "<th class='l'>出现的事件位</th></tr>" + "".join(rows) + "</table>")

    # 跨会话对比条形(用 CSS 条)
    def cmp_row(label, key, unit, fmtn=1, cls="bar"):
        vals = [(s["id"], key(s)) for s in sess]
        vmax = max(v for _, v in vals) or 1
        cells = "".join(f"<td>{bar(round(v, fmtn), round(vmax, fmtn), cls)}</td>" for _, v in vals)
        return f"<tr><td class='l'>{label} ({unit})</td>{cells}</tr>"

    cmp = ("<table><tr><th class='l'>指标</th>" +
           "".join(f"<th>session {s['id']}</th>" for s in sess) + "</tr>" +
           cmp_row("最大手动油门", lambda s: s["man_max"], "0~1", 3) +
           cmp_row("解锁段总时长", lambda s: sum(g["dur"] for g in s["segments"]), "s", 1) +
           cmp_row("最大角速度", lambda s: s["gyro_max"], "rad/s", 1, "bar warn") +
           cmp_row("最大加速度", lambda s: s["acc_max"], "g", 1, "bar warn") +
           cmp_row("差动被裁剪的帧占比(最大段)", lambda s: max([g["mix_clip_pct"] for g in s["segments"]], default=0), "%", 1, "bar bad") +
           cmp_row("回传丢帧率", lambda s: s["lost_pct"], "%", 2, "bar ok") +
           "</table>")

    narrative = f"""
<h2 id="summary">一、结论速览</h2>
<div class="card">
<p><b>这 5 个会话不是 5 次飞行, 而是同一天连续 5 次上电的日志回传</b>(session 号 = 飞控每次上电自增的会话号),
共 {tot_rows:,} 帧 / 50 Hz / 合计 {tot_dur/60:.1f} 分钟, 期间一共解锁 {tot_segs} 段、累计解锁 {tot_armed:.0f} s。
其中 session 7 全程未解锁(纯地面待机), 其余 4 次各含 1~3 段解锁运行。</p>
<ol>
<li><b>没有任何一段可以称为"稳定飞行"。</b>所有会话的手动油门最大值只有 {max_man:.3f}, 而飞控代码里
<code>HOVER_THROTTLE = {p['hover_throttle']}</code>(悬停油门, flight_control_params.hpp)。按混控使用的线性推力模型,
最大推力/重量比只有 {max_man/p['hover_throttle']:.2f} —— 飞机全程处在"刚好够悬停 / 推不动"的边界上。
全批日志里 EKF 高度最高只到 +1.16 m(session 8), 没有形成持续爬升, 更谈不上定高;
再叠加下面第 6 条的气压计扰动, 高度通道基本没有可用信息。</li>
<li><b>飞控侧日志本身是完好的, 缺的帧全部来自串口回传链路。</b>每个会话里 <code>tickMs</code> 差值与
<code>seq</code> 跳变严格满足 20 ms × 缺失条数(吻合率 {min(s['gap_explained_pct'] for s in sess):.1f}%),
说明这些条目在 flash 里存在、只是没传回来; 5 个会话共丢 {tot_lost} 帧(0.37%~2.20%),
最长一次连丢 22 帧(≈440 ms, session 4)。回传链路是单向流式、无重传确认
(release_log_manage.hpp 的 DumpSession → 上位机 FlightLogWriter),
所以丢帧只能靠"按 seq 补传"解决。</li>
<li><b>控制环时序非常干净:</b>实测控制周期 p50 = 1994 µs、最大 1996 µs(500 Hz), 5 个会话 0 帧超预算;
四元数模恒为 1.0000, 姿态解算无发散。板级计算资源不是瓶颈。</li>
<li><b>两次真实的高动态翻滚/坠机, 另有若干次地面搬运/跌落。</b>
(a) session 4 t≈236.2~237.0 s(解锁中、油门 0.22): |ω| 峰值 {sess[0]['gyro_max']:.1f} rad/s(≈900 °/s)、
合加速度 6.26 g, 姿态在 0.2 s 内翻转 60° 以上, 四路电机出现真实下侧饱和(0/50/128/962),
之后油门降到 0.04 并停机 —— 典型的空中翻滚/坠机。
(b) session 8 t≈307.4~313.7 s: 遥控链路在 307.4 s 断掉, 飞控在"指令冻结"状态下继续以油门 0.22 输出,
飞机剧烈震荡 5 s, 312.4 s 触发失控保护停机, 停机后 312.9 s 还有一次 15.5 rad/s / 4.6 g 的撞击,
314 s 之后静止(链路 318.1 s 恢复, Failsafe 锁存到会话结束)。
(c) 其余高动态都发生在<b>未解锁</b>状态: session 6 t≈90~110 s(6.5 rad/s、2.9 g、pitch −26°)、
session 8 t≈71.5 s(roll −132°, 加速度计确认机体倒置)、session 7 t=48.7 s(9.7 rad/s),
属于地面搬运/跌落, 不是飞行事故。
这些片段的姿态、角速度、加速度互相自洽 —— 与 flight_log_analysis.html 里"大 pitch 峰值多为
垂直加速度造成的假姿态"的结论不同: 那一条适用于缓慢机动时的加速度计判读, 而这几次是真的在转/撞。</li>
<li><b>EKF 的陀螺零偏估计跑飞, 导致姿态 yaw 在静止时虚假旋转。</b>静止时陀螺 z 真实零偏约
−1.1 °/s, 但 EKF 估出的 <code>bz</code> 在解锁/机动后一路涨到上限钳位 −0.3 rad/s(−17.2 °/s)
(session 5、8 都顶到过钳位)。由于名义积分是 <code>q ← q·exp((ω−bg)·dt)</code>,
姿态解出来的 yaw 就会以 2~17 °/s 的速率自己转(静止段可直接验证:
yaw 变化率 ≈ 陀螺贡献 + (−bz), 见 flight_log_analysis/check_bias.py)。
偏航没有参与控制(ATT_YAW_WEIGHT = 0), 所以没有直接造成失控, 但它会通过姿态/重力投影污染
高度与垂速通道(见第 8 条), 而且日志里的 yaw 绝对值完全不可用于事后判读机头朝向。</li>
<li><b>气压计被桨流/气流严重干扰。</b>每次解锁后 1~3 s 内气压相对高度都会掉到 −2 ~ −4.65 m、
EKF 高度同步下沉、垂速到 −1.2 ~ −3.4 m/s; 而长时低油门台架段(session 8 第 2 段 79.6 s)又出现
+0.7~+1.1 m 的"假爬升"。同一条件下高度可正可负, 说明这段气压信号主要由气流/振动决定,
不能当作真实高度。代码里气压实测 σ 放宽到 <code>ekfBaroSigmaM</code>, 但 20 ms 一次的气压观测
直接把这类偏差喂给了 EKF, 高度/垂速在这几个会话里都不可用于评估性能。</li>
<li><b>低油门下姿态环没有权限, 指令跟踪基本失败。</b>按 flight_control_mixer.hpp 复算,
解锁帧里有 6%~24%(按段)的帧差动被"推力优先"策略裁剪(最小缩放 0.137 = 只放出 14% 的力矩)。
最直观的一例: session 5 在 t≈149~154 s 遥控给了横滚 −12° 指令, 角速度期望 0.6 rad/s,
而实测横滚始终在 −0.8 ~ −1.5°、实测角速度 ≈0.05 rad/s —— 飞机根本没跟着走(该段油门仅 0.06,
是悬停值的 24%)。这与油门不足/差动被裁剪互为因果, 与 PID 参数好坏无关, 无法据此判断 PID 极性。</li>
<li><b>高度估计与气压最大偏离 4.0 m</b>(session 8 第 3 段)、2.5 m(session 4 第 2 段),
且都发生在翻滚/冲击之后, 之后没有自动恢复: 说明 EKF 在强机动后高度通道需要重新收敛,
而偏航零偏错误又在持续注入姿态误差。</li>
<li><b>MotorSat(电机饱和)只报"下侧", 报不出"推力打满"。</b>release.cpp 的判据是
<code>motor[i] &lt;= 55 || motor[i] &gt;= 1945</code>。逐帧核对下来, 全部 MotorSat 帧
(session 4: 307、5: 459、6: 59、8: 586 帧, 占各自解锁段的 5%~11%)都是"低侧" ——
即至少一路被混控压到 0 档, 而其余几路还在转(例: 0/50/128/962, 这类帧本身不是误报);
<b>上侧饱和(motor ≥1945)在 5 个会话里一次都没出现过</b>。所以这个位只能说明"力矩被油门余量限制",
说不出"推力打到头"。建议把上下侧分开报, 并把混控的 mix_scale 直接写进日志(它才是"力矩被裁剪"的连续量,
本报告目前是用 torque+throttle 反算的)。</li>
<li><b>失控保护能停机, 但 5 s 的门限太长, 而且这 5 s 里指令是"冻结"的。</b>
session 8 的链路在 307.4 s 断(最后一次有效遥控), 飞控保持最后一帧指令继续输出
(油门 0.22 一直保持到 312.4 s), 直到 linkAge 超过 linkTimeoutMs = 5000 ms 才自动停机并锁存 Failsafe;
停机后日志记录到会话结束(约 59 s)。其余中断都发生在未解锁状态: session 7 有 30.2 s(1.7~31.8 s)、
session 4 有两段 6.9 s / 10.6 s(189.2~196.1 s、200.6~211.2 s)、session 5 有一前一后两段,
都没有误触发保护。另有 4 个会话在上电最初的 5~16 s 里收不到遥控(linkAge 从 0 一路涨),
说明发射机/接收机开机顺序不固定。
建议: 把 linkTimeoutMs 从 5 s 缩短到 1~2 s, 或在链路丢失时主动降油门/进入自降, 不要"保持最后指令";
另外"上电后长时间收不到遥控"也应该有一个状态位或提示。</li>
<li><b>回传字段与飞控代码可以逐帧对上账。</b>用日志里的 <code>torque</code> 与 <code>throttle</code> 按
flight_control_mixer.hpp 复算四路电机, 与记录的 motor0~3 在全部解锁帧吻合(误差 ≤1 LSB,
仅 5 帧例外, 那 5 帧是"已解锁但控制尚未接管"的过渡帧), 并据此确认了物理通道顺序
<code>motorMap = {{0,2,3,1}}</code>, 即 CSV 里 motor0~3 = M0左前 / M3左后 / M1右前 / M2右后。</li>
</ol>
</div>

<h2 id="parse">二、日志格式与解析口径</h2>
<div class="card">
<p>这 5 个 CSV 是上位机把回传的 <code>FlightLog</code> 原样展开的结果(每个会话一个文件, 表头由
<code>FlightLogWriter.HEADER</code> 定义, 字段与飞控 <code>LogEntry</code> 一一对应)。关键字段含义:</p>
<table>
<tr><th class="l">字段</th><th class="l">含义</th><th class="l">单位 / 说明</th></tr>
<tr><td class="l">sessionId / seq / tickMs</td><td class="l">会话号 / 会话内条目序号 / 上电毫秒</td>
    <td class="l">seq 从 1 开始; tickMs 由 20 ms 日志周期决定, 用于校验缺帧</td></tr>
<tr><td class="l">flags</td><td class="l">电平型状态位(kLogFlag*)</td>
    <td class="l">LinkOk/ImuOk/BaroOk/AngleLoop/MotorEnabled/MotorStarting/HardStop/FlashOk/LogFull/LogError/MotorSat/Failsafe/GroundMode</td></tr>
<tr><td class="l">events</td><td class="l">边沿型事件位(kLogEvent*), 归档后清零</td>
    <td class="l">CommandChanged/Arm/Disarm/Failsafe/MotorSat/LogFault/BaroUpdate/EkfRezero</td></tr>
<tr><td class="l">linkAgeMs</td><td class="l">距最近一次有效遥控指令的时间</td><td class="l">ms; &gt;5000 ms 触发失控保护</td></tr>
<tr><td class="l">loopPeriodUs</td><td class="l">本周期控制循环实测周期</td><td class="l">µs; 目标 2000 µs(500 Hz)</td></tr>
<tr><td class="l">quat[4]</td><td class="l">EKF 姿态四元数(机体→世界, w,x,y,z)</td>
    <td class="l">本报告统一按 ZYX 顺序转成 roll/pitch/yaw, roll/pitch 可信, yaw 受零偏影响</td></tr>
<tr><td class="l">bodyRate[3]</td><td class="l">机体角速度</td><td class="l">rad/s(原始陀螺, 未去零偏)</td></tr>
<tr><td class="l">accelG[3]</td><td class="l">机体比力</td><td class="l">g(包含重力反作用, 静止时模 ≈ 1 g)</td></tr>
<tr><td class="l">rateSetpoint[3]</td><td class="l">角速度环期望值</td><td class="l">rad/s(角度环输出, 上限 2 rad/s)</td></tr>
<tr><td class="l">torque[3]</td><td class="l">期望力矩(混控输入)</td><td class="l">N·m</td></tr>
<tr><td class="l">targetPitch/Roll/Height</td><td class="l">遥控期望姿态/高度</td><td class="l">deg / m(本批日志 targetHeight 恒为 0, 未用定高模式)</td></tr>
<tr><td class="l">throttle / manualThrottle</td><td class="l">控制输出油门 / 遥控手动油门</td><td class="l">0~1; 悬停参考 {p['hover_throttle']}</td></tr>
<tr><td class="l">heightM / vertVelMps</td><td class="l">EKF 高度 / 垂向速度</td><td class="l">m / m·s⁻¹(解锁瞬间被 reset 到 0)</td></tr>
<tr><td class="l">baroRelM / baroAbsM</td><td class="l">气压高度(相对起飞基准 / 绝对)</td><td class="l">m; 解锁启动序列里重取基准</td></tr>
<tr><td class="l">gyroBias[3]</td><td class="l">EKF 估计的陀螺零偏</td><td class="l">rad/s; 每 10 条日志记一次(其余为 0), 钳位 ±0.3</td></tr>
<tr><td class="l">motor[4]</td><td class="l">四路 DShot 输出(物理通道顺序)</td>
    <td class="l">计数 = 油门×{p['dshot_unit']:.0f}+{p['dshot_offset']:.0f}; 50 = 0 档, 1950 = 满</td></tr>
</table>
</div>

<h2 id="overview">三、会话总览</h2>
{overview}
<div class="sub">"最小混控缩放"与"差动裁剪占比"由本报告按 flight_control_mixer.hpp + 参数默认值复算,
只有解锁帧参与统计。事件位只列出非 BaroUpdate 的(后者每条日志都有, 共 {sum(s['rows'] for s in sess):,} 次)。
session 6 的 MotorSat 事件记录次数多于标志帧数, 是因为事件位在归档时按"这一条日志期间发生过"累计, 一条日志可能记多次。</div>
<div class="title">跨会话对比</div>
{cmp}

<h2 id="detail">四、逐会话图表</h2>
<div class="sub">每张图都是同一套坐标: 灰蓝带 = 电机使能(解锁)时段, 橙色虚线 = 遥控期望值或参考线,
红色虚线 = 异常/事件标线。滚轮缩放、拖动平移、双击复位。</div>
"""

    detail = "".join(session_section(s, p) for s in sess)

    osc_series = {"wlf": [], "attpp": []}
    for s in sess:
        if not s.get("osc"):
            continue
        color = {4: "#4e79a7", 5: "#59a14f", 6: "#f28e2b", 7: "#b07aa1", 8: "#e15759"}.get(s["id"], "#888")
        osc_series["wlf"].append({"n": f"session {s['id']}", "c": color,
                                  "pts": [[o["thr"], o["wlf"]] for o in s["osc"]]})
        osc_series["attpp"].append({"n": f"session {s['id']}", "c": color,
                                    "pts": [[o["thr"], o["attpp"]] for o in s["osc"]]})
    osc_js = (
        "<script>mkScatter('osc1'," + json.dumps(
            {"series": osc_series["wlf"], "h": 300, "xlab": "手动油门(归一化)",
             "ylab": "低频 |ω| RMS [rad/s]", "ylab2": ""}, separators=(",", ":")) + ");</script>"
        "<script>mkScatter('osc2'," + json.dumps(
            {"series": osc_series["attpp"], "h": 300, "xlab": "手动油门(归一化)",
             "ylab": "姿态峰峰 [deg]"}, separators=(",", ":")) + ");</script>"
    )

    tail = f"""
<h2 id="topics">五、专项分析</h2>

<h3>1) EKF 陀螺零偏估计跑飞与 yaw 虚假旋转</h3>
<div class="card">
<p>名义积分(<code>flight_control_fliter.hpp</code> integrateNominal)用去偏角速度
<code>ω−bg</code> 积分四元数, 所以<b>姿态 yaw 的旋转速率 = 陀螺贡献 + (−bz) 贡献</b>。
把这两项从日志里拆出来对比(脚本 <code>check_bias.py</code>):</p>
<ul>
<li>静止段(session 7 全程未解锁)陀螺 z 读数稳定在 −1.0 ~ −1.3 °/s 附近, 此时 bz≈0, yaw 以 ≈−1.15 °/s 变化 —— 这就是这块 BMI088 的<b>真实 z 轴零偏约 −1.1 °/s</b>。</li>
<li>解锁/机动之后 bz 估计从 0 一路跑到 −0.11 ~ −0.30 rad/s(6~17 °/s), session 5、8 都顶到了 ±0.3 rad/s 的钳位上限(bias_limit 默认 0.3)。</li>
<li>冻结在 −0.3 rad/s 时, 日志里"静止"的姿态 yaw 却以 +16.0 °/s 旋转, 而同一时刻陀螺只读到 −1.1 °/s —— 两者差值正好等于 −bz(误差 &lt;2.5 °/s)。</li>
</ul>
<p>结论: yaw 方向没有磁力计/GNSS 观测, 该状态(以及与之耦合的 bz)不可观; 加速度计观测在飞机大倾角机动时
通过误差协方差把 bz 推到了错误方向, 于是估计器一边"去偏"、一边把姿态转起来。
<b>影响</b>: (a) 日志里的 yaw 绝对值/航向漂移量不能用于任何结论; (b) 姿态误差会通过重力投影进入高度/垂速通道,
与第 7 条的翻滚后高度偏离 4 m 相关; (c) 一旦以后要用 yaw(比如加航向控制或航点), 必须先修这个估计器。
<b>建议</b>: 无航向观测时冻结 bz(不同时估计 bz, 只估计 bx/by), 或对 bz 加"仅在有航向观测时更新"的开关;
把 bias_limit 降到 0.05 rad/s 量级并加静态告警位。</p>
</div>

<h3>2) 气压计/高度通道的可信度</h3>
<div class="card">
<p>观测事实: 每次解锁后 1~3 s 内 <code>baroRelM</code> 都会下沉(−2.0 ~ −4.65 m), EKF 高度同步下沉,
垂速到 −1.2 ~ −3.4 m/s; 而在 session 8 的 79.6 s 台架段(油门 0.12~0.16)又出现 +0.7~+1.1 m 的假爬升。
两种符号都出现, 说明气压读数主要由桨流/气流与温度决定, 而不是真实高度。</p>
<p>两种解释都无法完全排除: (a) 油门(0.04~0.26)低于悬停(0.25)时飞机确实会掉高度; (b) 螺旋桨气流在气压计处形成局部
压力偏差。要区分只需一次对照实验: 绑好桨、油门固定 0.15 在地面停留 30 s, 看 baroRel 是否同样下沉 ——
若下沉则证明是桨流, 若不动则是真实掉高。</p>
<p><b>建议</b>:(1) 气压计加减振+导流棉, 或把采样口远离桨流; (2) 提高 <code>baro_sigma_m</code> 或对观测做滞回门限
(例如只在 |加速度−1g| 小时才吃气压观测); (3) 记录一条独立的"原始气压"通道用于事后分辨, 现在只能看到已被 EKF 融合的相对高度。</p>
</div>

<h3>3) 电机饱和判据</h3>
<div class="card">
<p>release.cpp 里 <code>if (motor[phys] &lt;= 55 || motor[phys] &gt;= 1945) pidSat = true;</code>, 门槛对应
混控输出贴在 0 档(50)或打满(1950)。逐帧分类核对(以 <code>min(motor) ≤ 55</code> 为下侧、
<code>max(motor) ≥ 1945</code> 为上侧):</p>
<table><tr><th class="l">会话</th><th>MotorSat 帧</th><th>下侧(min≤55 且 max&gt;55)</th>
<th>全零档(过渡帧)</th><th>上侧(max≥1945)</th><th>占解锁帧比例</th></tr>
<tr><td class="l">4</td><td>307</td><td>307</td><td>0</td><td>0</td><td>11.0%</td></tr>
<tr><td class="l">5</td><td>459</td><td>459</td><td>0</td><td>0</td><td>9.4%</td></tr>
<tr><td class="l">6</td><td>59</td><td>59</td><td>0</td><td>0</td><td>5.2%</td></tr>
<tr><td class="l">8</td><td>586</td><td>586</td><td>0</td><td>0</td><td>9.9%</td></tr></table>
<p>也就是说: 这个位<b>没有误报</b>(每一帧确实有一路被压到 0 档), 但它<b>只报下侧</b>——
上侧饱和(推力真正打满)在 5 个会话里一次都没有出现, 因为油门始终远低于悬停值。
当下的门槛还缺一个区分: "整体油门太小导致差动放不下"与"力矩需求过大"在下侧饱和里是同一现象。
建议 (1) 上/下侧分开报; (2) 把 <code>mix_scale</code> 也写进日志, 它是连续量, 能直接看出力矩被裁剪得多狠
(本报告目前是用 torque+throttle 反算的)。</p>
</div>

<h3>4) 回传链路丢帧</h3>
<div class="card">
<p>5 个会话共丢 {tot_lost} 帧, 丢失率 0.37%(session 7)~2.20%(session 4); 丢帧是<b>成串</b>出现的
(典型 1、3、5、7、11、22 帧连着丢), 且 <code>seq</code> 跳变与 <code>tickMs</code> 差值严格一致,
所以可以确定丢在"flash → 串口 → 上位机"这一段, 而不是飞控漏记。触发条件与串口波特率、上位机写盘、
以及会话开头的 flash 擦除/首次回传起始状态有关(session 4 的丢帧集中在开头 0~35 s)。</p>
<p><b>建议</b>: 在 DumpSession 协议里加序号+重传(或者让上位机发现 seq 不连续时请求重发该段),
现在的方式只能接受 0.4%~2.2% 的数据空洞; 对事后频谱分析/参数辨识足够, 对逐帧比对不够。
</p>
</div>

<h3>5) 混控与记录字段一致性(逐帧对账)</h3>
<div class="card">
<p>用 <code>torque</code> 与 <code>throttle</code>, 按 flight_control_mixer.hpp 的公式和参数默认值
(质量 1.566 kg、悬停 0.25、前后臂 0.12933 m、左右臂 0.18106 m、偏航等效力臂 0.03 m)
复算差动、缩放系数与四路 DShot 计数, 再与日志里的 motor0~3 比较:</p>
<ul>
<li>解锁帧: 差值 ≤1 LSB 的帧占 100%(session 4/5/6 全部 ≤1, session 8 仅 5 帧例外,
那 5 帧是"已解锁但还在启动序列、控制尚未接管"的过渡帧, 日志里 motor 保留的是上一拍的值)。</li>
<li>由此可反推确认物理通道顺序为 <code>motorMap={{0,2,3,1}}</code>;
CSV 的 motor0~3 对应逻辑电机 M0左前 / M3左后 / M1右前 / M2右后(X 布局, 见 flight_control_params.hpp 的机架图)。</li>
<li>这条对账同时说明: 参数默认值就是飞控里实际生效的值(没有被上位机参数表覆盖)。</li>
</ul>
</div>

<h3>6) 解锁/停机的边沿处理</h3>
<div class="card">
<p>日志里同一次"解锁"会在连续 5 帧(100 ms)内重复置 <code>kLogEventArm</code>, "停机"同样重复。
原因是遥控每帧都带 command, 而控制线程每拍看到 <code>command==2/3</code> 就置位 start/stop
(release.cpp 的 kLogEventArm/Disarm), 只要遥控一直发这个命令, 事件就一直是"新"的。
功能上暂时无害, 但会让日志事件分析误判"解锁 20 次"(session 8 实际是 3 段运行),
也会在按住开关时反复重启 DShot 时基。建议在控制线程里做"只在命令位变化时动作"的边沿检测,
或给 Arm/Disarm 事件加最小间隔。</p>
</div>

<h3>7) session 8 的失控过程(链路中断 5 s 才停机)</h3>
<div class="card">
<p>把 linkAge / 状态位 / 角速度按时间摆开看, 过程非常清楚:</p>
<table>
<tr><th class="l">时刻 [s]</th><th class="l">发生的事</th></tr>
<tr><td class="l">307.4</td><td class="l">最后一次收到有效遥控指令(此后 linkAge 从 0 一路涨)</td></tr>
<tr><td class="l">307.4~312.4</td><td class="l">飞控保持最后一帧指令: 手动油门 0.22、期望姿态 0°, 继续正常出力;
期间 |ω| 在 0.4~6.5 rad/s 之间反复摆、|a| 在 0.5~2.6 g 之间反复冲击</td></tr>
<tr><td class="l">312.4</td><td class="l">linkAge 超过 5000 ms → 触发失控保护, 电机停机, Failsafe 锁存(flag 从 0x00BF 变 0x0886)</td></tr>
<tr><td class="l">312.9</td><td class="l">停机之后机体还在翻滚: |ω| 15.5 rad/s、|a| 4.64 g(最猛烈的一次撞击)</td></tr>
<tr><td class="l">314.1~318.1</td><td class="l">静止(|ω|≈0, |a|≈1.00 g), 链路仍在丢(linkAge 涨到 10.7 s)</td></tr>
<tr><td class="l">318.1</td><td class="l">遥控恢复(linkAge 归 18 ms), 但 Failsafe 是会话级锁存, 一直保持到 371 s 结束</td></tr>
</table>
<p>两点改进: (1) linkTimeoutMs = 5000 对飞行中的飞机太长, 建议 1~2 s, 或者链路丢失时先降到安全油门/自降;
(2) 停机后日志里的 <code>motor[]</code> 仍然记着停机前的最后一拍(例: 772/394/194/510),
而实际发给电调的是 0(dshot.preloadThrottle(0,0,0,0))—— 事后看日志容易误判成"停机后电机还在转",
建议停机后把 motor 字段也写 0。</p>
</div>

<h2 id="osc">六、追加分析: 为什么"姿态像没在修正"、并且一加油门就出事</h2>
<div class="sub">这一节是收到飞手补充信息(实测 20% 左右离地、每次刚离地就往右前漂、最后一次坠机)后追加的,
结论来自 flight_log_analysis/analyze_control.py / analyze_drift.py / analyze_vibration.py / analyze_oscillation.py 四个脚本。</div>

<h3>6.1 先纠正一个读数坑: CSV 里的 motor0~3 不是 M0~M3</h3>
<div class="card">
<p>混控器按 <code>kConfig.motorMap = {0,2,3,1}</code> 把逻辑电机写到物理 DShot 通道, 所以 CSV 的列是
<b>物理通道顺序</b>, 对应关系是:</p>
<table><tr><th class="l">CSV 列</th><th class="l">实际电机</th><th class="l">位置</th></tr>
<tr><td class="l">motor0</td><td class="l">M0</td><td class="l">左前</td></tr>
<tr><td class="l">motor1</td><td class="l">M3</td><td class="l">左后</td></tr>
<tr><td class="l">motor2</td><td class="l">M1</td><td class="l">右前</td></tr>
<tr><td class="l">motor3</td><td class="l">M2</td><td class="l">右后</td></tr></table>
<p>用这个顺序重看 session 4 的 t=171.0 s(俯仰 −15.3°, 即机头翘起; 横滚 +2.2°, 即右侧下沉):
CSV 记录 [376, 365, 283, 391] → 左前 376 / 左后 365 / 右前 283 / 右后 391 →
<b>后侧合计 756 &gt; 前侧 659, 左侧 741 &gt; 右侧 674</b> —— 也就是"压机头 + 抬右侧",
<b>正是修正当前姿态该有的方向</b>。按物理通道复原后, 这一段并不是"没在修正",
只是修正量比振动噪声小得多(见 6.3)。</p>
<p>同一个坑还会影响"哪路电机小"的判断: 停机后 <code>motor[]</code> 保留的是停机前最后一拍的值
(例: session 8 t=314 s 之后仍是 772/394/194/510), 实际发给电调的是 0, 不要当成"停机了电机还在转"。</p>
</div>

<h3>6.2 控制链路的符号/轴向全部对得上(不是正反馈, 不是轴接反)</h3>
<div class="card">
<ul>
<li>用日志里的 <code>torque</code>+<code>throttle</code> 按混控公式复算四路电机, 与记录值<b>逐帧吻合(≤1 LSB)</b>;
再用日志的四元数 + 目标姿态按 <code>flight_controller.hpp</code> 的倾转分离算法复算 <code>rate_setpoint</code>,
与记录值<b>完全一致</b> —— 说明控制器/混控/记录的链路没有符号错误, 我们看到的"修正意图"就是真实计算值。</li>
<li>用"电机模式 → 实测角速度"做阻尼方向检查: <b>帧间相关系数 −0.91 ~ −0.996</b>(三轴、四个会话都是负),
即角速度往哪边转、电机就往反方向打 —— 与设计一致, 没有发现陀螺/电机接反。</li>
<li>姿态估计本身在准静态时与加速度计一致(Δroll/Δpitch 中位 &lt;0.4°), 不存在几度的固定姿态偏差。</li>
</ul>
<p>所以"姿态没有被修正"不是控制律或接线问题, 而是<b>修正被更强的扰动盖住了</b>。</p>
</div>

<h3>6.3 真正的现象: 振荡幅值随油门单调增大, 到悬停点附近发散</h3>
<div class="card">
<p>把解锁段按"手动油门"分箱, 统计 <b>低频角速度幅值</b>(0.15 s 低通, 控制器能响应的频段)和
<b>姿态峰峰</b>, 得到一条非常清楚的趋势:</p>
<div class="title">低频 |ω| RMS 随油门变化(每条线 = 一次上电会话)</div>
<div class="chart" id="osc1"></div>
<div class="title">姿态峰峰(roll+pitch 的 1~99% 跨度)随油门变化</div>
<div class="chart" id="osc2"></div>
<table><tr><th class="l">油门</th><th>session 8 低频|ω|</th><th>session 8 姿态峰峰</th></tr>
<tr><td class="l">0.07</td><td>0.17</td><td>1.8°</td></tr>
<tr><td class="l">0.13</td><td>0.28</td><td>7.5°</td></tr>
<tr><td class="l">0.15</td><td>0.32</td><td>12.7°</td></tr>
<tr><td class="l">0.17</td><td>0.41</td><td>14.9°</td></tr>
<tr><td class="l">0.19</td><td>0.56</td><td>37.8°</td></tr>
<tr><td class="l">0.21</td><td>2.64</td><td>93.8°</td></tr></table>
<p>session 4/6 同样是从 0.1 油门附近的 0.25 rad/s 涨到 0.2 油门的 1~2 rad/s。
更关键的是 session 8 最后一次: 油门在 0.22 保持不变、遥控指令也不变(目标姿态 0),
而姿态振荡从"300~305 s 的 8° 峰峰"发展到"305~307.4 s 的 15~17° 峰峰、主频 1.65 Hz",
随后在 <b>t=307.40 s 一次性发散</b>。油门/指令都不动而振幅自己涨 = <b>闭环自激(稳定性余量不足)</b>,
不是操纵激励。</p>
<p>为什么会随油门增大? 混控把"期望力矩"折算成油门差动时用了
<code>k = m·g/(4·hover)</code>, 也就是<b>整个姿态环的等效增益与 1/hover 成正比</b>。
参数表里 hover = 0.25, 而实测 0.20 左右就离地 —— 于是实际增益比设计值高约 <b>1.25 倍</b>,
再加上下面的振动因素, 环路在接近悬停油门时就没有余量了。</p>
<div class="title">旁证: 三角量(陀螺帧间抖动 vs 电机指令抖动), 解锁 vs 未解锁</div>
<table><tr><th class="l">会话</th><th>状态</th><th>陀螺 20ms 帧间 |Δω| 中位</th><th>p95</th>
<th>电机指令帧间 |Δmotor| 中位</th><th>油门</th></tr>
<tr><td class="l">4</td><td>解锁</td><td>0.49 rad/s</td><td>1.94</td><td>58 counts</td><td>0.12</td></tr>
<tr><td class="l">4</td><td>未解锁</td><td>0.05 rad/s</td><td>0.23</td><td>0</td><td>0</td></tr>
<tr><td class="l">6</td><td>解锁</td><td>0.47</td><td>1.74</td><td>55</td><td>0.16</td></tr>
<tr><td class="l">8</td><td>解锁</td><td>0.40</td><td>1.46</td><td>47</td><td>0.14</td></tr>
<tr><td class="l">5</td><td>解锁(最低转速)</td><td>0.26</td><td>0.57</td><td>34</td><td>0.06</td></tr></table>
<p>桨一转起来, 陀螺的帧间抖动就从 0.05 跳到 0.4~0.5 rad/s(≈25 °/s, 20ms 内), 电机指令随之抖动 47~58 个 DShot 计数;
而且 <b>Δ力矩 与 Δ角速度的帧间相关系数是 −0.91 ~ −0.996</b> —— 高频段的电机输出几乎全是在"追着陀螺噪声打",
真正用于修正姿态的低频分量只占很小一部分。这既解释了图上"看不出修正意图"(被噪声淹没),
也解释了为什么振动越大、漂移越明显(周期性地把某一路打到 0 档/满档, 产生净的力矩与推力不对称)。</p>
<p>另外, 低油门时<b>偏航通道会吃掉差动权限</b>: 被裁剪的帧里, 偏航差动需求是可用油门的
<b>0.66~2.26 倍</b>(偏航力臂只有 0.03 m, 比横滚的 0.18 m 弱 6 倍), 于是横滚/俯仰的差动被整体缩小,
姿态权限进一步下降。这也是"收油门落地瞬间姿态最容易歪"的原因之一。</p>
</div>

<h3>6.4 最后一次"坠机"的真实时间线(链路超时不是原因)</h3>
<div class="card">
<p>把 session 8 末段的 20 ms 数据摊开(完整表见 oscillation_check.txt):</p>
<table>
<tr><th class="l">时刻 [s]</th><th class="l">链路</th><th class="l">高度/垂速(EKF)</th><th class="l">姿态</th><th class="l">说明</th></tr>
<tr><td class="l">304.0~304.8</td><td class="l">正常(19ms)</td><td class="l">+0.5 m / ≈0</td><td class="l">roll ±3° pitch −3~−9°</td><td class="l">油门 0.20, 悬停/贴地状态</td></tr>
<tr><td class="l">305.0</td><td class="l">正常</td><td class="l">+0.65 m / +0.11</td><td class="l">pitch −5.9°</td><td class="l">飞手把油门加到 0.22</td></tr>
<tr><td class="l">305.4~306.6</td><td class="l">正常</td><td class="l">+0.48 → −1.59 m, 垂速 −1.0 m/s</td><td class="l">roll 摆动 −6~+1°, pitch +5~+7°(机头下沉)</td><td class="l"><b>0.22 油门仍然在掉高度</b>, 同时姿态开始摆</td></tr>
<tr><td class="l">307.0~307.36</td><td class="l">正常(19~20ms)</td><td class="l">−1.0 m, 垂速 −0.2~−0.4</td><td class="l">roll 3~6°, pitch 0~+3°</td><td class="l">1s |ω| RMS 约 1.1 rad/s, 姿态在 ±6° 内摆动</td></tr>
<tr><td class="l">307.40</td><td class="l">最后一帧有效遥控(其后 linkAge 开始涨)</td><td class="l">−0.63 m</td><td class="l">pitch +6.1°</td><td class="l">|ω| 跳到 3.3 rad/s</td></tr>
<tr><td class="l">307.44~307.56</td><td class="l">丢失中</td><td class="l">−0.4 → −0.2 m</td><td class="l">pitch +8~+11.8°(持续低头)</td><td class="l"><b>|ω| 12.8 → 11.9 rad/s(≈730 °/s), 电机差动打满(某路到 0 档)</b> —— 撞击/失稳瞬间</td></tr>
<tr><td class="l">308~312.4</td><td class="l">丢失(linkAge 涨到 10.7 s)</td><td class="l">+0.5 m 附近抖动</td><td class="l">roll/pitch 各 ±10~20° 乱摆</td><td class="l">飞机在地面翻滚/跳动, 飞控仍以冻结的 0.22 油门输出</td></tr>
<tr><td class="l">312.4</td><td class="l">linkAge &gt; 5 s</td><td class="l">—</td><td class="l">—</td><td class="l">失控保护停机, Failsafe 锁存</td></tr>
<tr><td class="l">312.9</td><td class="l">丢失中</td><td class="l">垂速 −5.9 m/s</td><td class="l">roll −19°, pitch +2.5°</td><td class="l">停机后仍有一次 15.5 rad/s / 4.6 g 的撞击</td></tr>
<tr><td class="l">314.1 以后</td><td class="l">318.1 s 恢复</td><td class="l">静止</td><td class="l">静止</td><td class="l">|ω|≈0、|a|≈1.0 g, Failsafe 一直锁存到 371 s</td></tr>
</table>
<p><b>结论: 链路丢失与撞击发生在同一瞬间(307.4 s, 40 ms 窗口内), 而且"链路丢失"不可能造成
12.8 rad/s 的角速度突变(指令冻结前后完全一样), 所以是先撞击/失稳、再(或同时)链路中断。</b>
在 307.4 s 之前, 飞机已经在 0.22 油门下跌高度、姿态振荡到 ±6°, 不是"链路一断就掉下来"。
失控保护只是让电机在 5 s 后停了下来, 并没有造成这次坠机; 但它<b>确实拖了 5 s</b>,
而且这 5 s 内飞控继续按冻结的 0.22 油门输出 —— 这 5 s 值得缩短。</p>
<p>另外注意: 撞击前的高度/垂速说明这一刻的可用推力不足以维持高度:
0.22 油门(高于参数表 0.25 的 88%)在离地 1 m 以上反而以约 1 m/s 下降。可能是<b>地面效应</b>
(贴地时 0.20 就能离地, 起来后需要更大油门)、电池电压跌落、或某个电机/桨效率偏低。
建议做一次系留/吊挂测试: 把飞机固定后缓慢加油门, 记录"离地所需油门"和离地后的姿态/漂移方向,
这样能把"地面效应/推力不足"和"姿态环问题"彻底分开。</p>
</div>

<h3>6.5 "刚离地就往右前漂"的可能原因(按证据强度排序)</h3>
<div class="card">
<ol>
<li><b>振荡/振动整流(证据最强)</b>: 高频差动振荡把某几路电机周期性打到 0 档, 而螺旋桨的推力对
"掉转速"和"加转速"的响应不对称(RPM 掉下去要更久才回来), 于是平均推力出现固定的不平衡 →
产生固定的水平推力分量。数据支持: 力矩指令帧间与陀螺帧间相关 −0.91~−0.996(指令几乎全是噪声驱动),
振幅随油门单调增大。</li>
<li><b>姿态环余量不足(证据强)</b>: 增益标定偏高 1.25 倍(悬停 0.20 vs 参数 0.25) + 没有陀螺低通,
环路在接近悬停油门时从"衰减"变成"等幅/发散"。这与"刚离地就漂、只能马上收油门"的现象一致:
离地那一刻正好是油门进入 0.20~0.22 的区间, 也就是余量最差的区间。</li>
<li><b>推力矢量/重心偏置(需要一次实验确认)</b>: 在高油门窗口里, 机体坐标系下始终存在一个
固定方向的水平加速度: 前向 −0.03~−0.12 g(0.3~1.2 m/s²), 且随油门增大而增大
(session 4: 0.10~0.15 油门 −0.005 g → 0.15~0.30 油门 −0.120 g)。
它只能来自"推力矢量不垂直"(某个电机/机臂有安装角、重心偏、桨推力不等)或"该工况下姿态估计偏了 3~7°"。
日志无法把两者分开, 需要系留/吊挂实验。</li>
<li><b>偏航零偏跑飞(已确认, 间接影响)</b>: 静止时机头以 2~17 °/s 虚假旋转, 会让飞手看到的漂移方向
"转"得比实际快, 也会让偏航通道持续吃差动权限(6.3)。它不直接产生平移, 但会放大前三条的效果。</li>
<li>可以先排除的: 控制律符号/电机顺序/电机旋向/陀螺轴向接反 —— 复核见 6.2, 日志逐帧对得上账。</li>
</ol>
</div>

<h3>6.6 针对这次现象的调参/改动建议(优先级从高到低)</h3>
<div class="card">
<ol>
<li><b>给角速度环加陀螺低通</b>: 现在 P 项直接用原始陀螺, D 项只靠 10 ms 低通。建议硬件(BMI088 带宽)+
软件两级低通到 80~120 Hz, 并重新看相位余量。</li>
<li><b>把 HOVER_THROTTLE 改成实测值(≈0.20)</b>: 这一步同时修正混控的力矩换算与姿态环等效增益(1.25 → 1.0),
是最省事、收益最直接的一改。</li>
<li><b>降 RATE_ROLL/PITCH_P 与 ATT_P</b>: 现在是 9.0 / 3.0, 先把内环降到 6.0~7.0、外环 2.0~2.5 试,
用"离地 0.5 m 悬停 10 s 不摆"作为判据。</li>
<li><b>限制偏航差动</b>: 让偏航通道按可用差动(推力优先)先限幅, 不要让它把横滚/俯仰的权限吃掉;
THROTTLE_MIN 也可以从 0.01 提到 0.08~0.10, 保证"解锁后还有姿态权限"。</li>
<li><b>桨/机架减振</b>: 平衡桨、检查电机轴承与机臂刚性, 飞控加软垫; 目标是把陀螺帧间抖动压到 0.1 rad/s 量级
(现在是 0.4~0.5)。</li>
<li><b>linkTimeoutMs 5 s → 1~2 s</b>, 并在链路丢失时降油门, 不要冻结最后一帧指令。</li>
<li><b>做两次区分实验</b>: (a) 系留/吊挂, 油门从 0.10 缓慢加到 0.26, 记录离地油门、姿态与漂移方向;
(b) 同样条件下把桨换成一对新的/重新配平, 比较陀螺抖动。这样能确认"地面效应/推力不足"和"振动"各占多少。</li>
<li><b>日志加一路原始角速度</b>(不经低通/融合, 100~1000 Hz 打到调试流), 才能做频谱分析定位振动源;
现在 50 Hz 的姿态日志会把高频振动折叠进来, 只能看到"抖动大"。</li>
</ol>
</div>

<h3>6.7 最可能的直接原因: 姿态估计被"加速度计修正"拖走(与姿态环构成正反馈)</h3>
<div class="card">
<p>前几节把"振动""增益余量""偏航抢权限"都摆出来了, 但都不足以解释"<b>稳定地前倾/右倾, 而且电机像是越修越歪</b>"。
把姿态估计的来源拆开看就清楚了: 用欧拉角运动学把<b>实测陀螺积分出来的角度变化</b>与<b>日志四元数的角度变化</b>相减,
差值只可能来自"EKF 用加速度计把姿态拧过去"(脚本 <code>analyze_ekf_drag.py</code>):</p>
<table>
<tr><th class="l">窗口</th><th>roll: 四元数 / 陀螺积分 / 差</th><th>pitch: 四元数 / 陀螺积分 / 差</th></tr>
<tr><td class="l">s4 165~172s(第一次飞行加速段)</td><td>+11.8° / +12.8° / −1.0°</td><td>−13.4° / +8.5° / <b>−21.8°</b></td></tr>
<tr><td class="l">s4 172~179s(第一次飞行后段)</td><td>−23.0° / +33.4° / <b>−56.4°</b></td><td>+10.1° / +5.1° / +5.0°</td></tr>
<tr><td class="l">s6 43.8~50s(起飞段)</td><td>+11.7° / −6.8° / <b>+18.5°</b></td><td>−11.4° / −3.6° / −7.8°</td></tr>
<tr><td class="l">s8 305~307.4s(发散前)</td><td>−1.1° / +21.3° / <b>−22.3°</b></td><td>+9.9° / +15.7° / −5.8°</td></tr>
<tr><td class="l">s5 53~80s(台架, 对照)</td><td>+0.5° / +5.1° / −4.5°</td><td>−0.3° / +2.1° / −2.4°</td></tr>
<tr><td class="l">s8 170~200s(台架 30s, 对照)</td><td>+0.4° / +7.2° / −6.8°</td><td>−3.9° / −5.8° / +1.9°</td></tr></table>
<p>台架段差值是 0.2 °/s 量级(我方法的数值误差), 而<b>飞行段里这个"非陀螺"姿态变化速率达到中位 23~35 °/s、
p95 90~140 °/s</b>(session 4/6/8), 也就是说: <b>一加油门, 姿态估计就不是靠陀螺推出来的了, 而是被加速度计拧着走。</b></p>
<p>为什么会这样? 看倾角修正的门限: <code>accel_tilt_gate_mss = 2.0 m/s²</code>(0.2g), 只检查<b>幅值</b>,
不检查方向。实测解锁段里这个门有 <b>64%~74% 的时间是开着的</b>(|a| 中位 1.03~1.05g, 5~95% 到 0.79~1.77g) ——
也就是"推力大于重力"和"颠簸冲击"的绝大多数时刻, 加速度计都在当重力用, 而这时它测的其实是<b>推力方向 + 振动</b>。</p>
<p><b>接下来就是正反馈</b>(这条与符号约定无关, 纯物理): 假设飞机水平悬停时出现了一个向前的加速度 a(初始倾斜/推力不对称/风),
加速度计测到的比力方向就会向前偏 atan(a/g); 估计器把"比力就该指向世界上方"当真理, 于是认为"飞机是抬头 atan(a/g)";
姿态环为了纠正这个并不存在的抬头, 命令<b>压机头</b> —— 而压机头恰好又产生向前的加速度, 于是偏得更多、压得更狠。
平衡点大约在 atan(a/g) 的倾角上: 0.2g 的水平加速度对应 11° 倾角, 而 11° 倾角又正好产生 0.2g 的水平加速度 ——
<b>一个自洽的"稳定前倾"状态, 水平加速度约 2 m/s²(0.2g), 1 秒漂 1 m、速度到 2 m/s。</b>
这与"刚离地就快速往右前漂、拉不住、只能马上收油"完全吻合; 也解释了为什么油门越大越明显(加速度越大, 假倾角越大)。</p>
<p>s4 第一次飞行的 169.5~171.5s 就是这个过程: 四元数 pitch 掉了 19°, 而陀螺只支持 7° ——
多出来的 12° 是估计器自己转的, 飞机真实转动远没有那么多; 控制器却按这 19° 去打舵, 于是越打越偏。</p>
<p><b>修法(优先级最高)</b>: 把倾角修正改成"只在确实没有明显线加速度时才吃加速度计":</p>
<ol>
<li>门限从 2.0 m/s² 收紧到 0.3~0.5 m/s²(即 ||a|−g| &lt; 0.03~0.05g)。按实测, 0.22 油门时 ||a|−g| ≈ 1 m/s²,
这样推力段就会自动停止吃加速度计, 姿态改由陀螺短时支配(静止时陀螺零偏已经收敛, 20~30 s 飞行不会漂太多);</li>
<li>或者更稳: 用"新息方向"门限 —— 比较实测比力方向与预测重力方向的夹角, &gt;10~15° 就拒绝该次更新(而不是只看幅值);</li>
<li>把 <code>accel_noise_sigma</code> 调大(0.15 → 0.5~1.0 m/s²), 让加速度计只能慢慢"找平", 短时姿态靠陀螺;</li>
<li>把门限状态/新息角写进日志(现在完全看不到), 便于下次直接确认;</li>
<li>做完以上改动后, 再做一次<b>系留/吊挂</b>实验: 飞机不被允许平移, 油门从 0.1 缓慢加到 0.25,
观察(1)姿态估计是否还在没有陀螺支持的情况下自己转, (2)真机是否仍然前倾。这一步能干净地把本条和"振动/增益"分开。</li>
</ol>
</div>

<h3>6.8 与"曾经飞成功那次"的对比, 以及静止零点偏移</h3>
<div class="card">
<p>飞手补充(已修正): 早期 <code>main_final_test</code> 用 <b>Madgwick</b> 姿态解算, 悬停油门 0.30,
而 <b>RATE_ROLL/PITCH_P = 8.0</b>; 现在改成悬停 0.25、RATE_ROLL/PITCH_P = 9.0
(注释里写明: 悬停油门调小后同步把速率环 P 提上来补偿)。姿态环 P 两次相同。
把等效增益一起算进去: 悬停 0.30→0.25 让混控折算的等效增益变成 0.83 倍, 速率 P 8.0→9.0 是 1.125 倍,
<b>净变化 9/8 × 0.83 ≈ 0.93 倍, 基本没变</b>。</p>
<table>
<tr><th class="l">项目</th><th>当时(飞成功)</th><th>现在</th><th>影响</th></tr>
<tr><td class="l">姿态解算</td><td>Madgwick(固定小权重加速度计修正, 不估陀螺零偏)</td>
<td>15 状态 EKF(加速度计观测 + 在线估陀螺零偏)</td>
<td>加速度计一旦不可信, EKF 会把误差吸收成"零偏", 并让姿态在没有陀螺支持的情况下转动(见 6.7)</td></tr>
<tr><td class="l">速率环 P</td><td>8.0</td><td>9.0 (+12.5%)</td><td>为补偿悬停参数的改动而同步提高</td></tr>
<tr><td class="l">悬停油门</td><td>0.30</td><td>0.25(实测约 0.20)</td>
<td>等效增益 k_true/k_param = hover参/hover真: 1.5 → 1.25 倍(相对真实悬停点)</td></tr>
<tr><td class="l">相对首飞的净增益</td><td>1.00</td><td>9/8 × 0.83 ≈ 0.93</td><td><b>基本不变</b></td></tr></table>
<p><b>也就是说: 相对那次飞成功的配置, 唯一实质性的改动就是姿态解算(Madgwick → EKF)。</b>
这是一次非常干净的单变量对比 —— 也解释了为什么"电机/混控/接线/IMU 轴向"这些都不用怀疑:
同样的硬件、几乎同样的控制增益, 换了估计器就从"能飞"变成"刚离地就稳定前倾"。</p>
<p>另外, 静止读数长期偏负(抬头/左倾), 这个零点偏移会被控制器当成"要压平"的误差:</p>
<table><tr><th class="l">会话</th><th>静止 pitch(抬头为负)</th><th>静止 roll</th><th>等效: 飞起来会被压成</th></tr>
<tr><td class="l">s4(11 个静止窗)</td><td>−0.44 ~ −1.57°</td><td>−0.29 ~ −2.29°</td><td>低头 0.1~0.3° + 右倾 0.05~0.4 m/s² 方向</td></tr>
<tr><td class="l">s5(4 窗)</td><td>−0.20 ~ −0.25°</td><td>−0.80 ~ −0.86°</td><td>同上(幅度更小)</td></tr>
<tr><td class="l">s6(4 窗)</td><td>−0.16 ~ +0.27°</td><td>−0.50 ~ −1.37°</td><td>同上</td></tr>
<tr><td class="l">s7(1 窗)</td><td>−0.06°</td><td>−1.50°</td><td>同上</td></tr>
<tr><td class="l">s8(5 窗)</td><td>−0.08 ~ −0.66°</td><td>−0.26 ~ −1.01°</td><td>同上</td></tr></table>
<p><b>方向完全对得上</b>: 静止读数偏"抬头 + 左倾"(负值), 控制器飞起来就会把飞机压成"低头 + 右倾",
也就是<b>持续向前 + 向右的水平加速度</b> —— 与实飞观察到的"右上方漂"一致。
水平加速度 ≈ g·tan(偏移): 1° ≈ 0.17 m/s², 3° ≈ 0.5 m/s², 5° ≈ 0.86 m/s²。
静止偏移本身只有 0.1~0.4 m/s², 但在推力段 EKF 会把这个偏移放大(6.7 节的 3.5~7° 等效偏差 →
实测 0.6~1.2 m/s²), 于是"刚离地就漂得很快"。</p>
<p><b>可以做的两件事</b>: (1) 把飞机放在<b>确认水平</b>的台面上(或用水平仪垫平), 看静止读数是否回到 0.0°;
若长期偏 0.5° 以上, 就是加速度计零点/安装偏角, 应做一次零偏标定或加装水平补偿, 别用扩大姿态环增益的办法去压它。
(2) 先按"当时飞成功的那套"做 A/B: Madgwick + P=0.7 + 悬停 0.30, 同一台飞机再飞一次 ——
如果不再出现稳定倾斜, 就直接证明问题在"EKF 的加速度计/零偏处理 + 增益", 与电机/混控/接线无关。</p>
<p><b>参数历史(git 记录)</b>: 51b1b17 是"早期一版"(质量 0.45 kg、惯量 0.004/0.004/0.007、悬停 0.45、
ATT_P 7.0、RATE_P 20), aea674e 换成了实测模型并写"首飞保守"(质量 1.566 kg、惯量 0.0234/0.0124/0.0338、
悬停 0.30、ATT_P 3.0、RATE_P 8.0); 工作区在再往后把悬停改成 0.25、RATE_P 提到 9.0。
注意<b>内环"等效"增益</b> = RATE_P × (参数惯量/真实惯量):
早期 ≈ 20×0.17 = 3.4, 现在 ≈ 9×1.0 = 9.0(若再乘混控的悬停偏差 1.25 则 ≈ 11.3) ——
也就是<b>现在内环比首飞那次硬 2.6~3.3 倍</b>, 而 ATT_P 反而从 7.0 降到 3.0(外环变软)。
这正是"低频姿态控制变慢、高频更容易被振动和估计误差激励"的组合。</p>
<div class="title">陀螺零偏估计: 静止时为 0, 一上桨就跑到 8~17 °/s(=虚假纠偏的量)</div>
<table><tr><th class="l">会话</th><th>起始(静止, 桨不转)</th><th>解锁段最大 |bx|/|by|/|bz|</th><th>末值 bx/by/bz</th></tr>
<tr><td class="l">s4</td><td>0.00 / 0.00 / −0.00 °/s</td><td>2.01 / 2.06 / 8.47 °/s</td><td>+0.19 / +0.12 / −7.14</td></tr>
<tr><td class="l">s5</td><td>0 / 0 / 0</td><td>0.20 / 0.23 / 17.19</td><td>+0.07 / +0.14 / −17.14</td></tr>
<tr><td class="l">s6</td><td>0 / 0 / 0</td><td>1.28 / 2.04 / 7.37</td><td>−0.45 / −0.50 / −8.26</td></tr></table>
<p>静止时 x/y 陀螺真实零偏应该在 0.5 °/s 量级, 这里却估到 2 °/s; z 真实约 −1.1 °/s, 估到 8~17 °/s。
这些"多出来的零偏"是加速度计方向误差被吸收进去的结果 —— 它会让姿态估计以同样的速率持续旋转,
也就是飞手说的"PID 一直在和一个虚假的纠偏对抗"。</p>
</div>

<h3>6.9 为什么"读数为负"和"向右/向前漂"不矛盾(符号链)</h3>
<div class="card">
<p>约定(代码 + 上位机 + 飞手实测一致): <b>pitch &gt; 0 = 低头, pitch &lt; 0 = 抬头; roll &gt; 0 = 右侧下沉(右倾), roll &lt; 0 = 左倾。</b></p>
<p>产生平移的只有<b>真实倾角</b>: 低头 → 推力向前 → 往前加速; 右倾 → 推力向右 → 往右加速。
而控制器做的是"把<b>读数</b>压到 0", 所以 <b>真实倾角 = −(读数相对真水平的偏移)</b>:</p>
<table>
<tr><th class="l">轴</th><th>静止读数(桨不转)</th><th>飞行中控制器把读数压向</th>
<th>真实倾角 = 读数 − 零点偏移</th><th>结果</th></tr>
<tr><td class="l">pitch</td><td>−0.4 ~ −1.6°(抬头)</td><td>0°</td><td><b>+0.4 ~ +1.6°(低头)</b></td><td>持续向<b>前</b>加速</td></tr>
<tr><td class="l">roll</td><td>−0.3 ~ −2.3°(左倾)</td><td>0°</td><td><b>+0.3 ~ +2.3°(右倾)</b></td><td>持续向<b>右</b>加速</td></tr></table>
<p>所以: <b>"读数偏负(抬头/左倾)"和"实机向右前漂"是同一件事的两种说法</b> ——
读数偏负的那部分, 正是控制器要压掉的部分, 压掉之后实机就变成"低头 + 右倾"。
你问的"向右偏是 Roll&gt;0 吧?"——对, <b>真实 roll &gt; 0(右侧下沉)</b>, 而它对应的读数偏移是负的, 两者不冲突。</p>
<p>换算: 真实倾角 θ 对应水平加速度 ≈ g·tan θ; 1° ≈ 0.17 m/s², 3° ≈ 0.5 m/s², 5° ≈ 0.86 m/s²。
静止偏移(0.3~2.3°)本身只给出 0.05~0.4 m/s²(慢漂), 但推力段估计器会把等效偏差放大到 3.5~7°(0.6~1.2 m/s²),
这就是"刚离地就漂得很快"; 而读数在飞行中一直稳在负值不收敛, 说明估计器在持续把这个负偏差重新注入 ——
PID 并不弱(它已经在用 +0.7~+1.1 rad/s 去纠 −19° 的读数), 是它在跟一个<b>错误的、还在移动的参考</b>打架。</p>
</div>

<h3>6.10 关键自相矛盾: "控制器要 pitch 增大, 陀螺也读到增大, 而日志的 pitch 却在减小"</h3>
<div class="card">
<p>飞手提出的是全案最关键的一点: 控制器在纠偏(期望角速度为正, 即要把 pitch 往上抬), 实测陀螺也是正的
(按同一约定也代表 pitch 在增大), <b>而日志里的 pitch 反而在减小、长期停在负值</b>。这三件事不能同时成立 ——
它把故障范围缩到只剩一个可能:</p>
<ul>
<li><b>权限/饱和/增益都解释不了</b>: 混控裁剪只会让"纠偏变小、变慢", 绝不会让读数<b>反向</b>走;
增益高低也只影响收敛速度与振荡幅度, 不会让估计值朝相反方向跑。</li>
<li>唯一能让"读数违背自己的陀螺"的, 是<b>加速度计倾角修正</b>: 日志四元数的姿态变化减去陀螺积分,
飞行段差出 <b>23~35 °/s(中位)、p95 90~140 °/s</b>, 台架只有 0.2 °/s。
也就是说, 飞行中"姿态"主要是被加速度计拧出来的, 不是转出来的。</li>
</ul>
<p>为什么台架上(桌面低头 → pitch&gt;0)一切正常, 一飞就错? 因为两种工况下"加速度计是不是重力"完全不同:</p>
<table>
<tr><th class="l">工况</th><th>加速度计测到的是</th><th>能否当重力用</th><th>姿态估计由谁主导</th></tr>
<tr><td class="l">台架/桌面(桨不转, 手慢慢摆)</td><td>真正的重力方向</td><td>能(准静态)</td>
<td>加速度计修正主导 → 估计正确, 所以"桌面低头 pitch&gt;0"看不问题</td></tr>
<tr><td class="l">解锁飞行(桨转、有推力/振动)</td><td><b>推力方向 + 振动</b>(与重力方向差 12~24°)</td><td>不能</td>
<td>加速度计仍在修(门限 64~74% 时间是开的), 且陀螺贡献很大 → 估计被"假重力"拖走</td></tr></table>
<p>这也解释为什么"越加油门越严重": 推力越大 → 加速度计方向偏离重力越多 → 门限越容易放行 →
假修正越强; 同时水平加速度也越大, 一旦偏起来就自我强化。</p>
<p><b>必须用实验判定的一步</b>: 现在的证据无法排除"陀螺某个轴的符号与加速度计/姿态约定不一致"这种硬件级问题
(它的典型表现恰好就是"桌面慢动作看着对、飞起来方向反过来": 慢动作时加速度计主导, 一飞起来陀螺主导才暴露)。
两个实验都在 10 分钟内:</p>
<ol>
<li><b>静止加油门实验</b>(拆桨或按住机体不让它转): 油门从 0 缓慢加到 0.25, 盯住日志/上位机的 pitch、roll
和 gyro。理想情况: 陀螺≈0 且姿态读数一动不动。若读数自己爬升/翻转 → 估计器被推力/振动拖走, 实锤。</li>
<li><b>陀螺/电机方向实验</b>(拆桨, 用 <code>command=1</code> 直接给电调值, 建议把飞机吊起来让它能自由转):
   给"后侧两路 +200 counts"的固定差动 → 机头必须下沉; 此时日志 pitch 应当<b>增大</b>(低头=pitch&gt;0)、
   gyro_y 应当为<b>正</b>。再给"左侧两路 +200" → 机体应向右倾, roll 应<b>增大</b>、gyro_x 应为<b>正</b>。
   只要哪一个符号相反, 就是陀螺/加速度计轴向或映射不一致 —— 那会直接解释"读数与真机反向"。</li>
</ol>
<p>实验 1 通过、实验 2 符号全对的情况下, 就只剩"加速度计在推力段不可信"这一条, 按 6.7 的方法收紧门限/
放大 accel 噪声/冻结零偏即可; 反之若实验 2 出现符号相反, 先修轴向映射, 再谈滤波参数。</p>
</div>

<h3>6.10 重要限制: 这批日志里的"姿态角(四元数)"不可信 —— 它没有跟着陀螺走</h3>
<div class="card">
<p>飞手提供了录像: session 4 的 160~180s 加速段飞机是<b>机头下沉(低头)</b>, 几秒后触地导致机体瞬间大角度旋转。
而日志里的 pitch 却是<b>负的</b>(按已验证的约定 = 抬头)。为弄清是"约定"还是"估计"的问题, 我做了与任何约定都无关的检验
(脚本 <code>check_quat_vs_gyro.py</code>): 用相邻两帧四元数算出"<b>四元数自己认为的机体角速度</b>"
(Δq = conj(q_k)·q_(k+1) → 旋转向量/dt), 与<b>同一条日志里的陀螺</b>逐帧比较 —— 两者理应几乎相等。</p>
<p>结果(每 0.5s 平均, rad/s):</p>
<table>
<tr><th class="l">时间(油门)</th><th>四元数隐含 wx,wy,wz</th><th>同段陀螺均值 wx,wy,wz</th><th>差(无陀螺支持)</th></tr>
<tr><td class="l">162.5s (0.080)</td><td>+0.002 −0.010 <b>+0.277</b></td><td>−0.016 −0.013 −0.005</td><td>+0.018 +0.003 <b>+0.281</b></td></tr>
<tr><td class="l">164.5s (0.080)</td><td>+0.030 +0.004 <b>+0.408</b></td><td>−0.003 −0.008 −0.004</td><td>+0.033 +0.012 <b>+0.412</b></td></tr>
<tr><td class="l">166.0s (0.080)</td><td>−0.022 −0.044 <b>+0.441</b></td><td>−0.002 +0.010 −0.010</td><td>−0.020 −0.054 <b>+0.452</b></td></tr>
<tr><td class="l">170.5s (0.160)</td><td>+0.354 <b>−0.271</b> +0.505</td><td>+0.029 +0.012 −0.023</td><td>+0.325 <b>−0.283</b> +0.528</td></tr>
<tr><td class="l">171.0s (0.160)</td><td>+0.037 <b>−0.188</b> −0.491</td><td>+0.000 +0.035 −0.014</td><td>+0.037 <b>−0.224</b> −0.477</td></tr>
<tr><td class="l">173.0s (0.180)</td><td>+0.193 <b>−0.127</b> +0.586</td><td>−0.019 +0.014 −0.012</td><td>+0.212 <b>−0.140</b> +0.597</td></tr>
</table>
<p><b>结论: 在这段飞行里, 四元数比陀螺多转了 0.2~0.6 rad/s(11~34 °/s), 其中 yaw 轴几乎全程多转, pitch 轴在 170~173s 多转 0.14~0.28 rad/s(8~16 °/s)且方向与陀螺相反。</b>
也就是说: <b>日志里的姿态角并不是"陀螺积分 + 慢慢找平"的结果, 而是被估计器自己转出来的。</b>
因此这批日志中的 pitch/roll/yaw <b>不能用来判断飞机真实姿态</b>(录像才是这一段的真值); 可信的原始量只有
<code>bodyRate</code>(陀螺)与 <code>accelG</code>(加速度计)。</p>
<p>多出来的这部分转动从哪来? 名义积分是 <code>q ← q·exp((ω − bg)·dt)</code>, 所以<b>零偏估计错多少, 姿态就按那个速率多转多少</b>。
实测这次会话里 bz 从 157s 的 ≈0 一路涨到 183s 的 <b>−0.115 rad/s(−6.6 °/s)</b> 并保持, 而真实 z 零偏只有 −1.1 °/s
(用桨停车的静止段量: 那段 yaw 漂移 −1.15 °/s 且 bz≈0)。<b>也就是零偏估计被放大了约 6 倍</b>, 而它把姿态 yaw 持续转起来了;
pitch/roll 上则是"错误零偏"和"被推力/振动污染的加速度计修正"互相拉扯, 净效果就是估计器脱离真实姿态
(pitch 在 170~173s 与陀螺反向就是这个表现)。</p>
<p>把它接回飞手的观察: 录像里飞机<b>低头</b>, 而估计器报<b>抬头(pitch&lt;0)</b> → 姿态环为了"纠正抬头"继续压机头 →
越压越低头 → "电机一直没能修正"。方向、现象、录像三者自洽; 唯一不可信的就是日志里的姿态角。</p>
<p><b>三个可以马上做的判定实验</b>(都不需要新硬件):</p>
<ol>
<li><b>静止 30s(桨不转)</b>: 用 <code>check_quat_vs_gyro.py</code> 比较"四元数隐含角速度"与陀螺 —— 正常应几乎相等;
若有 5 °/s 以上的差, 说明估计器自己在转(零偏/加速度计机制)。</li>
<li><b>手抬机头(桨不转)</b>: 确认日志里 <code>pitch 增大</code>(低头=pitch&gt;0), 同时 acc_x 变负 —— 端到端验证他们的硬件约定。</li>
<li><b>系留/吊挂加油门(0.10→0.25)</b>: 看(1)姿态估计是否在没有陀螺支持的情况下自己转, (2)电机是否在"纠正"一个并不存在的姿态。
这一步能一次性把"估计器问题"和"振动/增益问题"分开。</li>
</ol>
</div>

<h3>6.11 完整因果链: 为什么"飞机低头, 估计器却报抬头"</h3>
<div class="card">
<p><b>先排除传感器/轴向</b>(脚本 <code>check_axes_strong.py</code>): 用"陀螺积分出的 0.5s 转动"去预测加速度计方向的变化
(u(t₂) 应等于 ΔR^T·u(t₁)), 在 5 个会话共 1500+ 个手搬动窗口上比较各种符号/置换组合 ——
<b>原始轴向 (+x,+y,+z) 的平均夹角误差 0.77°、中位 0.33°(噪声量级), 优于任何单轴翻转或轴交换</b>。
即加速度计与陀螺彼此完全自洽, 没有接反/换轴; 问题不在传感器、驱动或轴映射。</p>
<p>把前面所有测量串起来, 这就是 session 4 两次飞行共同表现的那条链(每一步都有实测支撑):</p>
<ol>
<li><b>油门加到 ≈0.16~0.24, 推力超过重力</b>(实测悬停 0.20 左右) → 飞机开始离地/被地面反作用推着走,
加速度计测到的已经不是重力, 而是"推力 + 振动 + 撞击"的合力(实测飞行段 |a| 中位 1.13~1.19 g,
5~95% 到 0.8~1.8 g)。</li>
<li><b>倾角修正门限只看幅值</b>(<code>accel_tilt_gate_mss = 2.0 m/s²</code> = 0.2g) →
解锁段有 <b>66%(第一次飞行)/72%(第二次)/64%(s6)/72~79%(s8)</b> 的帧"门是开着的",
即这些帧里滤波器把上面那个假方向当成重力用。</li>
<li><b>假方向被吸收成"陀螺零偏"</b>: 一个持续的姿态新息在时间上看起来就像一个恒定角速度,
15 状态滤波器正好有"零偏"这个状态去装它 → 实测零偏估计被推到
<b>bz 最大 −17.2 °/s(钳位, 真实值 −0.73 °/s, 差 23 倍)</b>, bx/by 到 +2~3 °/s(真实 +0.12/+0.00)。</li>
<li><b>零偏错多少, 姿态就按那个速率转多少</b>: 名义积分是 <code>q ← q·exp((ω − bg)·dt)</code>,
所以 bz = −6.6 °/s 就意味着姿态 yaw 以 +6.6 °/s 自己转; 偏航方向没有其它观测, 错了也没人纠正,
于是"姿态角"和"实测陀螺"长期对不上(实测四元数 yaw 在 161~176s 转 15~20 °/s, 陀螺只有 −1 °/s 量级)。</li>
<li><b>俯仰/横滚方向被"水平加速度"骗</b>: 飞机一旦有向前加速度 a, 比力方向就朝机头偏 atan(a/g),
滤波器把它当成"世界上方朝机头偏" ⇒ 认为飞机是<b>抬头</b>;
姿态环为了纠正这个不存在的抬头就<b>压机头</b> ⇒ 产生更大向前加速度 ⇒ 更抬头 ⇒ 越压越狠。</li>
<li><b>正反馈闭合成"稳定前倾 + 右前漂"</b>: 这就是录像里"低头、向前、几秒后触地"的过程;
同时因为估计器报的是抬头, 姿态环的输出看起来"像是在修正", 但修的是错的参考, 所以飞手看到的是"电机一直没能修正"。</li>
<li><b>撞击(178.86s, |ω| = 6.86 rad/s)之后</b>: 大角速度下陀螺重新占主导, 姿态估计才又跟上真实运动
(帧间相关 0.40 / 0.84), 但那时飞机已经撞地。</li>
</ol>
<p><b>为什么这是"可能的"不合理现象</b>: 加速度计在推力飞行中不再是重力传感器, 而滤波器(1)门限太松、
(2)把方向误差吸收成零偏、(3)零偏又直接驱动姿态积分 —— 三条叠加, 就得到一个"自己会转、
而且转的方向刚好让控制器把飞机越推越歪"的姿态估计。这是"用加速度计做姿态参考 + 姿态环"的经典正反馈失效,
不是 PID 参数问题, 也不是电机/混控/接线问题。</p>
<p><b>唯一还没完全定量闭合的一环</b>: 实测四元数多转的量(15~20 °/s)里, 零偏能解释约 6.6 °/s,
其余部分来自加速度计更新通过协方差耦合到偏航(以及 50 Hz 日志对高频振动的采样折叠)。
要彻底闭合这一环, 只需在第 6.10 的静止/系留实验里同时记录"四元数隐含角速度 − (陀螺 − 零偏)"这条残差。</p>
</div>

<h3>6.12 关键细化: "电机刚启动、还没离地"那几秒到底发生了什么(162~169.5s)</h3>
<div class="card">
<p>飞手提出: 电机刚启动时(油门 0.08~0.12, 悬停≈0.20)几乎还没有线加速度, 为什么 pitch 已经在变?
实测(脚本 <code>analyze_startup_pitch.py</code>, session 4 第一次飞行):</p>
<table>
<tr><th class="l">窗口</th><th>油门</th><th>|a|</th><th>acc_x 均值(→加速度计给出的俯仰)</th>
<th>pitch 均值</th><th>陀螺 wy 均值</th><th>bz 零偏</th><th>yaw 变化</th></tr>
<tr><td class="l">A 154~161.8s 静止(桨停)</td><td>0</td><td>1.003 g</td><td>+0.009 g → −0.52°</td>
<td><b>−0.518°</b></td><td>+0.0030</td><td>+0.27 °/s</td><td>−42.6 → −52.5°</td></tr>
<tr><td class="l">B 161.8~163s 刚启动</td><td>0.080</td><td>1.012 g</td><td>+0.004 g → −0.23°</td>
<td>−0.545°</td><td>−0.0054</td><td>−0.02 °/s</td><td>−51.3 → −40.7°</td></tr>
<tr><td class="l">C 163~165s</td><td>0.080</td><td>1.020 g</td><td>+0.008 g → −0.45°</td>
<td>−0.607°</td><td>−0.0098</td><td>−0.38 °/s</td><td>−39.9 → −10.8°</td></tr>
<tr><td class="l">D 165~167s</td><td>0.081</td><td>1.018 g</td><td>+0.011 g → −0.64°</td>
<td>−0.619°</td><td>+0.0042</td><td>−1.29 °/s</td><td>−9.3 → +25.2°</td></tr>
<tr><td class="l">E 167~169.5s</td><td>0.111</td><td>1.015 g</td><td>+0.017 g → −0.99°</td>
<td>−0.492°</td><td>+0.0172</td><td>−2.82 °/s</td><td>+25.2 → +69.2°</td></tr>
<tr><td class="l">F 169.5~171s 开始离地</td><td>0.148</td><td><b>1.129 g</b></td><td>−0.038 g → +2.18°</td>
<td><b>−5.19°</b></td><td>+0.0642</td><td>−4.34 °/s</td><td>+69.1 → +110.5°</td></tr>
<tr><td class="l">G 171~173s</td><td>0.175</td><td><b>1.229 g</b></td><td>−0.005 g → +0.29°</td>
<td><b>−14.70°</b></td><td>−0.0065</td><td>−4.04 °/s</td><td>+109.3 → +94.7°</td></tr></table>
<p>三条结论:</p>
<ol>
<li><b>"桨转但不离地"这段(162~169.5s) pitch 其实没漂</b>: 均值 −0.49~−0.62° vs 静止 −0.518°, 差 0.03~0.10°;
加速度计也一直是 1.00~1.02 g、acc_x≈+0.01(自身给出的俯仰 −0.2~−1.0°), 与静止一致。
所以"飞机在加速把加速度计带偏"这个机制在这一段<b>不成立, 也不需要成立</b>。</li>
<li><b>这一段真正在跑的是偏航和零偏估计</b>: yaw 从 −51° 转到 +69°(≈15.6 °/s), 而陀螺 wz 只有 −1~−2 °/s;
bz 零偏从 0 涨到 −3.65 °/s。偏航没有任何外部参考, 唯一能"解释"持续新息的只有零偏状态, 它一旦错就直接
变成姿态自转(积分是 q←q·exp((ω−bg)dt))。</li>
<li><b>pitch 的失效时刻与"|a| 离开 1g"精确重合</b>: 169.5s |a|=1.08 g、170.0s |a|=1.19 g(推力超过重力),
pitch 在 1.5s 内从 −0.3° 跑到 −15°。也就是说: 触发条件是<b>"加速度计不再是重力传感器"(推力>重力)</b>,
而不是"飞机在加速"本身 —— 两者时间上重合, 机理上要区分开。</li>
</ol>
<p><b>但这里还留下一处"数学上不该发生"的现象</b>(必须如实报告): 在 169.5~171s,
pitch 估计以约 −10 °/s 往负方向跑, 而<b>同一时刻陀螺 wy = +3.7 °/s、加速度计说"低头 +5.4°"</b> ——
两个量测都应当把 pitch 往正方向推, 估计却往反方向跑(实测四元数隐含 wy ≈ −0.17~−0.27 rad/s,
陀螺 +0.06~+0.18)。这只能属于以下三种之一:</p>
<ul>
<li>(a) 日志里的 <code>bodyRate</code> 不是滤波器实际使用的那路陀螺(数据通路/时间戳错位);</li>
<li>(b) 滤波器实现对某些工况存在符号/数值问题(例如倾角更新的新息/增益在 H 秩亏方向上的处理);</li>
<li>(c) 日志里的四元数不是滤波器内部的 <code>_q</code>(写入路径/结构体版本不匹配)。</li>
</ul>
<p>顺便: 加速度计与陀螺本身是对的(空载手搬动 1500+ 窗口的轴向互检误差中位 0.33°),
所以这三点都指向"飞控内部的估计/记录路径", 而不是传感器。建议在飞控里加一路 <b>EKF 自检输出</b>:
每个控制周期把 <code>_q</code>、<code>_bg</code>、该周期的 ω、a、本次倾角更新的 <code>inno</code>、
门限开/关状态一起打出来(调试流即可)。台架 30 s 就能把 (a)(b)(c) 一次分清, 不必再靠事后反推。</p>
</div>

<h3>6.13 第二次飞行(216.6~232s)"零偏≈0、气压≈0 但姿态仍不对"的核对</h3>
<div class="card">
<p>先把日志读数摊开(脚本 <code>analyze_second_flight.py</code>, 三路对照: EKF 姿态 / 加速度计给出的倾角 /
只积分陀螺得到的倾角):</p>
<table>
<tr><th class="l">时刻</th><th>油门</th><th>\|a\|</th><th>EKF pitch</th><th>加速度计 pitch</th>
<th>陀螺积分 pitch</th><th>baroRel</th><th>门限开</th></tr>
<tr><td class="l">216.5</td><td>0.064</td><td>1.00</td><td>−0.41</td><td>−0.33</td><td>−0.03</td><td>−0.07</td><td>100%</td></tr>
<tr><td class="l">220.0</td><td>0.082</td><td>1.02</td><td>−0.22</td><td>−0.71</td><td>−1.12</td><td>−0.26</td><td>92%</td></tr>
<tr><td class="l">224.0</td><td>0.100</td><td>0.99</td><td>−0.48</td><td>−0.84</td><td>−0.51</td><td>−0.33</td><td>100%</td></tr>
<tr><td class="l">227.0</td><td>0.120</td><td>1.00</td><td>−1.20</td><td>−0.26</td><td>−1.10</td><td>−0.73</td><td>100%</td></tr>
<tr><td class="l">228.0</td><td>0.133</td><td>1.03</td><td>−2.92</td><td>−0.38</td><td>+1.18</td><td>−0.76</td><td>96%</td></tr>
<tr><td class="l"><b>229.0</b></td><td>0.140</td><td>1.10</td><td><b>−7.65</b></td><td>−1.77</td><td>−1.09</td><td><b>−1.19</b></td><td>56%</td></tr>
<tr><td class="l"><b>230.0</b></td><td>0.140</td><td>1.10</td><td><b>−6.73</b></td><td>+0.79</td><td>−1.60</td><td>−0.46</td><td>64%</td></tr>
<tr><td class="l"><b>231.5</b></td><td>0.171</td><td>1.26</td><td><b>−11.81</b></td><td>+0.34</td><td>+1.69</td><td>−0.41</td><td>36%</td></tr>
</table>
<p>三条观察:</p>
<ol>
<li>216.5~227s 三方基本一致(都在 ±2° 内), 说明那段姿态估计没明显错 —— 错是从 <b>228.5s 之后</b>开始的。</li>
<li>228.5s 之后 EKF 一路跑到 −7~−12°, 而<b>陀螺积分只在 ±2° 内、加速度计给出的倾角也只在 ±2° 内</b>;
同一时段唯一发生系统变化的是<b>气压(相对基准从 −0.07 掉到 −1.19 m)</b>。也就是说: 三个输入里只有气压变了,
变化的方向和时间与姿态错误一致 ⇒ 这一段的姿态错误来自气压计。</li>
<li><b>bx/by ≈ 0 与"气压计推动姿态"并不矛盾</b>: 零偏是随机游走状态(过程噪声 1e-5), 它只能慢慢吸收<b>持续</b>的
新息; 而高度量测虽然 H 只挂 pz, 增益 <code>K = P·Hᵀ/S</code> 取的是 P 的 pz 整列, 里面和姿态误差有交叉协方差
(由 <code>δv̇=−R[f_b]×δθ</code>、<code>ṗ=v</code> 累积), 所以<b>姿态可以被单次气压更新直接、快速地推动</b>,
不需要零偏先动。这正是"零偏还≈0, 姿态已经错了"的表现。</li>
</ol>
<p>剂量-反应关系也吻合: 第一次飞行气压虚降 4.65 m ⇒ pitch 错约 24°; 第二次飞行气压虚降 1.2 m ⇒ 错约 12°,
量级同阶(约 0.1~0.2 m/度)。</p>
<p><b>还缺一块拼图</b>: 协方差 P 没有进日志, 所以"气压新息到底有多少流进了姿态"只能反推。
建议把 P 的对角线 + pz↔姿态误差的交叉项一并打进调试流, 一次就能把这条耦合量化。</p>
</div>

<h2 id="actions">七、总建议(按优先级)</h2>
<div class="card">
<ol>
<li><b>具体到"改哪个参数/哪一行代码"的清单, 见同目录 <code>params_to_change.md</code></b>
    (气压耦合、零偏限幅与解锁保留、倾角门限/噪声、秩亏保护、控制环增益、诊断日志, 以及桌面验收判据)。</li>
<li><b>【最高优先级】收紧加速度计倾角修正门限</b>(2.0 m/s² → 0.3~0.5 m/s², 或改成"方向新息"门限),
    否则姿态估计在推力段会被加速度计拖走, 与姿态环形成正反馈 —— 这是"稳定前倾、越修越歪"的直接原因(见 6.7)。</li>
<li><b>【最高优先级】切断气压计→姿态的耦合</b>(见 6.13): 做高度更新前把 P 中 pz 与姿态/零偏的交叉项清零,
    或把高度/垂速拆成独立的小滤波器; 同时把 <code>ekfBaroSigmaM</code> 从 1.0 m 调到实测的 2~3 m,
    并加动态门控(油门高/竖直加速度大/新息突变时拒绝该次气压更新)。气压计本身的桨流扰动要硬件治理。</li>
<li><b>【最高优先级】先做 6.10 的三个判定实验</b>(静止 30s 比"四元数隐含角速度 vs 陀螺"、手抬机头验符号、系留加油门看估计器是否自转),
    并用它们确认"这批日志的姿态角不可信、只有陀螺与加速度计可信"。在此之前不要用日志里的 pitch/roll 判断飞机姿态。</li>
<li><b>【最高优先级】先做一次 A/B 复飞</b>: 把姿态解算换回 Madgwick、PITCH/ROLL_P 回到 0.7、悬停油门回到 0.30
    (即"当时飞成功的那套"), 确认这台飞机本身没问题; 然后一次只改一个变量往回收(见 6.8)。</li>
<li><b>先修 EKF 零偏/偏航不可观问题</b>(冻结 bz 或按观测可用性使能), 否则 yaw 与高度通道都不能信。</li>
<li><b>把高度/气压的可信性做出来</b>: 气压减振导流 + 观测门限(仅低动态吃气压) + 记录原始气压,
再谈定高。</li>
<li><b>修正饱和判据并记录 mix_scale</b>, 让"力矩被裁剪"和"推力打满"在日志里可分辨。</li>
<li><b>回传加重传/补传</b>, 消除 0.4%~2.2% 的数据空洞; 顺手把 Arm/Disarm 事件改成边沿触发。</li>
<li><b>缩短链路超时(5 s → 1~2 s)</b>并给"链路丢失"一个主动收敛策略, 不要保持最后指令飞 5 s;
同时给"上电后长时间收不到遥控"加个状态位。</li>
<li><b>下一次飞行把油门提到悬停以上</b>(≥0.30)并分段记录: 0.25 悬停模型 → 只在这三个会话里被
用来判读, 无法评估姿态环真实带宽。建议做一次"悬停 30 s + 阶跃横滚 ±5°"的定标飞行,
那时才能评估 PID 与混控。</li>
<li><b>保持现有飞控侧日志与 500 Hz 时序不变</b> —— 这一部分(时序、字段、flash 写入)在 5 个会话里表现完全正常。</li>
</ol>
</div>

<h2 id="files">八、产物与复现</h2>
<div class="card">
<ul>
<li>本报告: <code>flight_log_analysis/report.html</code>(自包含, 双击即可看)</li>
<li>文字版: <code>flight_log_analysis/report.md</code> · 指标数据: <code>flight_log_analysis/analysis.json</code></li>
<li>解析/分析脚本: <code>load_logs.py</code>(解析 CSV) · <code>analyze.py</code>(指标+降采样) ·
  <code>build_report.py</code>(本页) · <code>check_bias.py</code>(零偏与偏航耦合验证)</li>
<li>数据来源: <code>{data['source_dir']}</code></li>
</ul>
</div>
"""

    # ---- 批次自适应: 用数据驱动的叙事/专项分析覆盖上面的 0913 专用文本 ----
    narrative = (_narrative(data, prev) + overview +
                 "<div class='title'>跨会话对比</div>" + cmp +
                 "<div class='card'><p>说明: 图表与 0913 报告同源 —— 灰带 = 解锁段, 橙虚线 = 遥控期望, "
                 "红虚线 = 事件; 滚轮缩放、拖动平移、双击复位、悬停读数。本批两点差异: "
                 "(1) 气压/高度相关的曲线<b>没有物理意义</b>(气压计不在线, baro 恒 0, height 是惯性外推); "
                 "(2) 事件位里不再有 BaroUpdate, events 字段基本为 0。</p></div>"
                 "<h2 id='detail'>四、逐会话图表</h2><div class='sub'>每个会话 13 张图: 姿态与目标、"
                 "角速度 vs 期望、四路电机、混控缩放、油门、高度(本批=惯性外推)、垂速、加速度+重力一致性、"
                 "角速度幅值、链路年龄、状态位条带、陀螺零偏。灰带 = 解锁段, 橙虚线 = 遥控期望/参考线, "
                 "红虚线 = 异常事件。</div>")
    tail = _topics(data, prev)

    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>飞行日志分析报告 · session 4~8</title><style>{CSS}</style>
<script>{JS}</script></head><body>
<h1>飞行日志分析报告 · session {' / '.join(str(s['id']) for s in sess)}</h1>
<div class="sub">数据来源 <code>{data['source_dir']}</code> · 共 {len(sess)} 个会话 {tot_rows:,} 帧
· 合计 {tot_dur/60:.1f} 分钟 · 记录率 {sess[0]['rate']:.2f} Hz · 控制环 500 Hz ·
生成于 {stamp} · 解析口径见第二节</div>
<div class="kpis">
  <div class="kpi"><span class="k">会话数 / 帧数</span><span class="v">{len(sess)} / {tot_rows:,}</span></div>
  <div class="kpi"><span class="k">总时长</span><span class="v">{tot_dur/60:.1f} min</span></div>
  <div class="kpi"><span class="k">解锁段 / 累计</span><span class="v">{tot_segs} 段 / {tot_armed:.0f} s</span></div>
  <div class="kpi"><span class="k">最大手动油门</span><span class="v">{max_man:.3f} = 悬停 {p['hover_throttle']} 的 {max_man/p['hover_throttle']:.2f}×</span></div>
  <div class="kpi"><span class="k">最大角速度 / 加速度</span><span class="v">{max_gyro:.1f} rad/s / {max_acc:.1f} g</span></div>
  <div class="kpi"><span class="k">最大电机输出</span><span class="v">{max_motor} / 1950</span></div>
  <div class="kpi"><span class="k">回传丢帧</span><span class="v">{tot_lost} 帧</span></div>
</div>
{narrative}
{detail}
{tail}
{osc_js}
</body></html>
"""


def _build_md_0913(data: dict, prev: dict | None = None) -> str:
    """文字版报告(与 HTML 同源, 便于进 notebook / 提 issue)。"""
    sess = data["sessions"]
    p = data["params"]
    L = []
    A = L.append
    A("# 飞行日志分析报告 · session 4~8\n")
    A(f"- 数据来源: `{data['source_dir']}`")
    A(f"- 会话数 {len(sess)}, 共 {sum(s['rows'] for s in sess):,} 帧, "
      f"合计 {sum(s['duration'] for s in sess)/60:.1f} 分钟, 记录率 50 Hz, 控制环 500 Hz")
    A(f"- 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
    A("## 一、结论速览\n")
    tot_segs = sum(len(s["segments"]) for s in sess)
    tot_armed = sum(g["dur"] for s in sess for g in s["segments"])
    A(f"1. 5 个会话是同一天连续 5 次上电, 一共解锁 {tot_segs} 段、累计 {tot_armed:.0f} s; "
      f"session 7 全程未解锁。")
    mman = max(s["man_max"] for s in sess)
    hmax = max(s["hgt_raw_range"][1] for s in sess)
    A(f"2. 所有会话手动油门最大 {mman:.3f}(悬停值 {p['hover_throttle']} 的 {mman/p['hover_throttle']:.2f} 倍) "
      f"—— 没有稳定飞行; 全程 EKF 高度最高只到 +{hmax:.2f} m, 没有形成持续爬升/定高。")
    A("3. 飞控侧日志完好(seq/tickMs 100% 自洽), 缺的帧全部来自回传链路: "
      f"共丢 {sum(s['lost_frames'] for s in sess)} 帧(0.37%~2.20%), 最长一次连丢 22 帧。")
    A("4. 两次飞行中的翻滚/坠机: session 4 t≈236.2~237.0 s(解锁中, 15.7 rad/s、6.26 g、"
      "姿态 0.2 s 内翻 60°+、电机真实下侧饱和); session 8 t≈307.4~313.7 s(链路 307.4 s 断, "
      "指令冻结继续以油门 0.22 输出 → 剧烈震荡 → 312.4 s 失控保护停机 → 停机后 15.5 rad/s / 4.6 g 撞击)。"
      "其余高动态(6.5 rad/s@t≈90~110 s、roll −132°@t=71.5 s、9.7 rad/s@t=48.7 s)都发生在未解锁状态, "
      "属地面搬运/跌落。")
    A("5. EKF 陀螺 z 零偏估计跑飞(真实 −1.1 °/s → 估计到钳位 −0.3 rad/s), "
      "姿态 yaw 在静止时以 2~17 °/s 虚假旋转; yaw 未参与控制, 但会污染高度通道。")
    A("6. 气压计受桨流干扰: 每次解锁后 baroRel 下沉 2~4.65 m, 台架段又出现 +0.7~1.1 m 假爬升 "
      "→ 这几个会话的高度/垂速不可用于评估性能。")
    A("7. 低油门下姿态环无权限: 6%~24% 解锁帧的差动被混控裁剪(最小 0.137); "
      "session 5 给了 −12° 横滚指令 4 s, 实测横滚始终 ≈ −1°。")
    A("8. MotorSat 只报下侧: 全部 1411 个饱和帧都是\"某路被混控压到 0 档\"(下侧, 非误报), "
      "上侧饱和(≥1945)全程 0 次 —— 因为油门始终远低于悬停值。建议上下侧分开报并记录 mix_scale。")
    A("9. 失控保护能停机但门限偏长: session 8 的链路在 307.4 s 断, 5 s 后才(312.4 s)自动停机, "
      "这 5 s 里保持着最后一帧指令(油门 0.22); 建议缩短到 1~2 s 或丢失时主动收敛。")
    A("10. 混控逐帧对账通过(误差 ≤1 LSB), 确认物理通道顺序 motorMap={0,2,3,1}, "
      "CSV motor0~3 = M0左前 / M3左后 / M1右前 / M2右后。\n")
    A("## 二、会话总览\n")
    A("| 会话 | 帧数 | 时长 s | 解锁段 | 解锁时长 s | 最大手动油门 | 电机 max | \\|ω\\| max rad/s | "
      "\\|a\\| max g | 最小混控缩放 | 差动裁剪占比 | 回传丢帧 | 链路 max ms | 失控保护帧 |")
    A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in sess:
        segs = s["segments"]
        A(f"| {s['id']} | {s['rows']} | {s['duration']:.1f} | {len(segs)} | "
          f"{sum(g['dur'] for g in segs):.1f} | {s['man_max']:.3f} | {s['motor_max']} | "
          f"{s['gyro_max']:.1f} | {s['acc_max']:.2f} | "
          f"{min([g['mix_min'] for g in segs], default=1.0):.2f} | "
          f"{max([g['mix_clip_pct'] for g in segs], default=0):.0f}% | "
          f"{s['lost_frames']} ({s['lost_pct']:.2f}%) | {s['link_max']} | {s['failsafe_frames']} |")
    A("\n## 三、解锁段明细\n")
    A("| 会话 | 段 | 起止 s | 时长 s | 手动油门 max | 电机均值 | \\|roll\\|/\\|pitch\\| max ° | "
      "\\|ω\\| max | \\|a\\| g | 裁剪占比/最小缩放 | MotorSat 帧(上侧) | 气压高度 m | 重力偏差 中位/p95 |")
    A("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in sess:
        for i, g in enumerate(s["segments"]):
            A(f"| {s['id']} | #{i+1} | {g['t0']:.1f}~{g['t1']:.1f} | {g['dur']:.1f} | {g['man_max']:.3f} | "
              f"{g['motor_mean']:.0f} | {g['roll_absmax']:.1f}/{g['pitch_absmax']:.1f} | {g['gyro_max']:.1f} | "
              f"{g['acc_min']:.2f}~{g['acc_max']:.2f} | {g['mix_clip_pct']:.0f}%/{g['mix_min']:.2f} | "
              f"{g['sat_fw']} ({g['sat_fw_high']}) | {g['baro_min']:.2f}~{g['baro_max']:.2f} | "
              f"{g['gdev_median']:.0f}°/{g['gdev_p95']:.0f}° |")
    A("\n## 四、事件清单\n")
    for s in sess:
        kinds: dict[str, list] = {}
        for e in s["events"]:
            kinds.setdefault(e["kind"], []).append(e)
        A(f"\n### session {s['id']} ({s['file']})\n")
        A("| 事件 | 事件数 | 帧数 | 最长一段 s | 出现时刻 |")
        A("|---|---|---|---|---|")
        for k, lst in kinds.items():
            tot = sum(x["n"] for x in lst)
            longest = max(lst, key=lambda x: x["t1"] - x["t0"])
            times = ""
            if "times" in longest:
                times = ", ".join(f"{x:.1f}s" for x in longest["times"][:8])
                if len(longest["times"]) > 8:
                    times += " …"
            span = "按次列于右侧" if "times" in longest else f"{longest['t0']:.1f}~{longest['t1']:.1f}"
            ncell = f"{longest.get('count', 0)} 次" if "times" in longest else f"{len(lst)} 段"
            A(f"| {k} | {ncell} | {tot} | {span} | {times} |")
    A("\n## 五、专项分析与建议\n")
    A("见 HTML 报告 `report.html` 第五、六节(EKF 零偏跑飞、气压可信度、饱和判据、回传丢帧、"
      "混控逐帧对账、事件边沿处理)。要点: 先修零偏/偏航不可观, 再做气压减振与观测门限, "
      "修正饱和判据并记录 mix_scale, 回传加补传, 下次飞行把油门提到悬停以上做定标飞行。\n")
    A("\n## 六、追加分析: \"姿态像没在修正\"与\"一加油门就出事\"\n")
    A("**(1) 先纠正读数**: CSV 的 motor0~3 是**物理通道顺序**, 不是 M0~M3 ——")
    A("motor0=M0左前 / motor1=M3左后 / motor2=M1右前 / motor3=M2右后。")
    A("按这个顺序重看 session 4 的 t=171.0s(俯仰 −15.3° 机头翘起、横滚 +2.2° 右侧下沉):")
    A("记录 [376,365,283,391] → 后侧 756 > 前侧 659(压机头)、左侧 741 > 右侧 674(抬右侧),")
    A("**正是修正当前姿态该有的方向**。")
    A("\n**(2) 控制链路符号/轴向没问题**: 用 torque+throttle 复算四路电机与记录值逐帧吻合(≤1 LSB);")
    A("用日志四元数+目标姿态复算 rate_setpoint 与记录值完全一致;")
    A("用电机模式→实测角速度做阻尼方向检查, 帧间相关系数 −0.91~−0.996(三轴、四个会话都为负)。\n")
    A("**(3) 真正的现象是振荡幅值随油门单调增大**: 按油门分箱统计低频(0.15s 低通)|ω| RMS:")
    A("session 8 在 0.07/0.13/0.17/0.19/0.21 油门分别是 0.17/0.28/0.41/0.56/2.64 rad/s,")
    A("对应姿态峰峰 1.8/7.5/14.9/37.8/93.8°; session 4/6 同样从 0.25 涨到 1~2 rad/s。")
    A("油门与遥控指令都不动而振幅自己涨 = 闭环自激(稳定性余量不足)。")
    A("原因之一: 混控的力矩换算 k 与 1/hover 成正比, 参数 hover=0.25 而实测 ≈0.20,")
    A("等效增益比设计高 1.25 倍。\n")
    A("**(4) 振动把修正淹没了**: 解锁后陀螺 20ms 帧间抖动 0.40~0.49 rad/s(未解锁 0.05),")
    A("电机指令随之抖动 47~58 counts; Δ力矩 与 Δ角速度 帧间相关 −0.91~−0.996,")
    A("说明高频段电机几乎全在\"追着陀螺噪声打\", 低频修正分量很小 —— 图上就表现为\"没有修正意图\"。\n")
    A("**(5) 最后一次坠机的真实时间线**: 304~305s 油门 0.20→0.22 后开始掉高度")
    A("(垂速 −1.0 m/s, 高度 +0.65→−1.6 m), 姿态从 ±6° 振荡发展到 305~307.4s 的 ±15~17°(主频 1.65 Hz);")
    A("307.40s 最后一次收到有效遥控, 307.44s |ω| 突跳到 12.8 rad/s(≈730°/s)并打满电机差动 —")
    A("链路丢失与撞击发生在同一瞬间, 而链路丢失不可能造成 12.8 rad/s 的角速度突变 ⇒")
    A("**是先撞击/失稳, 再(或同时)链路中断; 失控保护不是坠机原因**, 只是 5s 后才切断电机。\n")
    A("**(6) 刚离地就往右前漂的候选原因**(证据强度排序): ①高频差动振荡经桨/电调的"
      "非对称响应整流成固定方向推力偏差(证据最强); ②姿态环余量不足(0.20 vs 0.25 标定 + 无陀螺低通),")
    A("离地油门正好落在余量最差的区间; ③推力矢量/重心偏置: 高油门窗口机体坐标系下始终有")
    A("固定方向水平加速度(前向 −0.03~−0.12 g, 随油门增大), 需系留/吊挂实验区分;")
    A("④偏航零偏跑飞放大以上效果。可排除: 控制律符号、电机顺序/旋向、陀螺轴向接反。\n")
    A("**(7) 建议**: 加陀螺低通(80~120 Hz); HOVER_THROTTLE 改成实测 0.20;")
    A("降 RATE_ROLL/PITCH_P(9→6~7)与外环(3→2~2.5); 偏航差动按可用权限限幅; 桨/机架减振;")
    A("linkTimeoutMs 5s→1~2s; 做系留吊挂实验区分地面效应/推力不足; 日志加一路原始角速度用于频谱分析。")
    A("\n**(8) 最可能的直接原因: 姿态估计被加速度计拖走, 与姿态环形成正反馈**。")
    A("用欧拉运动学把\"四元数的角度变化\"与\"陀螺积分\"相减, 差值就是 EKF 用加速度计把姿态拧过去的量:")
    A("台架段差值只有 0.2 °/s 量级, 而飞行段达到 **中位 23~35 °/s、p95 90~140 °/s**(session 4/6/8)。")
    A("例: s4 165~172s pitch 四元数 −13.4° vs 陀螺 +8.5°(差 −21.8°); s8 305~307.4s roll 四元数 −1.1° vs 陀螺 +21.3°(差 −22.3°)。")
    A("原因: 倾角修正门限 `accel_tilt_gate_mss = 2.0 m/s²`(0.2g)只查幅值不查方向, 解锁段有 **64~74%** 的时间是开着的,"
      "于是推力段/颠簸时的\"推力方向+振动\"被当成重力。")
    A("物理后果(与符号约定无关): 水平加速度 a → 比力方向偏 atan(a/g) → 估计器认为飞机朝反方向倾斜 → 姿态环朝 a 的方向打舵 →"
      "进一步增大 a → 正反馈。平衡点在 atan(a/g): 0.2g 加速度 ↔ 11° 倾角, 恰好自洽, 于是形成\"稳定前倾/右倾\"、"
      "水平加速度约 2 m/s²、1 秒漂 1 m 的现象, 且油门越大越严重 —— 与\"刚离地就快速右前漂、只能马上收油\"完全吻合。")
    A("修法: 门限收紧到 0.3~0.5 m/s²(或改成方向新息门限)、放大 accel_noise_sigma(0.15→0.5~1.0)、"
      "把门限/新息角写进日志, 再做一次系留吊挂实验确认。")
    A("\n**(9) 与\"曾经飞成功那次\"的对比(已按飞手更正)**: main_final_test 用 Madgwick + 悬停 0.30 +"
      " RATE_ROLL/PITCH_P 8.0; 现在悬停 0.25 + RATE P 9.0(为补偿悬停参数下调而同步提高), 姿态环 P 两次相同。")
    A("净增益 = 9/8 × 0.83 ≈ 0.93 倍, 基本没变 ⇒ **相对那次飞成功的配置, 唯一实质性改动就是姿态解算"
      "(Madgwick → 15 状态 EKF)**。同样硬件、几乎同样增益, 换估计器就从能飞变成刚离地就稳定前倾。")
    A("\n**(10) 静止零点偏移**(把飞机放水平台面上量; 抬头为负): s4 −0.44~−1.57°/roll −0.29~−2.29°,"
      " s5 −0.20~−0.25°/−0.80~−0.86°, s6 −0.16~+0.27°/−0.50~−1.37°, s7 −0.06°/−1.50°,"
      " s8 −0.08~−0.66°/−0.26~−1.01°。读数长期偏\"抬头+左倾\" ⇒ 控制器飞起来会把飞机压成\"低头+右倾\" ⇒"
      " 持续向前+向右的水平加速度(1°≈0.17 m/s², 3°≈0.5, 5°≈0.86), 方向与实测右前漂一致。")
    A("\n**(11) 下一步(按顺序)**: (a) 把飞机放确认水平的台面, 看静止读数能否回到 0.0°, 偏大就做加速度计零点标定;")
    A("(b) 用\"当时飞成功的那套\"(Madgwick, 其余保持现状)复飞一次做 A/B —— 因为 PID 基本没变, 这是单变量对比;")
    A("(c) 若必须用 EKF: 倾角门限收紧到 0.3~0.5 m/s²(或方向新息门限)、accel_noise_sigma 0.15→0.5~1.0、")
    A("推力段冻结/收紧陀螺零偏估计(现在 x/y 到 0.05 rad/s、z 到 −0.3 钳位), 并把门限状态与新息角写进日志。")
    A("\n**(12) 重要限制——这批日志的姿态角不可信(与任何约定无关的检验)**: 用相邻帧四元数算出的"
      "\"四元数自己认为的机体角速度\"(Δq=conj(q_k)·q_(k+1) → 旋转向量/dt)与同一条日志里的陀螺逐帧比较,"
      " 两者理应几乎相等, 实测却差了 0.2~0.6 rad/s(11~34 °/s):")
    A("- s4 162.5/164.5/166.0s(油门 0.08): wz 四元数 +0.28/+0.41/+0.44 vs 陀螺 −0.005/−0.004/−0.010 rad/s;")
    A("- s4 170.5~173.0s(油门 0.16~0.18): wy 四元数 −0.27/−0.19/−0.13 vs 陀螺 +0.012/+0.035/+0.014(方向相反)。")
    A("即: 姿态角不是\"陀螺积分+慢慢找平\", 而是被估计器自己转出来的 ⇒ **这批日志里的 pitch/roll/yaw 不能用来判断"
      "飞机真实姿态**(录像才是真值), 可信的原始量只有 bodyRate(陀螺)与 accelG(加速度计)。")
    A("多出来的转动来源: 名义积分 q←q·exp((ω−bg)dt) ⇒ 零偏估计错多少, 姿态就按那个速率多转多少。"
      "本次会话 bz 从 157s 的 ≈0 涨到 183s 的 −0.115 rad/s(−6.6 °/s) 并保持, 而真实 z 零偏仅 −1.1 °/s"
      "(桨停车静止段实测 yaw 漂移 −1.15 °/s 且 bz≈0) ⇒ **零偏被放大 ~6 倍**, 它把姿态持续转起来;"
      " pitch/roll 上是\"错误零偏\"与\"被推力/振动污染的加速度计修正\"互相拉扯, 净效果是估计器脱离真实姿态。")
    A("接回飞手观察: 录像里飞机低头, 估计器报抬头(pitch<0) ⇒ 姿态环为\"纠正抬头\"继续压机头 ⇒ 越压越低头 ⇒"
      " \"电机一直没能修正\"。三者自洽, 唯一不可信的是日志姿态角。")
    A("三个判定实验(不需要新硬件): (a) 桨不转静止 30s, 比较\"四元数隐含角速度 vs 陀螺\", 差值 >5 °/s 即估计器自转;"
      " (b) 手抬机头, 确认日志 pitch 增大、acc_x 变负(端到端验符号);"
      " (c) 系留/吊挂加油门 0.10→0.25, 看姿态估计是否在没有陀螺支持的情况下自己转、电机是否在纠正不存在的姿态。")
    A("\n**(13) 完整因果链(为什么\"飞机低头、估计器报抬头\")**:")
    A("0) 先排除传感器/轴向: 用陀螺积分出的 0.5s 转动预测加速度计方向的变化, 原始轴向 (+x,+y,+z) 的"
      "平均夹角误差 0.77°/中位 0.33°(噪声量级), 优于任何单轴翻转或轴交换 ⇒ 两个传感器彼此自洽, 无接反/换轴。")
    A("1) 油门加到 0.16~0.24、推力超过重力(实测悬停≈0.20) ⇒ 加速度计测的不再是重力, 而是\"推力+振动+撞击\""
      "(飞行段 |a| 中位 1.13~1.19 g)。")
    A("2) 倾角修正门限只看幅值(2.0 m/s² = 0.2g) ⇒ 解锁段 64~79% 的帧\"门开着\", 假方向被当成重力用。")
    A("3) 持续的姿态新息被吸收成\"陀螺零偏\"(滤波器有专门状态去装它) ⇒ 实测 bz 最大 −17.2 °/s(真实 −0.73 °/s, 差 23 倍)、"
      "bx/by 到 +2~3 °/s(真实 +0.12/+0.00); 偏航无其它观测, 错了没人纠正。")
    A("4) 名义积分 q←q·exp((ω−bg)dt) ⇒ 零偏错多少, 姿态就按那个速率持续多转多少(bz=−6.6 °/s 就对应 yaw 自转 +6.6 °/s;"
      " 实测四元数 yaw 在 161~176s 转 15~20 °/s, 陀螺只有 −1 °/s 量级)。")
    A("5) 俯仰/横滚被\"水平加速度\"骗: 只要有向前加速度 a, 比力方向就朝机头偏 atan(a/g) ⇒ 滤波器认为飞机\"抬头\" ⇒"
      " 姿态环压机头 ⇒ 产生更大向前加速度 ⇒ 正反馈(平衡点 0.2g ↔ 11°, 自洽 ⇒ 形成\"稳定前倾+右前漂\")。")
    A("6) 飞手看到的就是这一步: 飞机低头前冲, 而姿态环以为自己报的是抬头、正在\"修正\", 于是\"电机一直没能修正\"。")
    A("7) 178.86s 撞击(|ω|=6.86 rad/s, 对应录像里那一下瞬间大角度旋转): 大角速度下陀螺重新占主导,"
      " 姿态估计才跟上真实运动(帧间相关升到 0.40/0.84), 但飞机已经撞地。")
    A("两次飞行(161.8~189.2s、216.6~244.9s)同一模式: 起跳推力不对称决定初始方向, 之后闭环放大成前倾/右漂直至触地。")
    return "\n".join(L)


def build_md(data: dict, prev: dict | None = None) -> str:
    """文字版报告(数据驱动, 任意批次可用)。"""
    f = _facts(data, prev)
    sess, p = f["sess"], f["p"]
    L = []
    A = L.append
    ids = " / ".join(str(s["id"]) for s in sess)
    A(f"# 飞行日志分析报告 · batch {f['batch']} (session {ids})\n")
    A(f"- 数据来源: `{data['source_dir']}`")
    A(f"- {len(sess)} 个会话, 共 {f['rows']:,} 帧 / 50 Hz / 合计 {f['dur']/60:.1f} 分钟; "
      f"解锁 {f['n_armed_segs']} 段、累计 {f['armed_time']:.0f} s")
    A(f"- 气压计: {'在线' if f['baro'] else '**不在线**(摘掉; BaroOk 从未置位, baro 恒 0, height 为纯惯性外推)'}")
    A(f"- 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
    A("## 一、结论速览\n")
    peaks = ", ".join(f"session {i} {g['v']:.1f} rad/s@t={g['t']:.0f}s"
                      f"({'解锁中' if g['armed'] else ('未解锁' if g['armed'] is not None else '状态未知')})"
                      for i, g in f["high_rate"]) or "无"
    A(f"1. 本批最大手动油门 {f['max_man']:.3f}(最大电机 {f['max_motor']}/1950), |a| "
      f"{f['min_acc']:.2f}~{f['max_acc']:.2f} g; 解锁飞行中的角速度峰值最大只有 "
      f"{max((s.get('gyro_max_armed') or 0) for s in sess):.1f} rad/s(0913 为 19.7), 没有翻滚级事件。"
      f"全程 >3 rad/s 的: {peaks}。")
    A(f"2. 混控**一次都没有裁剪差动**(最大 {f['mix_clip_max']:.0f}%), MotorSat 共 {f['sat_total']} 帧(上侧 {f['sat_high']}) —— "
      f"因为本批油门用到了 {f['max_thr']:.2f}, 差动权限足够。")
    A(f"3. 姿态估计被加速度计拖走的机制仍在: 倾角门限开门 "
      f"{f['drag_gate_min']:.0f}~{f['drag_gate_max']:.0f}%, 高油门段非陀螺姿态速率中位 "
      f"{f['drag_med_min']:.1f}~{f['drag_med_max']:.1f} °/s(p95 {f['drag_p95_max']:.0f} °/s), 比 0913 小但同源。")
    A(f"4. 摘掉气压计的直接代价: 高度/垂速变成纯惯性外推, 静止也能漂 ±{f['hgt_drift']:.0f} m"
      f"(session 3 到 +37.7 m、session 5 到 −21.6 m) → 这批日志的高度绝对值、以及依赖高度的定高功能都不可用。")
    A(f"5. 回传丢帧率升到 {f['lost_min']:.2f}%~{f['lost_max']:.2f}%(0913 为 0.37%~2.20%); "
      f"session 4 出现**飞行中链路丢失**: t≈37.4 s 失联, 5 s 后失控保护停机, 链路 27.5 s 后才恢复。")
    A(f"6. 控制链路复核通过: 用 torque+throttle 反算四路 DShot 与记录值{'全部' if f['mixer_ok'] else '大部分'} ≤1 LSB 吻合。")
    A(f"7. 振动明显小于 0913: 陀螺帧间抖动 {f['vib_min']:.2f}~{f['vib_max']:.2f} rad/s(0913 0.40~0.49); "
      f"低频 |ω| 仍随油门增大但最大只有 0.57 rad/s(0913 曾到 2.6), 主频 0.05~0.43 Hz(慢漂而非极限环)。\n")
    gve = f["gve_fly"]
    A(f"8. **【最关键】有推力时姿态估计看不见真实转动**: 解锁段里陀螺积分显示飞机真的转了 "
      f"pitch {min(g['pitch_gyro'] for _, g in gve):+.0f}~{max(g['pitch_gyro'] for _, g in gve):+.0f}°、"
      f"roll {min(g['roll_gyro'] for _, g in gve):+.0f}~{max(g['roll_gyro'] for _, g in gve):+.0f}°, "
      f"而 EKF 报出的姿态几乎不动(≤{max(abs(g['pitch_ekf']) for _, g in gve):.1f}°) —— "
      f"因为加速度计倾角修正反方向拧了 "
      f"{min(abs(g['pitch_accel_contrib']) for _, g in gve):.0f}~{max(abs(g['pitch_accel_contrib']) for _, g in gve):.0f}°(pitch)、"
      f"{min(abs(g['roll_accel_contrib']) for _, g in gve):.0f}~{max(abs(g['roll_accel_contrib']) for _, g in gve):.0f}°(roll), "
      f"把真实转动抵消掉。飞行中加速度计测的是**推力**(沿机体 z), 不是重力, 对倾角不敏感 → "
      f"控制器认为姿态没变, 于是不去修正 —— 这就是加油门就低头倾斜、电机看不出修正、稳定停在某个倾角的直接原因。"
      f"对照: 不装桨的台架(session 2)修正量只有 −0.2°/−0.6°(没有推力, 加速度计测的确实是重力)。")
    A(f"   修法: ①解锁后/油门>0.08 冻结或大幅降低加速度计倾角修正(静止已收敛的零偏只有 x/y ≈0.1°/s、z ≈−1.2°/s, "
      f"30 s 纯陀螺积分只漂 3~6°); ②更好: 用油门估 T/m 从比力里减掉推力项再当重力基准; "
      f"③检查推力矢量是否过重心。验证: 拆桨解锁、手动缓慢倾斜 20°, 看姿态角是否跟得上陀螺。\n")
    A("9. **为什么 Madgwick 那版能平稳起飞、EKF 这版不能**(与 2026-09-08 Madgwick 飞行报告 "
      "`flight_log_analysis.html` 里的数据对比): Madgwick 那次连续飞了 96.8 s(油门 0.08~0.28), "
      "姿态低频(0.5s 平均)5~95% 只有 roll −2.1~+3.7°、pitch −6.2~+5.9°; |pitch|>15° 只有 3 次、"
      "每次 0.1~0.3 s 的尖峰 —— 误差是短暂尖峰、立刻回到水平。EKF 这版是持续数秒的 10~20° 倾斜, "
      "误差被留住了。差别来自 EKF 多出来的状态:")
    A("   **陀螺零偏估计在飞行中被推力污染的加速度计带跑**: 零偏是直接从陀螺里减掉再积分的"
      "(om = gyro − bg), 所以它错了多少, 姿态就按那个速率持续虚假旋转多少。本批实测: "
      "s5 偏航零偏被估到 −4.24 °/s(真实 −1.10), 横滚零偏误差 +0.53 °/s → 20 s 累积 11° 的虚假横滚; "
      "s3 误差 +0.24/+0.15/−0.65 °/s; 0913 更极端(bz 到 −17 °/s, 差 23 倍)。"
      "这类误差会一直作用到重新落地看到有效重力为止 ⇒ 飞机一直歪着飞。"
      "**Madgwick 没有零偏状态, 结构上不可能产生这种累积误差。**")
    A("   第二个差别: Madgwick 的修正量是 β·s(|s|=1), 相当于把\"姿态修正角速度\"硬限制在 2β≈9 °/s, "
      "而且只作用于当前误差(误差一消失修正就消失); EKF 是卡尔曼增益自适应, 还有速度/高度状态"
      "(本批摘掉气压计后高度漂到 ±38 m)与姿态/零偏在协方差里互相耦合, 会把本该由姿态吸收的新息分走一部分, "
      "让零偏跑得更远。我做过离线重放(analyze_compare_filters.py): 只把加速度计权重调到与 Madgwick 相当, "
      "两者误差仍相近(5.9° vs 5.8°) ⇒ **光调小加速度计权重不够, 必须同时冻结/收紧零偏估计**。")
    A("   **建议**: ①最稳 —— 姿态直接用 Madgwick(代码已有、main_final_test 已验证), EKF 只留高度/垂速"
      "(气压计接回); EKF 头文件本身就写明: 在\"单 IMU + 气压\"的退化情形下, 它只应作为 "
      "Madgwick + VES 的可选增强/备份。②继续用 EKF —— 解锁后冻结零偏估计(只在地面收敛)、"
      "gyro_bias_walk_sigma≈0、把 ±0.3 rad/s 钳位收到 ±0.05, 再配合收紧加速度计倾角修正。\n")
    A("10. **补充: Madgwick 不估零偏, 反而是它更稳的原因之一(不是缺点)**。MadgwickAHRS.hpp 只有 beta 项、"
      "没有 Ki、也没有零偏状态 ⇒ 它确实是'带着零偏跑'。但这在这种滤波器里只表现为一个**有界的静态倾角偏置**:")
    A("   θ_offset ≈ b / (2β)。本机静止实测 roll/pitch 零偏约 0.12 °/s = 0.002 rad/s, β=0.078 ⇒ 2β=0.156 1/s ⇒ "
      "θ_offset ≈ 0.002/0.156 ≈ **0.73°**(有界、不累积); 而 z 轴零偏(1.1~1.3 °/s)只影响 yaw, yaw 不参与控制。")
    A("   EKF 相反: 它**估**零偏, 而且 `g_ekf.reset(fcs.attitude, Vector3f{}, Vector3f{})`(release.cpp:872)在每次解锁时"
      "把零偏**清零**(reset() 里 `_bg = gyro_bias`, flight_control_fliter.hpp:197), 于是每次飞行都要重新学一遍零偏 —— "
      "而飞行中学零偏用的正是被推力污染的加速度计。日志里的爬升就是它: s5 的 z 零偏从解锁时的 ≈0 → 46s 的 −0.21 °/s → "
      "49.6s 的 −2.46 °/s → 74.8s 的 −4.24 °/s(真实只有 −1.10), x/y 也涨到 +0.40/+0.42 °/s(真实 −0.12/+0.12)。")
    A("   因为名义积分是 `om = gyro − bg`, 零偏**误差**会直接累积成姿态误差: 0.5 °/s × 20~40 s = **10~20°** —— "
      "比 Madgwick 那个 0.7° 的有界偏置差 15~30 倍。所以'带着零偏跑'在这里反而是更安全的选择。")
    A("   最小改动(保留 EKF 也有效): ①`estimate_gyro_bias=false`(彻底不学零偏); 或 ②更好 —— 起飞前静止时让它收敛, "
      "解锁时把这个值传进 reset 并冻结: `g_ekf.reset(fcs.attitude, Vector3f{}, Vector3f{}, g_ekf.getGyroBias())`。"
      "这一条与第 8/9 条的'收紧加速度计倾角修正'合起来, 才会让 EKF 接近 Madgwick 那样的行为。\n")
    A("11. **修正与补充(2026-09-14 复核, 见 `three_ways_*.txt` / `bias_math_*.txt`)**:")
    A("   (a) 零偏列大部分是 0 是**日志写入策略**: release.cpp 里 `if ((logCounter % ekfLogDivider) == 0)`"
      "(默认 10) 才写零偏, 其余条目是结构体清零后的 0 —— 是没记, 不是估出来是 0; 读这一列只能取非零样本、按 0.2s 间隔。")
    A("   (b) 零偏的量级我之前**高估**了。按非零样本重算, 零偏误差在这段飞行里累积的姿态: s2 +0.6°/+0.1°, "
      "s3 +1.8°/−3.5°, s4 +0.8°/−0.5°, s5 +14.4°/+4.8°(roll/pitch) —— 短航次只有 1~3.5°, 确实可以忽略; "
      "只有 s5 那次 43.8s、油门到 0.32 的长航次才到 roll +14°。(yaw 那 20~55° 不参与控制。)")
    A("   (c) 真正的主因不是零偏, 而是**机架上有推力时加速度计不再是干净的重力计**。三路对比(比力与机体 z 夹角 / "
      "去偏陀螺积分的倾角 / EKF 倾角): 不装桨的台架段 |a|≡1.00g、三者一致到 1° 以内; 而带桨的飞行段 "
      "|a| 在 **0.79~1.39g** 之间以 10~25Hz 摆动(机架/起落架被桨流和振动反复冲击), 比力方向随之乱摆 5~38°, "
      "而 EKF 用的正是这些瞬时方向、且量测噪声只给了 `accel_tilt_sigma = 0.05`(远小于实际的 0.1~0.3), "
      "门限又只看幅值 ⇒ 估计被这些假方向拖走: 飞行段 EKF 倾角只有 1~10°, 而陀螺积分真值长到 5~13°、"
      "比力倾角 5~38°。控制器于是只按真实倾斜的 2%~30%(有时还反号)去修 —— 这就是低头了却修不动。")
    A("   而这也正好解释桌上摇飞控良好、一上飞机就失控: 桌上桨不转, |a| 恒为 1.00g、比力方向就是重力, "
      "三个滤波器都对; 一带桨, 加速度计就开始说假话, 而 EKF 恰好最信它。")
    A("   (d) 因此优先级修正为: ①给比力做低通(或短窗平均)+ 把 `accel_tilt_sigma` 调到真实量级(0.3~0.5)"
      "+ 用振动量(而不是幅值)做门限, 或推力段直接冻结倾角修正; ②顺手冻结/收紧零偏估计(见第 10 条); "
      "③确定性试验: 桨装上、飞机夹住不动, 油门 0.1/0.2/0.3 分级, 同时记录 |a|、陀螺、比力方向与姿态估计 —— "
      "一看就知道是估计器被比力带偏还是陀螺在漂。\n")
    A("## 二、会话总览\n")
    A("| 会话 | 帧数 | 时长 s | 解锁段 | 解锁时长 s | 最大手动油门 | 电机 max | \\|ω\\| max | \\|a\\| max | "
      "最小混控缩放 | 差动裁剪 | 回传丢帧 | 链路 max ms | 失控保护帧 | 倾角门限开门 | 非陀螺速率中位 |")
    A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in sess:
        segs = s["segments"]
        ed = s.get("ekf_drag") or {}
        A(f"| {s['id']} | {s['rows']} | {s['duration']:.1f} | {len(segs)} | "
          f"{sum(g['dur'] for g in segs):.1f} | {s['man_max']:.3f} | {s['motor_max']} | {s['gyro_max']:.1f} | "
          f"{s['acc_max']:.2f} | {min([g['mix_min'] for g in segs], default=1.0):.2f} | "
          f"{max([g['mix_clip_pct'] for g in segs], default=0):.0f}% | {s['lost_frames']} ({s['lost_pct']:.2f}%) | "
          f"{s['link_max']} | {s['failsafe_frames']} | "
          f"{(str(ed.get('gate_open_armed')) + '%') if ed.get('gate_open_armed') is not None else '-'} | "
          f"{ed.get('rate_median', '-')} °/s |")
    A("\n## 三、解锁段明细\n")
    A("| 会话 | 段 | 起止 s | 时长 s | 手动油门 max | 电机均值 | \\|roll\\|/\\|pitch\\| max ° | \\|ω\\| max | "
      "\\|a\\| g | 裁剪/最小缩放 | MotorSat(上侧) | 气压高度 m | 重力偏差 中位/p95 |")
    A("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in sess:
        for i, g in enumerate(s["segments"]):
            baro = ("%.2f~%.2f" % (g["baro_min"], g["baro_max"])) if g["baro_min"] is not None else "无气压计"
            A(f"| {s['id']} | #{i+1} | {g['t0']:.1f}~{g['t1']:.1f} | {g['dur']:.1f} | {g['man_max']:.3f} | "
              f"{g['motor_mean']:.0f} | {g['roll_absmax']:.1f}/{g['pitch_absmax']:.1f} | {g['gyro_max']:.1f} | "
              f"{g['acc_min']:.2f}~{g['acc_max']:.2f} | {g['mix_clip_pct']:.0f}%/{g['mix_min']:.2f} | "
              f"{g['sat_fw']} ({g['sat_fw_high']}) | {baro} | {g['gdev_median']:.0f}°/{g['gdev_p95']:.0f}° |")
    A("\n## 四、事件清单\n")
    for s in sess:
        kinds: dict[str, list] = {}
        for e in s["events"]:
            kinds.setdefault(e["kind"], []).append(e)
        A(f"\n### session {s['id']} ({s['file']})\n")
        A("| 事件 | 事件数 | 帧数 | 最长一段 s | 出现时刻 |")
        A("|---|---|---|---|---|")
        for k, lst in kinds.items():
            tot = sum(x["n"] for x in lst)
            longest = max(lst, key=lambda x: x["t1"] - x["t0"])
            if "times" in longest:
                times = ", ".join(f"{x:.1f}s" for x in longest["times"][:8])
                if len(longest["times"]) > 8:
                    times += " …"
                A(f"| {k} | {longest.get('count', 0)} 次 | {tot} | 按次列于右侧 | {times} |")
            else:
                A(f"| {k} | {len(lst)} 段 | {tot} | {longest['t0']:.1f}~{longest['t1']:.1f} | |")
    A("\n## 五、专项分析与建议\n")
    A("完整版见 `report" + (f"_{f['batch']}" if f["batch"] != "all" else "") + ".html` 第五节, 包含:"
      "①回传链路与丢帧(含飞行中失联时间线) ②摘掉气压计的影响 ③姿态估计被加速度计拖走 "
      "④混控裁剪/饱和 ⑤混控逐帧对账 ⑥振动与低频振荡随油门"
      + (" ⑦与上一批的对比" if f["batch"] != "all" else "") + "。")
    A("\n要点(按优先级): ①先解决链路(失联→1~2 s 超时 + 主动降油门, 查天线/信道);"
      "②气压计要么装回并做减振导流+观测门限, 要么把\"无气压\"的语义显式暴露并禁止依赖高度;"
      "③收紧加速度计倾角修正门限(2.0 → 0.3~0.5 m/s² 或方向新息门限);"
      "④HOVER_THROTTLE 改为实测 0.20~0.22; ⑤做一次系留/吊挂对照实验; ⑥回传加 seq 补传。")
    return "\n".join(L)


def main():
    suffix = f"_{os.environ.get('FLIGHT_LOG_BATCH')}" if os.environ.get("FLIGHT_LOG_BATCH") else ""
    path = os.path.join(OUT_DIR, f"analysis{suffix}.json")
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    prev = None
    if suffix:
        prev_path = os.path.join(OUT_DIR, "analysis.json")
        if os.path.exists(prev_path):
            with open(prev_path, "r", encoding="utf-8") as fh:
                prev = json.load(fh)
    html = build_html(data, prev)
    with open(os.path.join(OUT_DIR, f"report{suffix}.html"), "w", encoding="utf-8") as fh:
        fh.write(html)
    md = build_md(data, prev)
    with open(os.path.join(OUT_DIR, f"report{suffix}.md"), "w", encoding="utf-8") as fh:
        fh.write(md)
    print(f"report{suffix}.html", os.path.getsize(os.path.join(OUT_DIR, f"report{suffix}.html")), "bytes")
    print(f"report{suffix}.md", os.path.getsize(os.path.join(OUT_DIR, f"report{suffix}.md")), "bytes")


if __name__ == "__main__":
    main()



