// 用最小 DOM 桩跑一遍 report.html 里的所有脚本, 捕捉 JS 语法/运行错误。
// 用法: node smoke_test.mjs report.html
import fs from 'node:fs';

const file = process.argv[2] || 'report.html';
const html = fs.readFileSync(file, 'utf8');
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
console.log('inline script blocks:', blocks.length);

function mkEl(tag) {
  const el = {
    tagName: tag, children: [], attrs: {}, style: {}, _text: '', _html: '',
    setAttribute(k, v) { this.attrs[k] = v; },
    getAttribute(k) { return this.attrs[k]; },
    appendChild(c) { this.children.push(c); return c; },
    removeChild(c) { this.children = this.children.filter(x => x !== c); },
    addEventListener() {},
    getBoundingClientRect() { return { left: 0, top: 0, width: 1180, height: 200 }; },
    get firstChild() { return this.children[0] || null; },
    set textContent(v) { this._text = String(v); },
    get textContent() { return this._text; },
    set innerHTML(v) { this._html = String(v); },
    get innerHTML() { return this._html; },
  };
  return el;
}
const byId = new Map();
for (const m of html.matchAll(/id='([^']+)'/g)) byId.set(m[1], mkEl('div'));
const document = {
  createElement: mkEl,
  createElementNS: (ns, tag) => mkEl(tag),
  getElementById: (id) => {
    if (!byId.has(id)) byId.set(id, mkEl('div'));
    return byId.get(id);
  },
};
const window = { addEventListener() {}, BAGS: undefined };
let bad = 0;
// 逐块跑, 但块之间共享同一个函数作用域(等价于浏览器里的全局 const/function 声明)
const runner = new Function('document', 'window', blocks.join('\n;\n'));
try {
  runner(document, window);
} catch (e) {
  bad++;
  console.log(`FAILED: ${e.message}`);
  console.log(e.stack.split('\n').slice(0, 4).join('\n'));
}
console.log(`run ${bad ? 'FAILED' : 'OK'}`);
// 统计生成的 svg 数量
let svg = 0, paths = 0, rects = 0;
for (const el of byId.values()) {
  const walk = (n) => {
    if (!n) return;
    if (n.tagName === 'svg') svg++;
    if (n.tagName === 'path') paths++;
    if (n.tagName === 'rect') rects++;
    (n.children || []).forEach(walk);
  };
  walk(el);
}
console.log(`DOM 桩里生成: svg=${svg} path=${paths} rect=${rects}`);

// 逐图检查: 每个画布容器里有没有生成带坐标的 path, 有没有 NaN
let charts = 0, empty = 0, nan = 0, flat = 0;
for (const [id, el] of byId) {
  const svgs = (el.children || []).filter(c => c.tagName === 'svg');
  if (!svgs.length) continue;
  charts++;
  let pts = 0, hasNaN = false, rc = 0, elems = 0;
  let bx0 = Infinity, bx1 = -Infinity, by0 = Infinity, by1 = -Infinity;
  const walk = (n) => {
    elems++;
    if (n.tagName === 'path') {
      const d = n.attrs.d || '';
      if (d.includes('NaN') || d.includes('undefined')) hasNaN = true;
      pts += (d.match(/L/g) || []).length;
      for (const m of d.matchAll(/[ML]([-\d.]+) ([-\d.]+)/g)) {
        const X = +m[1], Y = +m[2];
        if (X < bx0) bx0 = X; if (X > bx1) bx1 = X;
        if (Y < by0) by0 = Y; if (Y > by1) by1 = Y;
      }
    }
    if (n.tagName === 'rect') rc++;
    (n.children || []).forEach(walk);
  };
  svgs.forEach(walk);
  // 真正算"空图"的判据: 元素太少(轴/网格都没画出来)。散点/常量曲线的图元素很多, 只是几何退化。
  if (elems < 10) { empty++; console.log(`  ${id}: 元素太少(${elems}) path 点 ${pts}, rect ${rc}`); }
  // s7 全程未解锁, 电机恒 50、混控缩放恒 1.0, 画出直线属正常
  if (pts >= 10 && (bx1 - bx0 < 100 || by1 - by0 < 5)) {
    flat++;
    console.log(`  [提示] ${id}: 曲线退化(常量) x[${bx0.toFixed(0)},${bx1.toFixed(0)}] y[${by0.toFixed(0)},${by1.toFixed(0)}]`);
  }
  if (hasNaN) { nan++; console.log(`  ${id}: path 里有 NaN`); }
}
console.log(`图表容器 ${charts} 个, 空图 ${empty} 个, NaN ${nan} 个, 常量曲线 ${flat} 个(仅提示)`);
process.exit(bad || empty || nan ? 1 : 0);
