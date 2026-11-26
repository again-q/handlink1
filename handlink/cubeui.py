"""魔方程视觉页面：http://127.0.0.1:8766/vision

左边是俯视相机的 MJPEG 画面（识别开着时叠了框和 9 格字母），右边是读数和标定按钮。
页面上做的每一步都对应一个接口，脚本/notebook 可以照抄：

    POST /camera/open      {"index":0,"backend":"msmf","width":640,"height":480}
    POST /camera/detect    {"enable":true,"mode":"cube","every":3}
    POST /vision/calibrate {"op":"palette","face":"U"}
    POST /vision/calibrate {"op":"reference"}
    POST /vision/quad      {"points":[[x,y],[x,y],[x,y],[x,y]]}
    GET  /vision/state
"""

HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>handlink 视觉 · 魔方读数</title>
<style>
  :root{--bg:#0b1020;--panel:#141c33;--line:#24304f;--txt:#e6ecff;--dim:#8d9bbd;
        --ok:#4ade80;--warn:#fbbf24;--err:#f87171;--accent:#60a5fa}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
  header{display:flex;align-items:center;gap:10px;padding:14px 20px;border-bottom:1px solid var(--line);flex-wrap:wrap}
  header h1{font-size:16px;margin:0;font-weight:600}
  .badge{font-size:12px;padding:2px 8px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
  .badge.on{color:var(--ok);border-color:#14532d;background:#052e16}
  .badge.off{color:var(--dim)}
  .badge.err{color:var(--err);border-color:#7f1d1d;background:#2a0f0f}
  .wrap{display:grid;grid-template-columns:minmax(420px,1fr) 400px;gap:18px;padding:18px;max-width:1500px;margin:0 auto}
  .col{display:flex;flex-direction:column;gap:14px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}
  .card h2{font-size:13px;margin:0 0 10px;color:var(--dim);font-weight:600;letter-spacing:.04em}
  .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
  button{cursor:pointer;border:1px solid var(--line);background:#1b2440;color:var(--txt);
         padding:7px 12px;border-radius:9px;font-size:13px}
  button:hover{border-color:var(--accent);color:#fff}
  button.primary{background:#1d4ed8;border-color:#2563eb}
  button.danger{background:#7f1d1d;border-color:#b91c1c}
  button:disabled{opacity:.45;cursor:not-allowed}
  select{background:#0e1526;border:1px solid var(--line);color:var(--txt);border-radius:8px;padding:6px 8px}
  #stage{position:relative;background:#05080f;border-radius:12px;overflow:hidden;min-height:300px;
         display:flex;align-items:center;justify-content:center}
  #view{display:block;width:100%;height:auto;max-height:66vh;object-fit:contain}
  #ph{position:absolute;inset:0;display:flex;flex-direction:column;gap:6px;align-items:center;
      justify-content:center;color:var(--dim);font-size:13px;text-align:center;padding:20px}
  /* hidden 的 UA 样式会被上面的 display:flex 盖掉，画面出来以后占位层还挂在上面 */
  #ph[hidden]{display:none}
  .muted{color:var(--dim);font-size:12px}
  .grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:260px;margin:0 auto}
  .cell{aspect-ratio:1;border-radius:8px;display:flex;align-items:center;justify-content:center;
        font-weight:700;font-size:18px;border:1px solid #0004;color:#111}
  .cell.q{background:#1b2440;color:var(--dim);border:1px dashed var(--line)}
  .angle{text-align:center;margin:12px 0 6px}
  .angle b{font-size:34px;font-variant-numeric:tabular-nums}
  table{width:100%;border-collapse:collapse;font-size:13px}
  th,td{text-align:left;padding:4px 4px;border-bottom:1px solid #1d2743}
  th{color:var(--dim);font-weight:500;width:40%}
  td{font-variant-numeric:tabular-nums}
  #log{height:104px;overflow:auto;font:12px/1.5 ui-monospace,Consolas,monospace;color:#9fb0d0;white-space:pre-wrap}
  #log b{color:var(--ok)} #log i{color:var(--err);font-style:normal}
  a{color:var(--accent);text-decoration:none}
  .bar{height:6px;background:#1d2743;border-radius:99px;overflow:hidden;margin:4px 0 10px}
  .bar>i{display:block;height:100%;background:linear-gradient(90deg,#60a5fa,#a78bfa);width:0}
</style>
</head>
<body>
<header>
  <h1>魔方视觉</h1>
  <span class="badge off" id="bCam">相机未开</span>
  <span class="badge off" id="bRecog">识别未开</span>
  <span class="badge" id="bPal">调色板：标准</span>
  <span class="badge" id="bRef">基准：未设</span>
  <span class="muted" style="margin-left:auto">俯视相机 · 读 U 面 9 格颜色 + 层转角</span>
</header>

<div class="wrap">
  <div class="col">
    <div class="card" style="padding:10px">
      <div id="stage">
        <img id="view" alt="俯视画面" hidden>
        <div id="ph">相机未打开<br><span class="muted">点下面的「打开相机」</span></div>
      </div>
      <div class="row" style="margin:10px 0 0">
        <button class="primary" id="btnOpen" onclick="openCam()">打开相机</button>
        <button class="danger" id="btnClose" onclick="closeCam()" disabled>关闭</button>
        <button id="btnRecog" onclick="toggleRecog()">开始识别</button>
        <button onclick="shot()">下载标注图</button>
        <span class="muted" id="hint" style="margin-left:auto"></span>
      </div>
    </div>

    <div class="card">
      <h2>读数（U 面，行优先）</h2>
      <div class="grid" id="grid"></div>
      <div class="angle"><b id="angle">--</b><span class="muted"> ° 相对基准（正=顺时针）</span></div>
      <div class="muted" id="quality">还没读数。</div>
      <table style="margin-top:8px">
        <tr><th>面中心 / 格子边长</th><td id="sCenter">-</td></tr>
        <tr><th>内部格线 / 颜色命中</th><td id="sGap">-</td></tr>
        <tr><th>综合分 / 来源</th><td id="sScore">-</td></tr>
        <tr><th>识别用时</th><td id="sMs">-</td></tr>
      </table>
    </div>
  </div>

  <div class="col">
    <div class="card">
      <h2>标定</h2>
      <div class="row">
        <span class="muted">学一个面：</span>
        <select id="face">
          <option value="U">U 白</option><option value="R">R 红</option>
          <option value="F">F 绿</option><option value="D">D 黄</option>
          <option value="L">L 橙</option><option value="B">B 蓝</option>
        </select>
        <button onclick="learnFace()">学习这个面的颜色</button>
      </div>
      <div class="row">
        <button onclick="setRef()">设托盘基准</button>
        <button id="btnQuad" onclick="pickQuad()">手动框四个角</button>
        <button onclick="clearQuad()">恢复自动</button>
      </div>
      <div class="row">
        <button class="danger" onclick="resetAll()">清空调色板与基准</button>
      </div>
      <div class="muted" id="calHint">六面都学一遍，颜色最准；换灯光/换相机要重学。</div>
    </div>

    <div class="card">
      <h2>接进仿真（桌面平面）</h2>
      <div class="row">
        <button id="btnTable" onclick="pickTable()">点桌面四角 → 标定</button>
        <select id="rot">
          <option value="0">方向 0°</option>
          <option value="90">方向 90°</option>
          <option value="180">方向 180°</option>
          <option value="270">方向 270°</option>
        </select>
        <label class="muted"><input type="checkbox" id="mir"> 左右镜像</label>
      </div>
      <div class="row" style="margin-top:6px">
        <input id="objName" value="料盒" style="width:84px" title="物体名称">
        <select id="objKind">
          <option value="box">方块</option>
          <option value="cylinder">圆柱</option>
        </select>
        <button id="btnPlace" onclick="pickPlace()">点画面里的物体 → 加进仿真</button>
      </div>
      <div class="muted" style="margin-top:4px">
        默认方块 8×8×6 cm、圆柱 ⌀4×10 cm；点物体与桌面接触的位置。更精确的尺寸用
        <span class="mono">POST /scene/place</span>。
      </div>
      <div class="muted" id="sceneHint">
        按画面左上 → 右上 → 右下 → 左下 的顺序点四个桌角。标定后相机看到的物体会
        进 /arm3d 的碰撞模型（橙色方块），桌面还会叠一层相机画面用来核对对位。
      </div>
    </div>

    <div class="card">
      <h2>怎么用</h2>
      <ol class="muted" style="margin:0;padding-left:18px">
        <li>相机装在托盘正上方，画面里魔方占 1/4 左右，别让强光直射贴纸。</li>
        <li>把复原的魔方六个面依次朝上，各点一次「学习这个面的颜色」。</li>
        <li>把魔方摆正（U 面与托盘边平行），点「设托盘基准」。</li>
        <li>之后每次读数：9 格字母 + 相对基准的转角（±45°）。</li>
        <li>识别不稳时先看「内部格线」这一项；掉到 0.5 以下基本是没框准或光不够。</li>
      </ol>
    </div>

    <div class="card">
      <h2>接口日志</h2>
      <div id="log">等待操作…</div>
    </div>
  </div>
</div>

<script>
var COLORS = {U:'#f2f2f2', R:'#d02a2a', F:'#2aa84a', D:'#ffd60a', L:'#f07a1a', B:'#1a5ad0'};
var NAMES = {U:'白', R:'红', F:'绿', D:'黄', L:'橙', B:'蓝'};
var streaming = false, picking = false, pickMode = null, corners = [], timer = null;

function log(msg, cls){
  var box = document.getElementById('log');
  var line = document.createElement('div');
  line.innerHTML = cls ? '<' + cls + '>' + msg + '</' + cls + '>' : msg;
  box.prepend(line);
  while (box.children.length > 40) box.lastChild.remove();
}
async function api(path, body){
  var opt = body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)};
  var r = await fetch(path, opt);
  var j;
  try { j = await r.json(); } catch(e){ j = {ok:false, error:'HTTP ' + r.status}; }
  if (j.ok === false) log('<i>✗ ' + path + ' ' + (j.error || ('HTTP ' + r.status)) + '</i>');
  return j;
}

function buildGrid(){
  var g = document.getElementById('grid');
  g.innerHTML = '';
  for (var i = 0; i < 9; i++){
    var d = document.createElement('div');
    d.className = 'cell q';
    d.id = 'c' + i;
    d.textContent = '?';
    g.appendChild(d);
  }
}

async function openCam(){
  var j = await api('/camera/devices?force=1');
  var list = (j.streams || []).filter(function(s){ return s.frame; });
  if (!list.length){ log('<i>没扫到能出图的相机</i>'); return; }
  var s = list[0];
  var r = await api('/camera/open', {index:s.index, backend:s.backend,
                                     width:s.width, height:s.height});
  if (!r.ok){ log('<i>打不开：' + (r.error || '') + '</i>'); return; }
  log('<b>✓ 相机已开</b> ' + s.backend + ' #' + s.index + ' ' + r.width + 'x' + r.height);
  startView();
  await toggleRecog(true);
}
async function closeCam(){
  await api('/camera/detect', {enable:false, mode:'cube'});
  await api('/camera/close', {});
  stopView();
  log('相机已关');
}
function startView(){
  streaming = true;
  var img = document.getElementById('view');
  img.hidden = false;
  document.getElementById('ph').hidden = true;
  img.src = '/camera.mjpg?t=' + Date.now();
  document.getElementById('btnOpen').disabled = true;
  document.getElementById('btnClose').disabled = false;
}
function stopView(){
  streaming = false;
  var img = document.getElementById('view');
  img.hidden = true;
  img.removeAttribute('src');
  document.getElementById('ph').hidden = false;
  document.getElementById('btnOpen').disabled = false;
  document.getElementById('btnClose').disabled = true;
}

async function toggleRecog(force){
  var on = force === true ? false : document.getElementById('btnRecog').dataset.on === '1';
  // 一次识别 100~200 ms，别每帧都跑；每 10 帧取一次画面看着就够流畅了
  var j = await api('/camera/detect', {enable:!on, mode:'cube', every:10});
  if (!j.ok){ log('<i>识别开不起来：' + (j.error || '') + '</i>'); return; }
  document.getElementById('btnRecog').dataset.on = on ? '0' : '1';
  document.getElementById('btnRecog').textContent = on ? '开始识别' : '停止识别';
  log(on ? '识别已停' : '<b>识别已开</b> 每 10 帧跑一次');
}

function shot(){
  var a = document.createElement('a');
  a.href = '/vision/snapshot.png?t=' + Date.now();
  a.download = 'cube-vision-' + Date.now() + '.png';
  a.click();
}

function renderReading(cube){
  var cells = cube.cells || [];
  for (var i = 0; i < 9; i++){
    var d = document.getElementById('c' + i);
    var letter = cells[i];
    if (!letter || letter === '?'){
      d.className = 'cell q'; d.style.background = ''; d.textContent = '?';
    } else {
      d.className = 'cell'; d.style.background = COLORS[letter] || '#555';
      d.textContent = letter;
      d.title = NAMES[letter] || letter;
    }
  }
  document.getElementById('angle').textContent = (cube.angle === undefined || cube.angle === null)
    ? '--' : (cube.angle > 0 ? '+' : '') + cube.angle.toFixed(1);
  var reliable = cube.reliable !== false;
  document.getElementById('quality').textContent = !cube.found
    ? '画面里没找到魔方'
    : (reliable
        ? ('综合 ' + cube.score.toFixed(2) + ' · 内线 ' + cube.gap.toFixed(2) + ' · 颜色 ' + cube.good.toFixed(2))
        : '这一层正在转（|角度| > 12°）：角度可信，颜色等摆正再读');
  document.getElementById('sCenter').textContent = cube.found
    ? ('(' + cube.center[0] + ', ' + cube.center[1] + ') / ' + cube.cell_px + ' px') : '-';
  document.getElementById('sGap').textContent = cube.found
    ? (cube.gap.toFixed(2) + ' / ' + cube.good.toFixed(2)) : '-';
  document.getElementById('sScore').textContent = cube.found
    ? (cube.score.toFixed(2) + ' / ' + (cube.source || '-')) : '-';
  document.getElementById('sMs').textContent = cube.found ? (cube.ms + ' ms') : '-';
}

async function poll(){
  try {
    var c = await api('/camera/state');
    var v = await api('/vision/state');
    var running = !!c.running;
    var bCam = document.getElementById('bCam');
    bCam.className = 'badge ' + (running ? 'on' : 'off');
    bCam.textContent = running ? ('直播中 ' + c.width + 'x' + c.height + ' ' + c.fps + 'fps') : '相机未开';
    if (running && !streaming) startView();
    if (!running && streaming) stopView();

    var rec = c.recognizer || {};
    var bRecog = document.getElementById('bRecog');
    bRecog.className = 'badge ' + (rec.enabled ? 'on' : 'off');
    bRecog.textContent = rec.enabled ? ('识别中 ' + (rec.detect_ms || 0) + 'ms') : '识别未开';
    document.getElementById('btnRecog').dataset.on = rec.enabled ? '1' : '0';
    document.getElementById('btnRecog').textContent = rec.enabled ? '停止识别' : '开始识别';

    var pal = (v.palette && v.palette.learned) || [];
    document.getElementById('bPal').textContent = pal.length
      ? ('调色板：已学 ' + pal.length + '/6（' + pal.join('') + '）') : '调色板：标准';
    document.getElementById('bRef').textContent = (v.ref_angle === null || v.ref_angle === undefined)
      ? '基准：未设' : ('基准：' + v.ref_angle.toFixed(1) + '°');

    var cube = (rec.result && rec.result.cube) || null;
    if (!cube && v.last) cube = v.last;
    if (cube) renderReading(Object.assign({cell_px: cube.cell, source: cube.source}, cube));
  } catch(e){
    document.getElementById('bCam').className = 'badge err';
    document.getElementById('bCam').textContent = '服务断开';
  }
}

async function learnFace(){
  var face = document.getElementById('face').value;
  var j = await api('/vision/calibrate', {op:'palette', face:face});
  if (!j.ok) return;
  log('<b>✓ 已学 ' + face + '（' + (NAMES[face] || '') + '）</b> Lab=' + j.lab.join(', '));
  document.getElementById('calHint').textContent = '已学：' + (j.palette.learned || []).join(' ');
}
async function setRef(){
  var j = await api('/vision/calibrate', {op:'reference'});
  if (!j.ok) return;
  log('<b>✓ 基准已设</b> ' + j.ref_angle + '°');
}
async function resetAll(){
  await api('/vision/calibrate', {op:'reset'});
  log('标定已清空');
}

function startPick(mode){
  if (!streaming){ log('<i>先打开相机</i>'); return; }
  if (picking && pickMode === mode){ cancelPick(); return; }
  picking = true;
  pickMode = mode;
  corners = [];
  var btn = document.getElementById(
    mode === 'table' ? 'btnTable' : (mode === 'place' ? 'btnPlace' : 'btnQuad'));
  btn.textContent = mode === 'table'
    ? '依次点桌面 4 个角…（再点取消）'
    : (mode === 'place' ? '点画面里的物体…（再点取消）' : '依次点 4 个角…（再点取消）');
  document.getElementById('view').style.cursor = 'crosshair';
  log(mode === 'table'
    ? '从画面左上角的桌角开始，顺时针点 4 个角（桌面 1.4×0.9 m）'
    : (mode === 'place'
        ? '点物体和桌面接触的位置（尺寸用上面的默认值）'
        : '从左上角开始，顺时针点 4 个角'));
}
function cancelPick(){
  picking = false;
  pickMode = null;
  corners = [];
  document.getElementById('btnQuad').textContent = '手动框四个角';
  document.getElementById('btnTable').textContent = '点桌面四角 → 标定';
  document.getElementById('btnPlace').textContent = '点画面里的物体 → 加进仿真';
  document.getElementById('view').style.cursor = '';
}
function pickQuad(){ startPick('quad'); }
function pickTable(){ startPick('table'); }
function pickPlace(){ startPick('place'); }
async function clearQuad(){
  await api('/vision/quad', {points:null});
  log('已恢复自动检测');
}
document.addEventListener('DOMContentLoaded', function(){
  var img = document.getElementById('view');
  img.addEventListener('click', async function(ev){
    if (!picking) return;
    var r = img.getBoundingClientRect();
    var scale = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
    var offX = r.left + (r.width - img.naturalWidth * scale) / 2;
    var offY = r.top + (r.height - img.naturalHeight * scale) / 2;
    var x = Math.round((ev.clientX - offX) / scale);
    var y = Math.round((ev.clientY - offY) / scale);
    if (pickMode === 'place'){
      var name = document.getElementById('objName').value || '物体';
      var kind = document.getElementById('objKind').value;
      var size = kind === 'cylinder' ? [0.04, 0.10] : [0.08, 0.08, 0.06];
      cancelPick();
      var j = await api('/scene/place', {name:name, u:x, v:y, size:size, kind:kind});
      if (j.ok){
        log('<b>✓ 已加入场景</b> ' + j.object.name + ' @ (' +
            j.object.center[0].toFixed(3) + ', ' + j.object.center[1].toFixed(3) + ') m');
        document.getElementById('sceneHint').textContent =
          '物体已进碰撞模型：打开 /arm3d 看青色物体。';
      }
      return;
    }
    corners.push([x, y]);
    log('角点 ' + corners.length + '：(' + x + ', ' + y + ')');
    if (corners.length === 4){
      var mode = pickMode;
      var pts = corners.slice();
      cancelPick();
      if (mode === 'quad'){
        var j = await api('/vision/quad', {points:pts});
        if (j.ok) log('<b>✓ 已切到手动框</b>');
      } else {
        var rot = parseInt(document.getElementById('rot').value, 10) || 0;
        var mir = document.getElementById('mir').checked;
        var c = await api('/scene/calibrate', {pixels:pts, rotate:rot, mirror:mir});
        if (!c.ok) return;
        log('<b>✓ 桌面标定完成</b> 残差 ' + c.rms_mm + ' mm，' + c.mm_per_px + ' mm/px');
        document.getElementById('sceneHint').textContent =
          '已标定：残差 ' + c.rms_mm + ' mm（' + c.mm_per_px + ' mm/px）。'
          + '打开 /arm3d 看橙色物体和桌面贴图；对位不准就换「方向 / 镜像」再点一次。';
      }
    }
  });
});

buildGrid();
poll();
timer = setInterval(poll, 800);
log('页面已连接。先打开相机，再逐面学颜色、设基准。');
</script>
</body>
</html>
"""
