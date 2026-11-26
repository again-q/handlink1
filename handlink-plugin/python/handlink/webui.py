"""内置网页演示：http://127.0.0.1:8766/demo

一个页面把 handlink 的接口全用上：拉 /state、点按钮发 /pose、套厂家点位、
播放动作、录制并出分析。它直接反映总线的真实状态——只是当前总线是仿真手。
"""

HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>handlink 演示 · 灵巧手</title>
<style>
  :root{--bg:#0b1020;--panel:#141c33;--line:#24304f;--txt:#e6ecff;--dim:#8d9bbd;
        --ok:#4ade80;--warn:#fbbf24;--err:#f87171;--accent:#60a5fa}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
  header{display:flex;align-items:center;gap:12px;padding:14px 20px;border-bottom:1px solid var(--line)}
  header h1{font-size:16px;margin:0;font-weight:600}
  .badge{font-size:12px;padding:2px 8px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
  .badge.sim{border-color:#7c5cff33;color:#c4b5fd;background:#7c5cff1a}
  .badge.ok{border-color:#4ade8040;color:#86efac;background:#4ade801a}
  .badge.bad{border-color:#f8717140;color:#fca5a5;background:#f871711a}
  .wrap{display:grid;grid-template-columns:minmax(320px,1fr) 460px;gap:18px;padding:18px;max-width:1100px;margin:0 auto}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}
  .card h2{font-size:13px;margin:0 0 10px;color:var(--dim);font-weight:600;letter-spacing:.04em}
  .row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
  button{cursor:pointer;border:1px solid var(--line);background:#1b2440;color:var(--txt);
         padding:7px 12px;border-radius:9px;font-size:13px}
  button:hover{border-color:var(--accent);color:#fff}
  button.primary{background:#1d4ed8;border-color:#2563eb}
  button.danger{background:#7f1d1d;border-color:#b91c1c}
  button:disabled{opacity:.45;cursor:not-allowed}
  table{width:100%;border-collapse:collapse;font-size:13px}
  th,td{text-align:right;padding:5px 6px;border-bottom:1px solid #1d2743}
  th:first-child,td:first-child{text-align:left}
  th{color:var(--dim);font-weight:500}
  .bar{height:6px;background:#1d2743;border-radius:99px;overflow:hidden;min-width:56px}
  .bar>i{display:block;height:100%;background:linear-gradient(90deg,#60a5fa,#a78bfa)}
  input[type=range]{width:110px;accent-color:#60a5fa;vertical-align:middle}
  #log{height:132px;overflow:auto;font:12px/1.5 ui-monospace,Consolas,monospace;color:#9fb0d0;white-space:pre-wrap}
  #log b{color:var(--ok)} #log i{color:var(--err);font-style:normal}
  svg{display:block;margin:0 auto}
  .muted{color:var(--dim);font-size:12px}
  .stat{display:flex;gap:18px;font-size:12px;color:var(--dim);margin-top:8px}
  .stat b{color:var(--txt);font-weight:600}
</style>
</head>
<body>
<header>
  <h1>handlink 演示</h1>
  <span class="badge sim" id="bus">连接中…</span>
  <button id="btnLink" title="打开 CAN 适配器连上真手；掉线后再点一次重连">连接机械手</button>
  <span class="badge" id="rec">未录制</span>
  <span class="badge" id="playing" hidden>动作播放中</span>
  <a class="badge" href="/arm3d" style="text-decoration:none">三维机械臂 →</a>
  <a class="badge" href="/camera" style="text-decoration:none">摄像头画面 →</a>
  <span class="muted" style="margin-left:auto">中科灵犀 LG-603 · 六指 6 主动自由度</span>
</header>

<div class="wrap">
  <div class="card">
    <h2>手部状态（/state 实时）</h2>
    <svg id="hand" viewBox="0 0 260 320" width="300" height="360"></svg>
    <div class="row" style="justify-content:center;margin-top:6px" id="actions">
      <button class="danger" onclick="stop()">停止</button>
    </div>
    <div class="stat"><span>指令：<b id="cmds">0</b></span><span>最近：<b id="reason">-</b></span></div>
    <div class="muted" id="linkMsg" style="margin-top:8px">还没确认连接——点右上角「连接机械手」。</div>
  </div>

  <div style="display:flex;flex-direction:column;gap:18px">
    <div class="card">
      <h2>逐指控制（拖动即下发 /pose）</h2>
      <table><thead><tr><th>手指</th><th>位置</th><th style="width:130px">设定</th><th>电流</th></tr></thead>
      <tbody id="rows"></tbody></table>
      <div class="muted" style="margin-top:8px">只改一根手指时，其余电机保持原位（响应里的 <code>held</code>）。</div>
    </div>

    <div class="card">
      <h2>AI 控制（自然语言 → 技能）</h2>
      <div class="row">
        <input id="aiText" placeholder="先张开，再握拳，然后食指指一下"
               style="flex:1;min-width:200px;background:#0e1526;border:1px solid var(--line);
                      color:var(--txt);border-radius:8px;padding:7px 10px;font-size:13px">
        <button class="primary" onclick="aiSend(false)">执行</button>
        <button onclick="aiSend(true)">只看计划</button>
      </div>
      <div id="aiPlan" class="muted" style="margin-top:8px">
        技能白名单：张开 / 握拳 / 捏 / 剪刀 / 食指指 / 指定角度 / 保持
      </div>
    </div>

    <div class="card">
      <h2>动作 / 录制</h2>
      <div class="row">
        <button onclick="track('动作一')">播放 动作一</button>
        <button onclick="track('动作一', true)">预览时间线</button>
        <button onclick="recStart()">开始录制</button>
        <button onclick="recStop()">停止并分析</button>
      </div>
      <div id="stats" class="muted">录制结束后这里显示行程、峰值电流、电流占用率。</div>
    </div>

    <div class="card">
      <h2>接口日志</h2>
      <div id="log">等待操作…</div>
    </div>
  </div>
</div>

<script>
const MOTORS = [
  {id:102, name:"拇指1", max:100, x:64,  y:214, len1:34, len2:22, sign:1,  base:-32},
  {id:107, name:"拇指2", max:45,  x:64,  y:214, len1:20, len2:14, sign:1,  base:-32},
  {id:103, name:"食指",  max:180, x:88,  y:172, len1:46, len2:30, sign:1},
  {id:104, name:"中指",  max:180, x:108, y:168, len1:52, len2:34, sign:1},
  {id:105, name:"无名指",max:180, x:128, y:172, len1:46, len2:30, sign:-1},
  {id:106, name:"小指",  max:180, x:148, y:178, len1:38, len2:24, sign:-1}
];
const state = {};
let cmds = 0;

// ---- 画手：每根手指 = 近节 + 远节，curl 越大弯得越多
const NS = "http://www.w3.org/2000/svg";
function el(tag, attrs, parent){
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function buildHand(){
  const svg = document.getElementById("hand");
  el("rect", {x:62, y:168, width:118, height:98, rx:26, fill:"#1b2440", stroke:"#2c3a5e"}, svg);
  el("rect", {x:72, y:262, width:98, height:16, rx:8, fill:"#161f38", stroke:"#2c3a5e"}, svg);
  for (const m of MOTORS){
    if (m.id === 107) continue;              // 拇指2 与拇指1 共用一条链
    const g = el("g", {id:"f"+m.id}, svg);
    el("rect", {x:-9, y:-m.len1, width:18, height:m.len1, rx:9, fill:"#2b3a63", stroke:"#3d4f7d"}, g);
    const g2 = el("g", {id:"g"+m.id}, g);
    el("rect", {x:-8, y:-m.len2, width:16, height:m.len2, rx:8, fill:"#334575", stroke:"#41568a"}, g2);
  }
}
function setCurl(){
  for (const m of MOTORS){
    if (m.id === 107) continue;
    const g = document.getElementById("f"+m.id);
    if (!g) continue;
    const st = state[m.id] || {angle:0};
    const flex = Math.min(1, Math.max(0, st.angle / m.max));
    const a = (m.base || 0) + m.sign * flex * 150;
    g.setAttribute("transform", `translate(${m.x},${m.y}) rotate(${a})`);
    document.getElementById("g"+m.id)
      .setAttribute("transform", `translate(0,${-m.len1}) rotate(${m.sign * flex * 70})`);
  }
}

// ---- 表格
function buildRows(){
  const tb = document.getElementById("rows");
  tb.innerHTML = "";
  for (const m of MOTORS){
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${m.name}<div class="muted">${m.id}</div></td>
      <td id="v${m.id}">-</td>
      <td><input type="range" id="s${m.id}" min="0" max="${m.max}" step="1" value="0"></td>
      <td><div class="bar"><i id="c${m.id}" style="width:0%"></i></div><span class="muted" id="a${m.id}">-</span></td>`;
    tb.appendChild(tr);
  }
  for (const m of MOTORS){
    const s = document.getElementById("s"+m.id);
    s.addEventListener("change", () => pose({[m.id]: +s.value}));
  }
}
function render(){
  setCurl();
  for (const m of MOTORS){
    const st = state[m.id]; if (!st) continue;
    document.getElementById("v"+m.id).textContent = st.angle.toFixed(1) + "°";
    document.getElementById("a"+m.id).textContent = st.current.toFixed(2) + "A";
    document.getElementById("c"+m.id).style.width = Math.min(100, 100*st.current/st.current_limit) + "%";
    const s = document.getElementById("s"+m.id);
    if (document.activeElement !== s) s.value = Math.round(st.angle);
  }
}

// ---- 接口
function log(msg, cls){
  const box = document.getElementById("log");
  const line = document.createElement("div");
  line.innerHTML = cls ? `<${cls}>${msg}</${cls}>` : msg;
  box.prepend(line);
  while (box.children.length > 60) box.lastChild.remove();
}
async function post(path, body){
  const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body||{})});
  const j = await r.json();
  cmds++; document.getElementById("cmds").textContent = cmds;
  if (j.ok === false || r.status >= 400){
    const msg = (j.issues && j.issues[0] && j.issues[0].message) || j.error || ("HTTP " + r.status);
    log(`<i>✗ ${path} ${msg}</i>`);
  } else {
    log(`<b>✓ ${path}</b> ${JSON.stringify(j).slice(0,120)}`);
  }
  if (j.reason) document.getElementById("reason").textContent = j.reason;
  return j;
}
async function pose(partial){
  return post("/pose", {motors: partial, reason:"webui"});
}
async function point(name){
  const r = await fetch("/points/"+encodeURIComponent(name));
  const j = await r.json();
  if (!j.ok) return log(`<i>✗ 没有点位 ${name}</i>`);
  const motors = {};
  for (const m of j.motors) motors[m.motorId] = {angle:m.angle, speed:m.speed, torque:m.torque};
  return post("/pose", {motors, reason:"point:"+name});
}
async function randomPose(){
  const motors = {};
  for (const m of MOTORS) motors[m.id] = {angle: Math.round(Math.random()*m.max*0.85), speed:60, torque:0.3};
  return post("/pose", {motors, reason:"random"});
}
async function track(name, dry){
  const j = await post("/track", {name, dry_run:!!dry});
  if (dry && j.timeline) log(`时间线 ${j.timeline.length} 步，共 ${j.duration_ms} ms`);
}
async function runAction(name){ return post("/action", {name}); }
async function loadActions(){
  try{
    const r = await fetch("/actions");
    const j = await r.json();
    if (!j.ok) return;
    const box = document.getElementById("actions");
    const stopBtn = box.lastElementChild;
    for (const a of j.actions){
      const b = document.createElement("button");
      b.textContent = a.label;
      b.title = a.poses.join(" → ");
      b.onclick = () => runAction(a.name);
      box.insertBefore(b, stopBtn);
    }
    log(`已加载 ${j.actions.length} 组预设动作`);
  }catch(e){ log("<i>动作组加载失败</i>"); }
}
async function stop(){ return post("/stop", {}); }
async function aiSend(dry){
  const text = document.getElementById("aiText").value.trim();
  const box = document.getElementById("aiPlan");
  if (!text) return;
  box.textContent = "解析中…";
  const j = await post("/ai", {text, dry_run: !!dry});
  if (!j.ok && !j.plan){ box.innerHTML = `<i>${j.error || "失败"}</i>`; return; }
  const src = j.plan ? j.plan.source : "?";
  box.innerHTML = `<b style="color:var(--accent)">${src}</b> · ${j.explain || ""}` +
    (j.ok ? "" : ` <i>（${j.error || "未执行"}）</i>`);
}
async function recStart(){
  const j = await post("/record/start", {path:"demo/live-"+Date.now()+".jsonl", hz:20});
  document.getElementById("rec").textContent = j.ok ? "录制中" : "录制失败";
  document.getElementById("rec").style.color = j.ok ? "var(--ok)" : "var(--err)";
}
async function recStop(){
  const j = await post("/record/stop", {});
  document.getElementById("rec").textContent = "未录制";
  document.getElementById("rec").style.color = "";
  if (!j.ok) return;
  const s = j.stats, rows = Object.entries(s.motors).map(([id,m]) =>
    `<tr><td>${m.name}</td><td>${m.angle_min}°</td><td>${m.angle_max}°</td><td>${m.rom}°</td>
     <td>${m.speed_max}</td><td>${m.current_max}A</td><td>${(m.current_peak_ratio*100).toFixed(0)}%</td></tr>`).join("");
  document.getElementById("stats").innerHTML =
    `<div class="muted">${s.samples} 帧 / ${s.duration_s}s</div>
     <table><thead><tr><th>手指</th><th>最小</th><th>最大</th><th>行程</th><th>峰速</th><th>峰值电流</th><th>占用</th></tr></thead><tbody>${rows}</tbody></table>`;
}

// ---- 连接机械手：真的去开 CAN，看到心跳才算连上
let linked = false;
function setLink(j){
  const badge = document.getElementById("bus");
  const msg = document.getElementById("linkMsg");
  const btn = document.getElementById("btnLink");
  const ok = !!(j && j.ok && j.connected);
  linked = ok;
  const kind = (j && j.bus) || "?";
  badge.className = "badge " + (ok ? "ok" : "bad");
  badge.textContent = ok
    ? ("已连接 · " + kind + (j.hand_id ? " 0x" + j.hand_id.toString(16).toUpperCase() : ""))
    : ("未连接 · " + (kind === "sim" ? "仿真" : kind));
  const parts = [];
  if (j && j.detail) parts.push(j.detail);
  if (j && j.bus === "can" && j.motors) parts.push("六指 " + j.motors + " 路有回传");
  if (msg && parts.length) msg.textContent = (ok ? "✓ " : "· ") + parts.join(" · ");
  btn.textContent = ok ? "重连机械手" : "连接机械手";
}
async function handStatus(){
  try{
    const r = await fetch("/hand/status");
    setLink(await r.json());
  }catch(e){
    const badge = document.getElementById("bus");
    badge.className = "badge bad"; badge.textContent = "服务断开";
  }
}
async function linkHand(){
  const btn = document.getElementById("btnLink");
  const msg = document.getElementById("linkMsg");
  btn.disabled = true; btn.textContent = "连接中…";
  if (msg) msg.textContent = "正在打开 CAN 适配器，等第一帧心跳（约 1 秒）…";
  try{
    const j = await post("/hand/attach", {bus:"can"});
    setLink(j);
    if (!j.ok && msg) msg.innerHTML = `<i>连不上：${j.error || "未知原因"}</i>` +
      (j.detail ? ` <span class="muted">${j.detail}</span>` : "");
  }catch(e){
    if (msg) msg.innerHTML = `<i>请求失败：${e.message}</i>`;
  }finally{
    btn.disabled = false;
    if (!linked) btn.textContent = "连接机械手";
  }
}

// ---- 轮询
async function poll(){
  try{
    const r = await fetch("/state");
    const j = await r.json();
    Object.assign(state, j.motors);
    render();
  }catch(e){
    const badge = document.getElementById("bus");
    badge.className = "badge bad"; badge.textContent = "服务断开";
  }
}
document.getElementById("btnLink").onclick = linkHand;
buildHand(); buildRows(); loadActions();
poll(); setInterval(poll, 120);
handStatus(); setInterval(handStatus, 1000);
log("页面已连上服务。点右上「连接机械手」确认手真的在线，再拖滑块下发。");
</script>
</body>
</html>
"""
