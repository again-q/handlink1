"""摄像头页面：http://127.0.0.1:8766/camera

左边是实时画面（MJPEG，不依赖任何前端库），右边是设备 / 参数 / 识别。
识别结果走同一套接口返回，识别不开也照常能看画面。
"""

HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>handlink 摄像头 · 实时画面</title>
<style>
  :root{--bg:#0b1020;--panel:#141c33;--line:#24304f;--txt:#e6ecff;--dim:#8d9bbd;
        --ok:#4ade80;--warn:#fbbf24;--err:#f87171;--accent:#60a5fa}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
  header{display:flex;align-items:center;gap:12px;padding:14px 20px;border-bottom:1px solid var(--line);flex-wrap:wrap}
  header h1{font-size:16px;margin:0;font-weight:600}
  .badge{font-size:12px;padding:2px 8px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
  .badge.on{color:var(--ok);border-color:#14532d;background:#052e16}
  .badge.off{color:var(--dim)}
  .badge.err{color:var(--err);border-color:#7f1d1d;background:#2a0f0f}
  .wrap{display:grid;grid-template-columns:minmax(420px,1fr) 380px;gap:18px;padding:18px;max-width:1500px;margin:0 auto}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}
  .card h2{font-size:13px;margin:0 0 10px;color:var(--dim);font-weight:600;letter-spacing:.04em}
  .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
  button{cursor:pointer;border:1px solid var(--line);background:#1b2440;color:var(--txt);
         padding:7px 12px;border-radius:9px;font-size:13px}
  button:hover{border-color:var(--accent);color:#fff}
  button.primary{background:#1d4ed8;border-color:#2563eb}
  button.danger{background:#7f1d1d;border-color:#b91c1c}
  button:disabled{opacity:.45;cursor:not-allowed}
  select,input[type=number]{background:#0e1526;border:1px solid var(--line);color:var(--txt);
         border-radius:8px;padding:6px 8px;font-size:13px}
  #stage{position:relative;background:#05080f;border-radius:12px;overflow:hidden;min-height:320px;
         display:flex;align-items:center;justify-content:center}
  #view{display:block;width:100%;height:auto;max-height:72vh;object-fit:contain}
  #placeholder{position:absolute;inset:0;display:flex;flex-direction:column;gap:8px;
               align-items:center;justify-content:center;color:var(--dim);font-size:13px;text-align:center;padding:20px}
  table{width:100%;border-collapse:collapse;font-size:13px}
  th,td{text-align:left;padding:5px 4px;border-bottom:1px solid #1d2743}
  th{color:var(--dim);font-weight:500;width:38%}
  td{font-variant-numeric:tabular-nums}
  .muted{color:var(--dim);font-size:12px}
  .flex{display:grid;grid-template-columns:auto 1fr auto;gap:8px;align-items:center;font-size:12px}
  .bar{height:6px;background:#1d2743;border-radius:99px;overflow:hidden}
  .bar>i{display:block;height:100%;background:linear-gradient(90deg,#60a5fa,#a78bfa);width:0}
  #log{height:110px;overflow:auto;font:12px/1.5 ui-monospace,Consolas,monospace;color:#9fb0d0;white-space:pre-wrap}
  #log b{color:var(--ok)} #log i{color:var(--err);font-style:normal}
  a{color:var(--accent);text-decoration:none}
</style>
</head>
<body>
<header>
  <h1>handlink 摄像头</h1>
  <span class="badge off" id="live">未连接</span>
  <span class="badge" id="res">-</span>
  <span class="badge" id="fps">-</span>
  <span class="badge" id="recog">识别未开</span>
  <span class="muted" style="margin-left:auto">MJPEG 直出 · 同一网段可直接嵌入 <code>&lt;img src="/camera.mjpg"&gt;</code></span>
</header>

<div class="wrap">
  <div style="display:flex;flex-direction:column;gap:14px">
    <div class="card" style="padding:10px">
      <div id="stage">
        <img id="view" alt="实时画面" hidden>
        <div id="placeholder">摄像头未打开<br><span class="muted">右边选一路设备，点「打开」</span></div>
      </div>
      <div class="row" style="margin:10px 0 0">
        <button class="primary" id="btnOpen" onclick="openCam()">打开</button>
        <button class="danger" id="btnClose" onclick="closeCam()" disabled>关闭</button>
        <button onclick="shot()">截图 PNG</button>
        <button onclick="reconnect()">重连画面</button>
        <span class="muted" id="hint" style="margin-left:auto"></span>
      </div>
    </div>

    <div class="card">
      <h2>设备与取流方式</h2>
      <div class="row">
        <select id="stream" style="flex:1;min-width:240px"></select>
        <button onclick="scan()">扫描</button>
      </div>
      <div class="muted" id="phys">读取物理设备…</div>
    </div>
  </div>

  <div style="display:flex;flex-direction:column;gap:14px">
    <div class="card">
      <h2>参数</h2>
      <div class="row">
        <label class="muted">旋转</label>
        <select id="rotate" onchange="applyCfg()">
          <option value="0">0°</option><option value="90">90°</option>
          <option value="180">180°</option><option value="270">270°</option>
        </select>
        <label class="muted">镜像</label>
        <select id="mirror" onchange="applyCfg()">
          <option value="0">关</option><option value="1">开</option>
        </select>
      </div>
      <div class="row">
        <label class="muted">分辨率</label>
        <select id="size" onchange="applyCfg()">
          <option value="640x480">640×480（最流畅）</option>
          <option value="1280x720">1280×720</option>
          <option value="1280x800">1280×800</option>
          <option value="960x600">960×600</option>
          <option value="640x360">640×360</option>
        </select>
      </div>
      <div class="muted">改完参数会重开一次设备（约 1 秒出画面）。</div>
    </div>

    <div class="card">
      <h2>状态</h2>
      <table>
        <tr><th>后端 / index</th><td id="sBackend">-</td></tr>
        <tr><th>分辨率</th><td id="sRes">-</td></tr>
        <tr><th>帧率</th><td id="sFps">-</td></tr>
        <tr><th>累计帧 / 错误</th><td id="sFrames">-</td></tr>
        <tr><th>最近错误</th><td id="sErr">-</td></tr>
      </table>
    </div>

    <div class="card">
      <h2>视觉识别</h2>
      <div class="row">
        <button id="btnTrack" onclick="pickTrack()">框住机械手（点手掌）</button>
        <button id="btnRecog" onclick="toggleRecog()">开识别（MediaPipe 人手）</button>
      </div>
      <div class="row"><span class="muted" id="recogHint"></span></div>
      <div id="handBox" class="muted">没有识别结果。</div>
      <div class="row" style="margin-top:10px">
        <span class="muted">人手的「每根手指弯曲量」会出现在下面；机械手只给位置和置信度。</span>
      </div>
      <div id="flexBox"></div>
    </div>

    <div class="card">
      <h2>麦克风</h2>
      <div class="row">
        <select id="mic" style="flex:1;min-width:170px"></select>
        <button id="btnMicTest" onclick="micTest()">试录 2 秒</button>
      </div>
      <div class="row">
        <button class="primary" id="btnMic" onclick="micToggle()">开始录音</button>
        <span class="muted" id="micState">未在录制</span>
      </div>
      <div class="flex"><span class="muted">电平</span>
        <div class="bar"><i id="micLevel" style="width:0%"></i></div>
        <span class="muted" id="micDb">-90 dB</span></div>
      <div class="muted" id="micLast" style="margin-top:8px">USB 摄像头的麦一般显示成「麦克风 (USB2.0 MIC)」。</div>
    </div>

    <div class="card">
      <h2>接口日志</h2>
      <div id="log">等待操作…</div>
    </div>
  </div>
</div>

<script>
const FINGER_NAMES = {"102":"拇指1","107":"拇指2","103":"食指","104":"中指","105":"无名指","106":"小指"};
const STORE = "handlink.cam";
let cfg = loadCfg(), streaming = false, timer = null;

function loadCfg(){
  try{ return Object.assign({index:0,backend:"any",width:640,height:480,rotate:0,mirror:false,saved:false},
                            JSON.parse(localStorage.getItem(STORE)||"{}")); }
  catch(e){ return {index:0,backend:"any",width:640,height:480,rotate:0,mirror:false,saved:false}; }
}
function saveCfg(){ try{ localStorage.setItem(STORE, JSON.stringify(cfg)); }catch(e){} }

function log(msg, cls){
  const box = document.getElementById("log");
  const line = document.createElement("div");
  line.innerHTML = cls ? `<${cls}>${msg}</${cls}>` : msg;
  box.prepend(line);
  while (box.children.length > 40) box.lastChild.remove();
}
async function api(path, body){
  const opt = body === undefined ? {} : {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)};
  const r = await fetch(path, opt);
  let j;
  try{ j = await r.json(); }catch(e){ j = {ok:false, error:"HTTP "+r.status}; }
  if (j.ok === false) log(`<i>✗ ${path} ${j.error||("HTTP "+r.status)}</i>`);
  return j;
}

// ---- 设备列表
async function scan(){
  log("扫描摄像头…");
  const j = await api("/camera/devices?force=1");
  fillStreams(j.streams || []);
  showPhysical(j.devices || []);
  log(`扫到 ${(j.streams||[]).length} 路取流方式`);
  if (!j.streams || !j.streams.length) log("<i>没扫到能出图的设备</i>");
}
function fillStreams(list){
  const sel = document.getElementById("stream");
  sel.innerHTML = "";
  for (const s of list){
    const o = document.createElement("option");
    o.value = `${s.backend}:${s.index}:${s.width}x${s.height}`;
    const fps = s.measured_fps ? ` · ${s.measured_fps} fps` : "";
    o.textContent = `${s.backend_label} #${s.index} · ${s.width}×${s.height}${fps}${s.frame?"":"（无画面）"}`;
    if (s.backend === cfg.backend && s.index === cfg.index) o.selected = true;
    sel.appendChild(o);
  }
  if (!list.length) sel.innerHTML = '<option value="">没有可用设备</option>';
}
function showPhysical(list){
  const box = document.getElementById("phys");
  if (!list.length){ box.textContent = "注册表里没找到 UVC 摄像头。"; return; }
  box.innerHTML = list.map(d =>
    `${d.present ? "●" : "○"} ${d.name}<span class="muted"> · ${d.vidpid}</span>`).join("<br>");
}
function readSelect(){
  const v = document.getElementById("stream").value;
  const m = /^(\w+):(\d+):(\d+)x(\d+)$/.exec(v||"");
  if (m){ cfg.backend = m[1]; cfg.index = +m[2]; }
  const s = /^(\d+)x(\d+)$/.exec(document.getElementById("size").value||"");
  if (s){ cfg.width = +s[1]; cfg.height = +s[2]; }
  cfg.rotate = +document.getElementById("rotate").value || 0;
  cfg.mirror = document.getElementById("mirror").value === "1";
  saveCfg();
}

// ---- 开关
async function openCam(){
  readSelect();
  cfg.saved = true; saveCfg();
  const j = await api("/camera/open", cfg);
  if (!j.ok){ log(`<i>✗ 打不开：${j.error}</i>`); return; }
  if (j.fallback) log(`<b>↪ 自动切到</b> ${j.used}（原选的那路没画面）`);
  if (j.native_size) log(`<b>↪ 改用设备原生分辨率</b> ${j.width}×${j.height}（页面存的那个它不支持）`);
  syncControls(j);
  startView();
  log(`<b>✓ 已打开</b> ${cfg.backend} #${cfg.index} ${j.width}×${j.height}`);
}
async function closeCam(){
  await api("/camera/close", {});
  stopView();
  log("已关闭摄像头");
}
async function applyCfg(){
  readSelect();
  if (streaming) await openCam();
}
function startView(){
  streaming = true;
  const img = document.getElementById("view");
  img.hidden = false;
  document.getElementById("placeholder").hidden = true;
  img.src = "/camera.mjpg?t=" + Date.now();
  document.getElementById("btnOpen").disabled = true;
  document.getElementById("btnClose").disabled = false;
  img.onerror = () => { log("<i>画面断了</i>"); };
}
function stopView(){
  streaming = false;
  const img = document.getElementById("view");
  img.hidden = true;
  img.removeAttribute("src");
  document.getElementById("placeholder").hidden = false;
  document.getElementById("btnOpen").disabled = false;
  document.getElementById("btnClose").disabled = true;
  document.getElementById("live").className = "badge off";
  document.getElementById("live").textContent = "未连接";
}
async function reconnect(){ if (streaming) startView(); else openCam(); }
function shot(){
  if (!streaming) return log("<i>先打开摄像头</i>");
  const a = document.createElement("a");
  a.href = "/camera/snapshot.png?t=" + Date.now();
  a.download = "handlink-" + Date.now() + ".png";
  a.click();
}

// ---- 识别
let picking = false;
async function pickTrack(){
  if (!streaming){ log("<i>先打开摄像头</i>"); return; }
  picking = !picking;
  const btn = document.getElementById("btnTrack");
  btn.textContent = picking ? "点画面里的手掌…（再点此取消）" : "框住机械手（点手掌）";
  document.getElementById("view").style.cursor = picking ? "crosshair" : "";
}
document.addEventListener("DOMContentLoaded", () => {
  const img = document.getElementById("view");
  img.addEventListener("click", async (ev) => {
    if (!picking) return;
    picking = false;
    document.getElementById("btnTrack").textContent = "框住机械手（点手掌）";
    img.style.cursor = "";
    // 画面是 object-fit:contain，先换算回原始像素坐标
    const r = img.getBoundingClientRect();
    const scale = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
    const offX = r.left + (r.width - img.naturalWidth * scale) / 2;
    const offY = r.top + (r.height - img.naturalHeight * scale) / 2;
    const x = (ev.clientX - offX) / scale;
    const y = (ev.clientY - offY) / scale;
    // 手掌模板：140×110，按当前分辨率缩放
    const s = (img.naturalWidth || 640) / 640;
    const box = {x: Math.round(x - 70 * s), y: Math.round(y - 55 * s),
                 w: Math.round(140 * s), h: Math.round(110 * s)};
    const j = await api("/camera/track", box);
    if (!j.ok){ log(`<i>✗ 标定失败：${j.error}</i>`); return; }
    const k = await api("/camera/detect", {enable: true, mode: "robot", every: 3});
    if (!k.ok){ log(`<i>✗ 开启跟踪失败：${k.error}</i>`); return; }
    log(`<b>✓ 已框住机械手</b> 模板 ${j.rect[2]}×${j.rect[3]} @ (${j.rect[0]}, ${j.rect[1]})`);
    document.getElementById("recogHint").textContent = "模板跟踪中";
  });
});

async function toggleRecog(){
  const on = document.getElementById("btnRecog").dataset.on === "1";
  const j = await api("/camera/detect", {enable: !on, mode: "hand"});
  if (!j.ok){ document.getElementById("recogHint").textContent = j.error || "识别不可用"; return; }
  const on2 = !on;
  const btn = document.getElementById("btnRecog");
  btn.dataset.on = on2 ? "1" : "0";
  btn.textContent = on2 ? "关识别" : "开识别（MediaPipe 人手）";
  document.getElementById("recogHint").textContent = on2 ? (j.recognizer && j.recognizer.name || "") : "";
  if (!on2) document.getElementById("flexBox").innerHTML = "";
}

// ---- 轮询
async function poll(){
  try{
    const j = await api("/camera/state");
    const running = !!j.running;
    document.getElementById("live").className = "badge " + (running ? "on" : "off");
    document.getElementById("live").textContent = running ? "直播中" : "未连接";
    document.getElementById("res").textContent = running ? `${j.width}×${j.height}` : "-";
    document.getElementById("fps").textContent = running ? `${j.fps} fps` : "-";
    document.getElementById("sBackend").textContent = `${j.config.backend} #${j.config.index}`;
    document.getElementById("sRes").textContent = running ? `${j.width}×${j.height}` : "-";
    document.getElementById("sFps").textContent = running ? `${j.fps} fps` : "-";
    document.getElementById("sFrames").textContent = `${j.frames} / ${j.errors}`;
    document.getElementById("sErr").textContent = j.last_error || "-";
    if (running && !streaming) startView();
    if (!running && streaming) stopView();

    const rec = j.recognizer || {};
    const enabled = !!rec.enabled;
    const badge = document.getElementById("recog");
    badge.className = "badge " + (enabled ? "on" : "off");
    badge.textContent = enabled ? `识别中 · ${rec.detect_ms||0} ms` : "识别未开";
    const btn = document.getElementById("btnRecog");
    btn.dataset.on = enabled ? "1" : "0";
    btn.textContent = enabled ? "关识别" : "开识别（MediaPipe 手部）";
    if (enabled && rec.result) renderResult(rec.result);
  }catch(e){
    document.getElementById("live").className = "badge err";
    document.getElementById("live").textContent = "服务断开";
  }
}
function renderResult(res){
  const box = document.getElementById("handBox");
  const flex = document.getElementById("flexBox");
  if (!res.hands){
    const score = (res.score === null || res.score === undefined) ? "" : ` · 最近得分 ${res.score}`;
    box.innerHTML = `<span class="muted">没找到${score}</span>`;
    flex.innerHTML = "";
    return;
  }
  const b = res.bbox || [0,0,0,0];
  if (res.center){
    box.innerHTML = `掌心 <b>(${res.center[0]}, ${res.center[1]})</b> · 置信度 <b>${res.score}</b>` +
                    `<div class="muted">框 [${b.join(", ")}]</div>`;
  } else {
    box.innerHTML = `手 × ${res.hands} · 置信度 ${res.score} · 框 [${b.join(", ")}]`;
  }
  const f = res.flex || {};
  flex.innerHTML = Object.keys(f).sort().map(k =>
    `<div class="flex"><span>${FINGER_NAMES[k]||k}</span>
     <div class="bar"><i style="width:${Math.min(100, f[k]*100)}%"></i></div>
     <span class="muted">${f[k].toFixed(2)}</span></div>`).join("");
}

// ---- 麦克风
let micRunning = false, micDevices = [], micTesting = false;

async function loadMics(){
  const j = await api("/audio/devices");
  const sel = document.getElementById("mic");
  micDevices = j.devices || [];
  sel.innerHTML = "";
  if (!micDevices.length){
    sel.innerHTML = '<option value="">没有找到麦克风</option>';
    if (j.windows === false) document.getElementById("micLast").textContent = "这个系统上录音还没做（目前只支持 Windows）。";
    else document.getElementById("micLast").textContent = "系统里没有录音设备：插一个带麦的 USB 摄像头，或检查系统麦克风权限。";
    return;
  }
  for (const d of micDevices){
    const o = document.createElement("option");
    o.value = String(d.index);
    o.textContent = `#${d.index} ${d.name}`;
    sel.appendChild(o);
  }
  const usb = micDevices.find(d => /usb/i.test(d.name));
  if (usb) sel.value = String(usb.index);
  document.getElementById("micLast").textContent =
    `共 ${micDevices.length} 个输入：${micDevices.map(d => `#${d.index} ${d.name}`).join(" · ")}`;
}
async function micToggle(){
  if (micRunning) return micStop();
  const idx = +document.getElementById("mic").value || 0;
  const j = await api("/audio/start", {index: idx});
  if (!j.ok){ document.getElementById("micState").textContent = j.error || "开不了麦"; return; }
  log(`<b>● 开始录音</b> #${idx} ${j.rate} Hz · 最长 ${j.max_seconds} 秒`);
  micPoll();
}
async function micStop(){
  const j = await api("/audio/stop", {});
  if (!j.ok){ document.getElementById("micState").textContent = j.error || "没在录"; return; }
  showClip(j);
  log(`<b>■ 录音结束</b> ${j.seconds} 秒 · 峰值 ${j.peak_db} dB`);
}
function showClip(j){
  const name = (j.path || "").replace(/^.*[\\/]/, "");
  if (!name) return;
  const size = j.seconds ? `${j.seconds} 秒` : "";
  document.getElementById("micLast").innerHTML =
    `上一段：<a href="/audio/file/${encodeURIComponent(name)}" download>${name}</a>` +
    `<span class="muted"> · ${size}${j.peak_db !== undefined ? ` · 峰值 ${j.peak_db} dB` : ""}</span>`;
}
async function micTest(){
  if (micTesting || micRunning) return;
  const btn = document.getElementById("btnMicTest");
  const idx = +document.getElementById("mic").value || 0;
  micTesting = true;
  btn.disabled = true;
  document.getElementById("micState").textContent = "试录中…（说句话）";
  const j = await api("/audio/test", {index: idx, seconds: 2});
  micTesting = false;
  btn.disabled = false;
  if (!j.ok){ document.getElementById("micState").textContent = j.error || "试录失败"; return; }
  document.getElementById("micState").textContent = `${j.signal} · 峰值 ${j.peak_db} dB`;
  showClip(j);
  log(`<b>♪ 试录</b> ${j.seconds} 秒 · ${j.signal} · 峰值 ${j.peak_db} dB`);
}
async function micPoll(){
  try{
    const j = await api("/audio/state");
    const lv = Math.round((j.level || 0) * 100);
    document.getElementById("micLevel").style.width = Math.min(100, lv) + "%";
    document.getElementById("micDb").textContent = `${(j.level_db ?? -90).toFixed(1)} dB`;
    const on = !!j.running;
    const btn = document.getElementById("btnMic");
    btn.textContent = on ? "停止录音" : "开始录音";
    btn.className = on ? "danger" : "primary";
    document.getElementById("btnMicTest").disabled = on || micTesting;
    if (on){
      document.getElementById("micState").textContent = `录音中 ${j.seconds.toFixed(1)} 秒`;
    } else if (micRunning){
      document.getElementById("micState").textContent = "未在录制";
      if (j.last && j.last.path) showClip(j.last);
    }
    micRunning = on;
  }catch(e){ /* 服务没起来时静默 */ }
}

// ---- 启动
function syncControls(j){
  // 把服务端实际在跑的那一路同步到控件上：不然页面还拿着旧 index/分辨率，
  // 点一次「打开」就把你切到别的摄像头去了。
  cfg.index = j.config.index; cfg.backend = j.config.backend;
  cfg.width = j.width; cfg.height = j.height;
  cfg.rotate = j.config.rotate; cfg.mirror = !!j.config.mirror;
  saveCfg();
  const size = document.getElementById("size");
  const opt = `${j.width}x${j.height}`;
  if (![...size.options].some(o => o.value === opt)) size.appendChild(new Option(`${j.width}×${j.height}（设备实际）`, opt));
  size.value = opt;
  document.getElementById("rotate").value = String(cfg.rotate);
  document.getElementById("mirror").value = cfg.mirror ? "1" : "0";
  const sel = document.getElementById("stream");
  const want = `${cfg.backend}:${cfg.index}:`;
  for (const o of sel.options) if (o.value.startsWith(want)) { sel.value = o.value; break; }
}

async function boot(){
  document.getElementById("rotate").value = String(cfg.rotate||0);
  document.getElementById("mirror").value = cfg.mirror ? "1" : "0";
  document.getElementById("size").value = `${cfg.width}x${cfg.height}`;
  const j = await api("/camera/state");
  if (!j.cv2){
    document.getElementById("hint").textContent = "没装 opencv，只能列设备";
  }
  await scan();
  await loadMics();
  setInterval(micPoll, 300);
  if (j.running){ syncControls(j); startView(); log(`摄像头已在运行，接管画面（${j.config.backend} #${j.config.index} ${j.width}×${j.height}）`); }
  else if (cfg.saved){ openCam(); }
  else { document.getElementById("hint").textContent = "选好设备后点「打开」"; }
  poll(); timer = setInterval(poll, 900);
}
boot();
</script>
</body>
</html>
"""
