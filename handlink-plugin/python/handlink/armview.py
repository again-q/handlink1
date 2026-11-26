"""三维机械臂视图：http://127.0.0.1:8766/arm3d

模型不是拼出来的胶囊了：``handlink/static/nova/`` 下是越疆 Nova 5 的**原厂 STL**
（base_link + Link1~Link6，来自 nova5_2f85_description，和 ``kin.py`` 的关节原点同源），
每段连杆按 URDF 坐标系直接挂到正解出来的位姿上，螺钉、线槽、关节罩都在。

场景是**安全环境**，不是展示台：

* 工位：桌面、料盒、相机立柱、防护栏、站在桌前的操作员、禁入柱（``handlink/cell.py``）；
* 机械臂的碰撞代理：从 STL 生成的胶囊链（``static/nova/proxies.json``），
  实时算到工位每样东西的最小间距，最近的一对用红点标出来；
* SSM：按 ISO/TS 15066 的简化式算"当前速度需要多大间距"，和实测间距并排显示；
* 下发闸门：``POST /arm/check`` 先把 MovJ 的整条关节插值路径扫一遍，
  服务端的 ``arm_move`` 也会再拦一次——不是只做个动画。

数据源和以前一样：在线就每 100 ms 拉 30004 的关节角，不在线就拖滑块。
"""
from __future__ import annotations

HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>handlink · 三维机械臂 · 安全环境</title>
<style>
  :root{--bg:#0a0f1c;--panel:#121a2d;--line:#233150;--txt:#e8eeff;--dim:#8fa0c4;
        --ok:#4ade80;--warn:#fbbf24;--err:#f87171;--accent:#5b9bff;--blue:#3d72c4}
  *{box-sizing:border-box}
  html,body{height:100%}
  body{margin:0;background:var(--bg);color:var(--txt);
       font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif;overflow:hidden}
  header{display:flex;align-items:center;gap:10px;padding:10px 18px;
         border-bottom:1px solid var(--line);height:52px;background:#0c1322}
  header h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.02em}
  .badge{font-size:12px;padding:2px 9px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
  .badge.sim{border-color:#7c5cff55;color:#c4b5fd;background:#7c5cff1a}
  .badge.live{border-color:#4ade8055;color:#86efac;background:#4ade801a}
  a.badge{text-decoration:none}
  .spacer{margin-left:auto}
  main{display:grid;grid-template-columns:1fr 408px;height:calc(100% - 52px)}
  #stage{position:relative;min-width:0;overflow:hidden;background:#080d18}
  canvas{display:block}
  #readout{position:absolute;left:14px;top:12px;font:12px/1.7 ui-monospace,Consolas,monospace;
           color:#9fb0d0;white-space:pre;pointer-events:none;text-shadow:0 1px 2px #000}
  #hint{position:absolute;left:14px;bottom:12px;color:#5f6d8f;font-size:12px;pointer-events:none}
  #hud{position:absolute;right:16px;top:14px;min-width:210px;padding:12px 14px;border-radius:14px;
       background:#0b1220dd;border:1px solid var(--line);backdrop-filter:blur(6px);pointer-events:none}
  #hud .state{font-size:12px;letter-spacing:.14em;color:var(--dim)}
  #hud .num{font:600 34px/1.1 ui-monospace,Consolas,monospace;margin:2px 0 1px}
  #hud .pair{font-size:12px;color:var(--dim)}
  #hud .bar{height:5px;border-radius:99px;background:#1b2438;margin-top:9px;overflow:hidden}
  #hud .bar i{display:block;height:100%;width:0;background:var(--ok);transition:width .18s}
  .panel{border-left:1px solid var(--line);background:var(--panel);overflow:auto;padding:14px}
  .card{border:1px solid var(--line);border-radius:12px;padding:11px 12px;margin-bottom:12px;background:#0f1728}
  .card h2{font-size:12px;margin:0 0 9px;color:var(--dim);font-weight:600;letter-spacing:.06em}
  .row{display:flex;gap:6px;flex-wrap:wrap}
  button{cursor:pointer;border:1px solid var(--line);background:#1a2540;color:var(--txt);
         padding:6px 10px;border-radius:8px;font-size:12.5px}
  button:hover{border-color:var(--accent);color:#fff}
  button.on{background:#1d4ed8;border-color:#2563eb}
  button:disabled{opacity:.4;cursor:not-allowed}
  button.danger{background:#3a1620;border-color:#7f1d1d;color:#fecaca}
  .joint{display:grid;grid-template-columns:26px 1fr 62px;gap:8px;align-items:center;
         margin:5px 0;font-size:12.5px}
  .joint b{color:var(--dim);font-weight:600}
  .joint output{text-align:right;font:12px ui-monospace,Consolas,monospace;color:#cbd6f0}
  input[type=range]{width:100%;accent-color:#5b9bff}
  table{width:100%;border-collapse:collapse;font:12px ui-monospace,Consolas,monospace}
  td{padding:2px 0;color:#cbd6f0}
  td:first-child{color:var(--dim);width:44px}
  .muted{color:var(--dim);font-size:12px}
  .kv{display:flex;justify-content:space-between;font-size:12.5px;padding:2px 0;gap:8px}
  .kv span:first-child{color:var(--dim)}
  .kv b{font-weight:600;font-family:ui-monospace,Consolas,monospace}
  .pill{display:inline-flex;align-items:center;gap:6px;font-size:12px}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--dim);box-shadow:0 0 8px currentColor}
  .dot.ok{background:var(--ok);color:var(--ok)} .dot.warn{background:var(--warn);color:var(--warn)}
  .dot.err{background:var(--err);color:var(--err)}
  .check{display:flex;align-items:center;gap:7px;font-size:12.5px;color:#cbd6f0;margin:4px 0}
  .check input{accent-color:#5b9bff}
  .note{font-size:11.5px;color:#7587aa;line-height:1.55}
  .mono{font-family:ui-monospace,Consolas,monospace}
  .saved{display:flex;flex-wrap:wrap;gap:6px;margin-top:9px}
  .saved .empty{font-size:12px;color:var(--dim);line-height:1.5}
  .chip{display:inline-flex;border:1px solid var(--line);border-radius:8px;overflow:hidden}
  .chip button{border:0;border-radius:0;padding:5px 9px;font-size:12px}
  .chip button.del{padding:5px 7px;color:#fca5a5;background:#2a1522}
  .chip button.del:hover{background:#3a1620;color:#fff}
  input.txt{flex:1;min-width:0;background:#0e1526;border:1px solid var(--line);color:var(--txt);
            border-radius:7px;padding:5px 8px;font:12px "Segoe UI","Microsoft YaHei",sans-serif}
  input.txt:focus{outline:none;border-color:var(--accent)}
</style>
<script type="importmap">
{ "imports": { "three": "/static/three.module.js" } }
</script>
</head>
<body>
<header>
  <h1>三维机械臂 · 安全环境</h1>
  <span class="badge sim" id="src">仿真</span>
  <span class="badge" id="mode">—</span>
  <a class="badge" href="/demo">灵巧手 →</a>
  <a class="badge" href="/camera">摄像头 →</a>
  <span class="spacer muted">Dobot Nova 5 · 原厂 STL · 碰撞代理 · three.js</span>
</header>

<main>
  <div id="stage">
    <div id="readout"></div>
    <div id="hud">
      <div class="state" id="hudState">SAFE</div>
      <div class="num" id="hudDist">—</div>
      <div class="pair" id="hudPair">等待模型…</div>
      <div class="bar"><i id="hudBar"></i></div>
    </div>
    <div id="hint">左键旋转 · 滚轮缩放 · 右键平移</div>
  </div>

  <aside class="panel">
    <div class="card">
      <h2>数据源</h2>
      <div class="row">
        <button id="btnFollow" class="on">跟随真机</button>
        <button id="btnSim">手动 / 仿真</button>
        <button id="btnAttach">重连真机</button>
      </div>
      <div class="kv"><span>状态</span><span id="srcText">仿真（滑块）</span></div>
      <div class="kv"><span>帧率</span><span id="fps">—</span></div>
    </div>

    <div class="card">
      <h2>机械臂使能</h2>
      <div class="row" style="align-items:center">
        <button id="btnEnable" class="primary">使能</button>
        <button id="btnDisable" class="danger" disabled>下使能</button>
        <span class="pill" style="margin-left:auto">
          <span class="dot" id="enDot"></span><span id="enText">未使能</span>
        </span>
      </div>
      <div class="kv" style="margin-top:7px"><span>模式</span><b id="enMode">—</b></div>
      <div class="kv"><span>速度因子</span><b id="enSpeedNow">—</b></div>
      <div class="row" style="margin-top:6px;align-items:center">
        <span class="muted">使能时设为</span>
        <input id="enSpeed" type="number" min="1" max="100" step="1" value="20"
               style="width:62px;background:#0e1526;border:1px solid var(--line);
                      color:var(--txt);border-radius:7px;padding:4px 6px;
                      font:12px ui-monospace,Consolas,monospace">
        <span class="muted">%</span>
      </div>
      <div class="kv" style="margin-top:6px"><span>结果</span><span id="enMsg" class="muted">—</span></div>
      <div class="row" style="margin-top:10px;align-items:center">
        <button id="btnDrag" disabled>拖动示教</button>
        <span class="pill" style="margin-left:auto">
          <span class="dot" id="dragDot"></span><span id="dragText">未进入</span>
        </span>
      </div>
      <div class="note" style="margin-top:6px">
        点「拖动示教」→ 臂变柔顺，<b>直接用手把臂掰到想要的姿态</b>，不需要写任何
        动作指令；掰到位再点一次退出，姿态会原地保持，之后可以用它当抓取姿态。
        进之前要已使能、且没有报警 / 急停。
      </div>
      <div class="note" style="margin-top:6px">
        使能 = EnableRobot()，下使能 = DisableRobot()；臂在「下使能」时先
        RequestControl() 接管 TCP，再设速度因子。发完都用 30004 的 EnableStatus
        复核；真机不在线 / 有报警 / 急停未复位 / 正在运动时不会发指令。
        第一次上线速度别超过 20%。
      </div>
    </div>

    <div class="card">
      <h2>安全状态</h2>
      <div class="kv"><span>最小间距</span><b id="sMin">—</b></div>
      <div class="kv"><span>最近的一对</span><span id="sPair" class="mono">—</span></div>
      <div class="kv"><span>到操作员</span><b id="sHuman">—</b></div>
      <div class="kv"><span>SSM 要求</span><b id="sSsm">—</b></div>
      <div class="kv"><span>当前 TCP 速度</span><b id="sSpeed">—</b></div>
      <div class="note" style="margin-top:7px">
        红色 ≤ <span id="thSafe">60</span> mm 拦截，黄色 &lt; <span id="thWarn">150</span> mm 提醒。
        数值来自胶囊代理与工位几何（<span class="mono">handlink/cell.py</span>），不是装饰。
      </div>
    </div>

    <div class="card">
      <h2>相机实时环境</h2>
      <div class="kv"><span>桌面平面标定</span><b id="scCal">—</b></div>
      <div class="kv"><span>重投影误差</span><b id="scRms">—</b></div>
      <div class="kv"><span>桌上物体</span><span id="scCount" class="mono">—</span></div>
      <div class="kv"><span>视觉延迟</span><b id="scAge">—</b></div>
      <label class="check"><input type="checkbox" id="cbCam" checked> 在桌面叠加相机画面</label>
      <div class="note">
        标定入口在 <a href="/vision" style="color:#8ab4ff">/vision</a> 的「点桌面四角」。
        相机看到的物体进的是**碰撞模型**（<span class="mono">/arm/check</span> 和服务端下发
        闸门都用它），不是只画个框。
      </div>
    </div>

    <div class="card">
      <h2>关节角（度）</h2>
      <div id="joints"></div>
      <div class="row" style="margin-top:8px">
        <button data-pose="home">零位</button>
        <button data-pose="ready">预备</button>
        <button data-pose="reach">前伸</button>
        <button data-pose="side">侧伸</button>
        <button data-pose="fold">折叠</button>
      </div>
    </div>

    <div class="card">
      <h2>保存的姿态</h2>
      <div class="row">
        <input id="poseName" class="txt" type="text" maxlength="40" autocomplete="off"
               placeholder="名字（留空自动编号，例如 桌面抓取）">
      </div>
      <div class="row" style="margin-top:6px">
        <button id="btnSavePose">保存当前姿态</button>
        <button id="btnReloadPoses">刷新列表</button>
        <span class="pill" style="margin-left:auto"><span id="poseCount" class="muted">0 个</span></span>
      </div>
      <div class="saved" id="savedPoses"></div>
      <div class="kv" style="margin-top:9px"><span>结果</span><span id="saveMsg" class="muted">—</span></div>
      <div class="note" style="margin-top:6px">
        存的是<b>六个关节角</b>（跟随时取真机 30004 的实测值，手动 / 仿真时取滑块上的值），
        另外记一份末端位姿方便人认。点名字回填到滑块 → 校验 → 再下发；点 × 删除。
        重名就是覆盖，文件写在服务端（见下面路径），重启后还在。
      </div>
      <div class="kv" style="margin-top:6px"><span>文件</span>
        <span id="posePath" class="mono" style="font-size:11px;text-align:right;word-break:break-all">—</span>
      </div>
    </div>

    <div class="card">
      <h2>组动作</h2>
      <div class="row" id="actRow" style="flex-wrap:wrap;gap:6px"></div>
      <div class="kv" style="margin-top:6px"><span>进度</span><span id="actMsg" class="muted">—</span></div>
      <div class="note">
        组动作是<b>一串关节路点</b>（<span class="mono">handlink/arm_actions.py</span>），
        全部相对「现在这个姿态」算增量，所以臂站在哪都能做。仿真里点一下先看动画；
        真机要已使能，每段都过手腕防缠 + 碰撞闸门，任一段不过就地停下。
      </div>
    </div>

    <div class="card">
      <h2>末端位姿（mm / deg）</h2>
      <table id="pose"></table>
      <div class="muted" id="poseSrc">模型正解</div>
    </div>

    <div class="card">
      <h2>安全环境</h2>
      <label class="check"><input type="checkbox" id="cbProxies"> 显示碰撞代理（胶囊）</label>
      <label class="check"><input type="checkbox" id="cbKeepout" checked> 显示禁入区</label>
      <label class="check"><input type="checkbox" id="cbTrail"> 显示上次校验的 TCP 轨迹</label>
      <div class="kv" style="margin-top:6px"><span>障碍</span><span id="sObs" class="mono">—</span></div>
      <div class="note">桌面 1.4×0.9 m、操作员站在桌前、料盒/相机立柱/防护栏都在碰撞模型里。</div>
    </div>

    <div class="card">
      <h2>灵巧手</h2>
      <div class="kv"><span>来源</span><span id="handSrc">—</span></div>
      <div class="kv"><span>形态</span><span id="handShape">—</span></div>
    </div>

    <div class="card">
      <h2>手腕走线（电缆防缠）</h2>
      <div class="kv"><span>J4 累计扭转</span><b id="tw4">—</b></div>
      <div class="kv"><span>J6 累计扭转</span><b id="tw6">—</b></div>
      <div class="note" style="margin-top:6px">
        J4/J6 每转一圈，手里的线就多拧一圈。下发时会把目标换成**等效角**
        （可以反方向转），并把累计扭转限制在 ±360°；超了就只接受能回缠的方向。
      </div>
      <button id="btnCableMark" style="margin-top:7px">把当前姿态标记为「没缠」</button>
    </div>

    <div class="card">
      <h2>校验后下发</h2>
      <div class="note" style="margin-bottom:8px">
        MovJ 是关节空间直线插值，所以校验扫的是**整条路径**；
        服务端在真正下发前还会再算一遍，低于红线直接拒绝。
      </div>
      <button id="btnCheck" disabled>校验到当前滑块的路径</button>
      <div class="row" style="margin-top:8px">
        <button id="btnSend" class="danger" disabled>让真机走到这个姿态</button>
        <button id="btnForce" class="danger" disabled
                title="force=true：跳过碰撞红线/视觉时效检查直接下发">强制下发 force</button>
      </div>
      <div class="note" style="margin-top:7px">
        「强制下发」= `force=true`：只跳过<b>碰撞红线</b>和视觉时效，手腕超缠、未使能、
        报警、运动中服务端照样拦。红线报警时先确认现场，再点它。
      </div>
      <div class="kv" style="margin-top:8px"><span>结果</span><span id="sendMsg">—</span></div>
    </div>
  </aside>
</main>

<script>
  window.__booted = false;
  window.addEventListener("error", (e) => {
    const el = document.getElementById("readout");
    if (el) el.textContent = "JS 错误：" + (e.message || e.type) + (e.filename ? "\n" + e.filename : "");
  }, true);
  setTimeout(() => {
    if (!window.__booted) {
      const el = document.getElementById("readout");
      el.textContent = "模块没有启动（多半是 /static/ 下的 three.js 没取到）";
    }
  }, 4000);
</script>

<script type="module">
import * as THREE from "three";
import { OrbitControls } from "/static/OrbitControls.js";
import { RoomEnvironment } from "/static/RoomEnvironment.js";

window.__booted = true;

const DEG = Math.PI / 180;
const stage = document.getElementById("stage");
const $ = (id) => document.getElementById(id);

let MODEL = null;
let CELL = null;
let PROXIES = [];
let TOOLCAPS = [];
let TH = { safe_mm: 40, warn_mm: 100 };

let joints = [0, 0, 0, 0, 0, 0];
let target = [0, 0, 0, 0, 0, 0];
let following = true;
let live = false;
let armEnabled = false;
let armModeName = "";
let armSpeedScaling = 0;
let armBusy = false;
let armDragging = false;
let handCurl = [0.25, 0.25, 0.25, 0.25, 0.3];
let tcpSpeed = 0;
let checkedPath = null;          // 上次校验通过的姿态
let trailLine = null;

// ---------------------------------------------------------------- 3x3 / FK
const I3 = [[1,0,0],[0,1,0],[0,0,1]];
const mv = (R, v) => [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
                      R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
                      R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
const mm3 = (A, B) => A.map(r => [0,1,2].map(j => r[0]*B[0][j] + r[1]*B[1][j] + r[2]*B[2][j]));
const add3 = (a, b) => [a[0]+b[0], a[1]+b[1], a[2]+b[2]];

function rpyMat(r, p, y) {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
    [-sp,   cp*sr,            cp*cr],
  ];
}
function rotZMat(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [[c,-s,0],[s,c,0],[0,0,1]];
}

/** 7 个连杆坐标系（URDF 基座系，米）：base_link + Link1..Link6。 */
function linkFrames(q) {
  let R = [[1,0,0],[0,1,0],[0,0,1]];
  let p = [0,0,0];
  const out = [{ p: p.slice(), R: R.map(r => r.slice()) }];
  for (let i = 0; i < 6; i++) {
    const o = MODEL.joint_origins[i];
    p = add3(p, mv(R, o.xyz));
    R = mm3(R, rpyMat(o.rpy[0], o.rpy[1], o.rpy[2]));
    R = mm3(R, rotZMat(q[i] * DEG));
    out.push({ p: p.slice(), R: R.map(r => r.slice()) });
  }
  return out;
}
function toMatrix4(fr) {
  const m = new THREE.Matrix4();
  m.set(fr.R[0][0], fr.R[0][1], fr.R[0][2], fr.p[0],
        fr.R[1][0], fr.R[1][1], fr.R[1][2], fr.p[1],
        fr.R[2][0], fr.R[2][1], fr.R[2][2], fr.p[2],
        0, 0, 0, 1);
  return m;
}

// ---------------------------------------------------------------- 场景
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(2, devicePixelRatio || 1));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.12;
stage.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x080d18);
scene.fog = new THREE.Fog(0x080d18, 3.6, 9.5);

const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 100);
camera.position.set(-1.90, 1.45, 2.05);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.10, 0.46, 0.02);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.minDistance = 0.55;
controls.maxDistance = 6.5;
controls.maxPolarAngle = Math.PI * 0.495;
controls.autoRotate = false;          // 不自动环绕：视角只跟用户操作走
controls.update();

const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.62;

scene.add(new THREE.HemisphereLight(0x9fc0ff, 0x080d18, 0.72));
const key = new THREE.DirectionalLight(0xffffff, 2.15);
key.position.set(-2.2, 3.6, 2.4);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.left = -1.9; key.shadow.camera.right = 1.9;
key.shadow.camera.top = 1.9; key.shadow.camera.bottom = -1.9;
key.shadow.camera.near = 0.2; key.shadow.camera.far = 12;
key.shadow.bias = -0.0005;
key.shadow.normalBias = 0.02;
key.shadow.radius = 3;
scene.add(key);
const rim = new THREE.DirectionalLight(0x7aa2ff, 1.0);
rim.position.set(-2.4, 1.6, -2.2);
scene.add(rim);
const fill = new THREE.DirectionalLight(0xffe9d0, 0.5);
fill.position.set(0.4, 1.4, 3.2);
scene.add(fill);

// URDF 是 Z 向上，three 是 Y 向上：整个场景一起转，物体内部仍用 URDF 坐标
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

// 地面 + 网格（three 世界坐标，不算在 world 里）
const FLOOR = -0.72;
const floorMesh = new THREE.Mesh(
  new THREE.CircleGeometry(4.2, 72),
  new THREE.MeshStandardMaterial({ color: 0x0d1422, roughness: 0.96, metalness: 0.04 })
);
floorMesh.rotation.x = -Math.PI / 2;
floorMesh.position.y = FLOOR;
floorMesh.receiveShadow = true;
scene.add(floorMesh);
const grid = new THREE.GridHelper(9, 45, 0x1d3a63, 0x131e33);
grid.position.y = FLOOR + 0.0015;
grid.material.transparent = true;
grid.material.opacity = 0.55;
scene.add(grid);

// ---------------------------------------------------------------- 材质
const MAT = {
  white:  new THREE.MeshPhysicalMaterial({ color: 0xe9ebf1, roughness: 0.38, metalness: 0.06,
            clearcoat: 0.45, clearcoatRoughness: 0.45, envMapIntensity: 1.0 }),
  blue:   new THREE.MeshPhysicalMaterial({ color: 0x3d72c4, roughness: 0.42, metalness: 0.08,
            clearcoat: 0.35, clearcoatRoughness: 0.5 }),
  dark:   new THREE.MeshStandardMaterial({ color: 0x272c35, roughness: 0.42, metalness: 0.55 }),
  steel:  new THREE.MeshStandardMaterial({ color: 0xb7bec9, roughness: 0.26, metalness: 0.92 }),
  table:  new THREE.MeshStandardMaterial({ color: 0xcfd6e2, roughness: 0.55, metalness: 0.12 }),
  leg:    new THREE.MeshStandardMaterial({ color: 0x59637a, roughness: 0.5, metalness: 0.55 }),
  wood:   new THREE.MeshStandardMaterial({ color: 0x6b7f9e, roughness: 0.8 }),
  crate:  new THREE.MeshStandardMaterial({ color: 0x2f6f8f, roughness: 0.62, metalness: 0.1 }),
  fence:  new THREE.MeshStandardMaterial({ color: 0x33415c, roughness: 0.5, metalness: 0.55,
            transparent: true, opacity: 0.92 }),
  human:  new THREE.MeshStandardMaterial({ color: 0x4b5f83, roughness: 0.75 }),
  head:   new THREE.MeshStandardMaterial({ color: 0xc9a68b, roughness: 0.8 }),
  keepout:new THREE.MeshBasicMaterial({ color: 0xff5a5a, transparent: true, opacity: 0.10,
            side: THREE.DoubleSide, depthWrite: false }),
};

function mesh(geo, mat, cast = true) {
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = cast; m.receiveShadow = true;
  return m;
}

// ---------------------------------------------------------------- 工位
const cellGroup = new THREE.Group();
world.add(cellGroup);
const keepoutMeshes = [];

function boxAt(name, size, center, mat) {
  const m = mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), mat);
  m.position.set(center[0], center[1], center[2]);
  m.name = name;
  cellGroup.add(m);
  return m;
}
function cylAt(name, r, h, base, mat) {
  const m = mesh(new THREE.CylinderGeometry(r, r, h, 28), mat);
  m.rotation.x = Math.PI / 2;
  m.position.set(base[0], base[1], base[2] + h / 2);
  m.name = name;
  cellGroup.add(m);
  return m;
}

function buildCell() {
  const table = CELL.table;
  boxAt("桌面", table.size, table.center, MAT.table);
  // 桌腿：从地面到桌面下沿
  const lx = table.size[0] / 2 - 0.06, ly = table.size[1] / 2 - 0.06;
  const legH = -0.04 - FLOOR;
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    const leg = mesh(new THREE.CylinderGeometry(0.022, 0.026, legH, 16), MAT.leg);
    leg.position.set(sx * lx, sy * ly, FLOOR + legH / 2);
    leg.rotation.x = Math.PI / 2;
    cellGroup.add(leg);
  }
  // 障碍物
  for (const ob of (CELL.obstacles || [])) {
    if (ob.type === "box") boxAt(ob.name, ob.size, ob.center,
                                 ob.name === "料盒" ? MAT.crate : MAT.fence);
    else if (ob.type === "cylinder") cylAt(ob.name, ob.radius, ob.height, ob.base, MAT.leg);
  }
  // 人
  const human = CELL.human;
  if (human) {
    for (const cap of human.capsules) {
      const a = cap[0], b = cap[1], r = cap[2];
      const dir = new THREE.Vector3(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
      const len = dir.length();
      const m = mesh(new THREE.CapsuleGeometry(r, Math.max(0.01, len), 8, 20), MAT.human);
      m.position.set((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2);
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
      cellGroup.add(m);
    }
    // 头
    const headCap = human.capsules[2] || human.capsules[0];
    const hx = (headCap[0][0] + headCap[1][0]) / 2;
    const hy = (headCap[0][1] + headCap[1][1]) / 2;
    const hz = (headCap[0][2] + headCap[1][2]) / 2;
    const head = mesh(new THREE.SphereGeometry(0.105, 28, 20), MAT.head);
    head.position.set(hx, hy, hz);
    cellGroup.add(head);
    // 禁入柱
    const r = human.keepout_radius, top = human.keepout_top;
    const ko = new THREE.Mesh(new THREE.CylinderGeometry(r, r, top - FLOOR, 40, 1, true),
                              MAT.keepout);
    ko.rotation.x = Math.PI / 2;
    ko.position.set(human.capsules[0][0][0], human.capsules[0][0][1], (top + FLOOR) / 2);
    cellGroup.add(ko);
    keepoutMeshes.push(ko);
  }
}

// ---------------------------------------------------------------- 相机实时环境
//
// 数据来自 /scene/state（见 handlink/scene.py）：桌面平面单应 H 把像素换到基座
// 坐标，相机看到的物体已经算好世界位姿。这里只负责画：
//   * liveGroup：桌上的实时物体（橙=视觉，青=手动放置），和 cell.py 的静态工位分开，
//     但会合并进本地安全 HUD 的 CELL.obstacles，保证 HUD 和服务端算法一致；
//   * camPlane：把 /camera 的画面按 H 反投影贴回桌面——魔方贴纸和方块对上了，
//     说明标定是对的；对不上就是角点顺序/旋转选错了。
const LIVE_MAT = new THREE.MeshStandardMaterial({
  color: 0xff9f43, transparent: true, opacity: 0.55, roughness: 0.5, metalness: 0.05,
  side: THREE.DoubleSide });
const LIVE_MAT_MANUAL = new THREE.MeshStandardMaterial({
  color: 0x35d0c0, transparent: true, opacity: 0.5, roughness: 0.55, metalness: 0.05,
  side: THREE.DoubleSide });
const LIVE_EDGE = new THREE.LineBasicMaterial({ color: 0xffc078, transparent: true, opacity: 0.9 });
const liveGroup = new THREE.Group();
world.add(liveGroup);

let SCENE = null;
let BASE_OBS = null;
let liveSig = "";
let camPlane = null, camTexture = null, camImg = null, camOn = true;

function liveObstacle(o) {
  if (o.kind === "cylinder") {
    return { name: o.name, type: "cylinder", radius: o.size[0], height: o.size[1],
             base: [o.center[0], o.center[1], 0] };
  }
  const w = o.size[0], d = o.size[1], h = o.size[2];
  const c = Math.abs(Math.cos((o.yaw || 0) * DEG));
  const s = Math.abs(Math.sin((o.yaw || 0) * DEG));
  return { name: o.name, type: "box", size: [w * c + d * s, w * s + d * c, h],
           center: [o.center[0], o.center[1], h / 2] };
}

function disposeGroup(g) {
  for (const c of g.children.slice()) {
    g.remove(c);
    if (c.geometry) c.geometry.dispose();
  }
}

function buildLive() {
  const objs = (SCENE && SCENE.objects) || [];
  const sig = JSON.stringify(objs.map(o => [o.name, o.kind, o.center, o.size, o.yaw, o.source]));
  if (sig === liveSig) return;
  liveSig = sig;
  disposeGroup(liveGroup);
  for (const o of objs) {
    const mat = o.source === "vision" ? LIVE_MAT : LIVE_MAT_MANUAL;
    let geo, m;
    if (o.kind === "cylinder") {
      geo = new THREE.CylinderGeometry(o.size[0], o.size[0], o.size[1], 24);
      m = new THREE.Mesh(geo, mat);
      m.position.set(o.center[0], o.center[1], o.size[1] / 2);
      m.rotation.x = Math.PI / 2;
    } else {
      geo = new THREE.BoxGeometry(o.size[0], o.size[1], o.size[2]);
      m = new THREE.Mesh(geo, mat);
      m.position.set(o.center[0], o.center[1], o.size[2] / 2);
      m.rotation.z = (o.yaw || 0) * DEG;
    }
    m.castShadow = true; m.receiveShadow = true;
    liveGroup.add(m);
    const edge = new THREE.LineSegments(new THREE.EdgesGeometry(geo), LIVE_EDGE);
    edge.position.copy(m.position);
    edge.rotation.copy(m.rotation);
    liveGroup.add(edge);
  }
}

function setupCamPlane() {
  const size = (CELL.table && CELL.table.size) || [1.4, 0.9, 0.04];
  const geo = new THREE.PlaneGeometry(size[0], size[1], 28, 18);
  camImg = new Image();
  camTexture = new THREE.Texture(camImg);
  camTexture.colorSpace = THREE.SRGBColorSpace;
  camTexture.minFilter = THREE.LinearFilter;
  camTexture.magFilter = THREE.LinearFilter;
  camImg.onload = () => {
    camTexture.needsUpdate = true;
    if (camPlane && camPlane.visible) applyCamUV();
  };
  camPlane = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
    map: camTexture, transparent: true, opacity: 0.62, depthWrite: false,
    side: THREE.DoubleSide }));
  camPlane.position.set(0, 0, 0.002);
  camPlane.renderOrder = 2;
  camPlane.visible = false;
  world.add(camPlane);
}

/** 桌面网格的每个顶点按 H⁻¹ 算回像素坐标，作为纹理 UV（分段线性近似投影贴图）。 */
function applyCamUV() {
  if (!camPlane || !SCENE || !SCENE.inverse || !camImg || !camImg.naturalWidth) return;
  const H = SCENE.inverse;
  const W = camImg.naturalWidth, Hh = camImg.naturalHeight;
  const pos = camPlane.geometry.attributes.position;
  const uv = camPlane.geometry.attributes.uv;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i);
    const w = H[2][0] * x + H[2][1] * y + H[2][2];
    if (Math.abs(w) < 1e-9) continue;
    const u = (H[0][0] * x + H[0][1] * y + H[0][2]) / w;
    const v = (H[1][0] * x + H[1][1] * y + H[1][2]) / w;
    uv.setXY(i, u / W, 1 - v / Hh);
  }
  uv.needsUpdate = true;
}

function updateSceneUI() {
  const cal = (SCENE && SCENE.calibration) || {};
  const ok = !!(SCENE && SCENE.calibrated);
  $("scCal").textContent = ok ? "已标定" : "未标定";
  $("scCal").style.color = ok ? "#86efac" : "#fca5a5";
  $("scRms").textContent = (cal.rms_mm != null) ? (cal.rms_mm + " mm") : "—";
  const objs = (SCENE && SCENE.objects) || [];
  $("scCount").textContent = objs.length
    ? objs.length + " 个：" + objs.map(o => o.name).join(" · ")
    : "—（相机没认出物体）";
  const age = SCENE && SCENE.vision ? SCENE.vision.age : null;
  $("scAge").textContent = (age == null) ? "—" : age.toFixed(1) + " s";
  const stale = !!(SCENE && SCENE.summary && SCENE.summary.vision_stale);
  $("scAge").style.color = stale ? "#f87171" : "#cbd6f0";
  $("sObs").textContent = ((CELL && CELL.obstacles) || []).map(o => o.name).join(" · ");
}

async function pollScene() {
  try {
    SCENE = await (await fetch("/scene/state")).json();
  } catch (e) {
    return;
  }
  // 本地 HUD 和服务端用同一份障碍物：静态工位 + 相机看到的
  if (BASE_OBS) {
    CELL.obstacles = BASE_OBS.concat(((SCENE && SCENE.objects) || []).map(liveObstacle));
  }
  updateSceneUI();
  buildLive();
  const vis = !!(SCENE && SCENE.calibrated && camOn
                 && SCENE.camera && SCENE.camera.running);
  if (camPlane) {
    camPlane.visible = vis;
    if (vis) applyCamUV();
  }
}

function pollCamFrame() {
  if (!camPlane || !camPlane.visible) return;
  camImg.src = "/camera/frame.jpg?t=" + Date.now();
}

// ---------------------------------------------------------------- 机械臂
const robot = new THREE.Group();
world.add(robot);
const linkMeshes = [];
const linkMat = new THREE.MeshPhysicalMaterial({
  vertexColors: true, roughness: 0.38, metalness: 0.10,
  clearcoat: 0.4, clearcoatRoughness: 0.45, envMapIntensity: 1.0,
});

function parseSTL(buf) {
  const dv = new DataView(buf);
  const n = dv.getUint32(80, true);
  const pos = new Float32Array(n * 9);
  const nor = new Float32Array(n * 9);
  let o = 84;
  for (let i = 0; i < n; i++) {
    const nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
    o += 12;
    for (let v = 0; v < 3; v++) {
      const k = i * 9 + v * 3;
      pos[k] = dv.getFloat32(o, true);
      pos[k+1] = dv.getFloat32(o + 4, true);
      pos[k+2] = dv.getFloat32(o + 8, true);
      nor[k] = nx; nor[k+1] = ny; nor[k+2] = nz;
      o += 12;
    }
    o += 2;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  g.setAttribute("normal", new THREE.BufferAttribute(nor, 3));
  g.computeBoundingSphere();
  return g;
}

const COL = {
  white: new THREE.Color(0xe9ebf1),
  blue:  new THREE.Color(0x3d72c4),
  dark:  new THREE.Color(0x252a33),
  steel: new THREE.Color(0xaeb6c2),
};
function smooth(a, b, x) {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
}
/** 按连杆自身的局部坐标刷颜色：白壳 + 蓝罩，位置和真机同款。 */
function paintLink(name, geo) {
  const pos = geo.attributes.position;
  const colors = new Float32Array(pos.count * 3);
  const c = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    let base = COL.white, accent = COL.white, t = 0;
    if (name === "base_link") {
      base = COL.white; accent = COL.blue;
      t = smooth(0.090, 0.096, z) * (1 - smooth(0.110, 0.116, z));
      if (z < 0.008) { base = COL.dark; accent = COL.dark; }
    } else if (name === "Link1") {
      base = COL.white; accent = COL.blue;
      t = smooth(-0.004, 0.010, z);
    } else if (name === "Link2") {
      base = COL.blue; accent = COL.blue;
    } else if (name === "Link3") {
      base = COL.white; accent = COL.white;
      // 靠近手腕的一端有一点灰
      t = smooth(-0.34, -0.37, x) * 0.8;
      accent = COL.steel;
    } else if (name === "Link4") {
      base = COL.white; accent = COL.blue;
      t = smooth(0.012, 0.020, z) * (1 - smooth(0.030, 0.036, z));
    } else if (name === "Link5") {
      base = COL.white; accent = COL.blue;
      t = smooth(0.016, 0.026, z);
    } else {
      base = COL.white; accent = COL.steel;
      t = 1 - smooth(-0.012, -0.004, z);
    }
    c.copy(base).lerp(accent, t);
    colors[i*3] = c.r; colors[i*3+1] = c.g; colors[i*3+2] = c.b;
  }
  geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  return geo;
}

const FILES = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"];

async function buildRobot() {
  for (let i = 0; i < 7; i++) {
    const res = await fetch("/static/nova/" + FILES[i] + ".STL");
    if (!res.ok) throw new Error("取不到 " + FILES[i] + ".STL");
    const geo = paintLink(FILES[i], parseSTL(await res.arrayBuffer()));
    const m = new THREE.Mesh(geo, linkMat);
    m.castShadow = true; m.receiveShadow = true;
    m.matrixAutoUpdate = true;
    robot.add(m);
    linkMeshes.push(m);
  }
  linkMeshes[6].add(buildHand());
}

// ---------------------------------------------------------------- 灵巧手
const fingerChains = [];

function phalanx(len, r0, r1, mat) {
  const g = new THREE.CylinderGeometry(r1, r0, len, 16);
  const m = mesh(g, mat);
  m.rotation.x = Math.PI / 2;
  m.position.z = len / 2;
  return m;
}

function buildHand() {
  const hand = new THREE.Group();
  hand.name = "LG-603";

  // 法兰 + 黑色腕部
  const flange = mesh(new THREE.CylinderGeometry(0.036, 0.036, 0.008, 32), MAT.steel);
  flange.rotation.x = Math.PI / 2; flange.position.z = 0.004; hand.add(flange);
  const wrist = mesh(new THREE.CylinderGeometry(0.031, 0.034, 0.040, 28), MAT.dark);
  wrist.rotation.x = Math.PI / 2; wrist.position.z = 0.028; hand.add(wrist);

  // 掌：把胶囊压扁
  const palm = mesh(new THREE.CapsuleGeometry(0.023, 0.050, 10, 24), MAT.white);
  palm.scale.set(1.65, 1.0, 0.62);
  palm.rotation.x = Math.PI / 2;
  palm.position.set(0, 0, 0.078);
  hand.add(palm);

  // 掌背的黑色腱鞘
  const back = mesh(new THREE.BoxGeometry(0.060, 0.010, 0.040), MAT.dark);
  back.position.set(0, -0.016, 0.076);
  hand.add(back);

  const xs = [-0.030, -0.010, 0.010, 0.030];
  for (let i = 0; i < 4; i++) {
    const base = new THREE.Group();
    base.position.set(xs[i], 0, 0.108);
    hand.add(base);

    const knuckle = mesh(new THREE.SphereGeometry(0.0095, 14, 10), MAT.dark);
    base.add(knuckle);

    const seg1 = new THREE.Group(); base.add(seg1);
    seg1.add(phalanx(0.030, 0.0085, 0.0078, MAT.steel));
    const j2 = new THREE.Group(); j2.position.z = 0.031; seg1.add(j2);
    j2.add(mesh(new THREE.SphereGeometry(0.0082, 12, 9), MAT.dark));
    j2.add(phalanx(0.024, 0.0076, 0.0070, MAT.steel));
    const j3 = new THREE.Group(); j3.position.z = 0.025; j2.add(j3);
    j3.add(mesh(new THREE.SphereGeometry(0.0076, 12, 9), MAT.dark));
    j3.add(phalanx(0.019, 0.0070, 0.0060, MAT.steel));
    const pad = mesh(new THREE.SphereGeometry(0.0072, 12, 9), MAT.dark);
    pad.position.z = 0.020; pad.scale.set(1, 0.8, 1.2); j3.add(pad);

    fingerChains.push({ seg1, j2, j3, idx: i });
  }

  // 拇指：从掌侧斜着出去
  const thumb = new THREE.Group();
  thumb.position.set(-0.040, 0.002, 0.052);
  thumb.rotation.set(0.35, -0.55, 0);
  hand.add(thumb);
  thumb.add(mesh(new THREE.SphereGeometry(0.011, 14, 10), MAT.dark));
  const t1 = new THREE.Group(); thumb.add(t1);
  t1.add(phalanx(0.030, 0.0105, 0.0095, MAT.white));
  const tj = new THREE.Group(); tj.position.z = 0.031; t1.add(tj);
  tj.add(mesh(new THREE.SphereGeometry(0.0095, 12, 9), MAT.dark));
  tj.add(phalanx(0.024, 0.0090, 0.0078, MAT.steel));
  const tp = mesh(new THREE.SphereGeometry(0.0082, 12, 9), MAT.dark);
  tp.position.z = 0.026; tp.scale.set(1, 0.85, 1.15); tj.add(tp);
  fingerChains.push({ seg1: t1, j2: tj, j3: null, idx: 4 });

  hand.userData.back = back;
  return hand;
}

function updateHand() {
  for (const f of fingerChains) {
    const c = handCurl[f.idx] || 0;
    f.seg1.rotation.x = 0.12 + c * 0.95;
    f.j2.rotation.x = 0.10 + c * 1.05;
    if (f.j3) f.j3.rotation.x = 0.08 + c * 0.85;
  }
}

// ---------------------------------------------------------------- 碰撞代理可视化
let proxyLines = null;
function buildProxyLines() {
  const verts = [];
  for (const link of PROXIES) {
    for (const c of link.capsules) verts.push(c[0], c[1], c[2], c[3], c[4], c[5]);
  }
  for (const c of TOOLCAPS) verts.push(c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2]);
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
  const m = new THREE.LineBasicMaterial({ color: 0x54e6ff, transparent: true, opacity: 0.55 });
  proxyLines = new THREE.LineSegments(g, m);
  proxyLines.visible = false;
  world.add(proxyLines);
}

// 最近的一对：红线 + 两个小球
const nearGroup = new THREE.Group();
world.add(nearGroup);
const nearGeo = new THREE.BufferGeometry();
nearGeo.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
const nearLine = new THREE.Line(nearGeo,
  new THREE.LineBasicMaterial({ color: 0xff5a5a }));
const nearBallA = mesh(new THREE.SphereGeometry(0.012, 14, 10),
  new THREE.MeshBasicMaterial({ color: 0xff5a5a }), false);
const nearBallB = nearBallA.clone();
nearGroup.add(nearLine, nearBallA, nearBallB);
nearGroup.visible = false;

// ---------------------------------------------------------------- 安全计算
function worldCapsules(q) {
  const F = linkFrames(q);
  const out = [];
  for (let i = 0; i < PROXIES.length; i++) {
    const p = F[i].p, R = F[i].R;
    for (const c of PROXIES[i].capsules) {
      out.push({
        index: i, link: PROXIES[i].name,
        a: add3(p, mv(R, [c[0], c[1], c[2]])),
        b: add3(p, mv(R, [c[3], c[4], c[5]])),
        r: c[6],
      });
    }
  }
  const p6 = F[6].p, R6 = F[6].R;
  for (const c of TOOLCAPS) {
    out.push({
      index: 6, link: "灵巧手",
      a: add3(p6, mv(R6, c[0])),
      b: add3(p6, mv(R6, c[1])),
      r: c[2],
    });
  }
  return out;
}

function sub3(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function dot3(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function len3(a) { return Math.sqrt(dot3(a, a)); }

function segSeg(p1, q1, p2, q2) {
  const d1 = sub3(q1, p1), d2 = sub3(q2, p2), r = sub3(p1, p2);
  const a = dot3(d1, d1), e = dot3(d2, d2), f = dot3(d2, r);
  const eps = 1e-12;
  let s = 0, t = 0;
  if (a <= eps && e <= eps) return len3(r);
  if (a <= eps) {
    s = 0; t = Math.min(1, Math.max(0, f / e));
  } else {
    const c = dot3(d1, r);
    if (e <= eps) {
      t = 0; s = Math.min(1, Math.max(0, -c / a));
    } else {
      const b = dot3(d1, d2);
      const den = a * e - b * b;
      s = den > eps ? Math.min(1, Math.max(0, (b * f - c * e) / den)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
      else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
    }
  }
  const c1 = [p1[0] + d1[0]*s, p1[1] + d1[1]*s, p1[2] + d1[2]*s];
  const c2 = [p2[0] + d2[0]*t, p2[1] + d2[1]*t, p2[2] + d2[2]*t];
  return { d: len3(sub3(c1, c2)), a: c1, b: c2 };
}

function pointBoxDist(p, center, size, out) {
  const dx = Math.max(Math.abs(p[0]-center[0]) - size[0]/2, 0);
  const dy = Math.max(Math.abs(p[1]-center[1]) - size[1]/2, 0);
  const dz = Math.max(Math.abs(p[2]-center[2]) - size[2]/2, 0);
  out[0] = p[0]; out[1] = p[1]; out[2] = p[2];
  return Math.sqrt(dx*dx + dy*dy + dz*dz);
}
const _tmpP = [0, 0, 0];

function capsuleBox(cap, box) {
  let best = 1e9, bp = null;
  for (let k = 0; k < 5; k++) {
    const t = k / 4;
    const p = [cap.a[0] + (cap.b[0]-cap.a[0])*t,
               cap.a[1] + (cap.b[1]-cap.a[1])*t,
               cap.a[2] + (cap.b[2]-cap.a[2])*t];
    const d = pointBoxDist(p, box.center, box.size, _tmpP);
    if (d < best) { best = d; bp = p; }
  }
  return { d: best - cap.r, p: bp };
}

function planeDist(cap, z) {
  return Math.min(cap.a[2], cap.b[2]) - z - cap.r;
}

/** 当前姿态下，机械臂与工位/自身的最小间距。 */
function evaluate(q) {
  const caps = worldCapsules(q);
  let best = 1e9, pair = ["", ""], bestA = null, bestB = null;
  let humanBest = 1e9;

  const obstacles = (CELL.obstacles || []);
  const human = CELL.human ? CELL.human.capsules : [];

  for (const cap of caps) {
    for (const ob of obstacles) {
      if (ob.type === "box") {
        const hit = capsuleBox(cap, ob);
        if (hit.d < best) {
          best = hit.d; pair = [cap.link, ob.name]; bestA = hit.p;
          const qp = hit.p, c = ob.center, s = ob.size;
          bestB = [Math.min(c[0]+s[0]/2, Math.max(c[0]-s[0]/2, qp[0])),
                   Math.min(c[1]+s[1]/2, Math.max(c[1]-s[1]/2, qp[1])),
                   Math.min(c[2]+s[2]/2, Math.max(c[2]-s[2]/2, qp[2]))];
        }
      } else if (ob.type === "cylinder") {
        const oc = { a: ob.base, b: [ob.base[0], ob.base[1], ob.base[2] + ob.height], r: ob.radius };
        const hit = segSeg(cap.a, cap.b, oc.a, oc.b);
        const d = hit.d - cap.r - oc.r;
        if (d < best) { best = d; pair = [cap.link, ob.name]; bestA = hit.a; bestB = hit.b; }
      }
    }
    for (let i = 0; i < human.length; i++) {
      const hc = human[i];
      const hit = segSeg(cap.a, cap.b, hc[0], hc[1]);
      const d = hit.d - cap.r - hc[2];
      if (d < humanBest) humanBest = d;
      if (d < best) { best = d; pair = [cap.link, "操作员"]; bestA = hit.a; bestB = hit.b; }
    }
    // 底座和肩柱贴着安装面，胶囊又外扩，算桌面会得到恒定假告警，跳过
    if (cap.link !== "base_link" && cap.link !== "Link1") {
      let d = planeDist(cap, CELL.table_top || 0);
      if (d < best) { best = d; pair = [cap.link, "桌面"]; bestA = null; }
      d = planeDist(cap, CELL.floor_z);
      if (d < best) { best = d; pair = [cap.link, "地面"]; bestA = null; }
    }
  }

  // 自碰撞：只查隔了两段以上的连杆
  for (let i = 0; i < caps.length; i++) {
    for (let j = i + 1; j < caps.length; j++) {
      if (Math.abs(caps[i].index - caps[j].index) <= 2) continue;
      const hit = segSeg(caps[i].a, caps[i].b, caps[j].a, caps[j].b);
      const d = hit.d - caps[i].r - caps[j].r;
      if (d < best) {
        best = d; pair = [caps[i].link, caps[j].link];
        bestA = hit.a; bestB = hit.b;
      }
    }
  }
  return { min: best, pair, human: humanBest, a: bestA, b: bestB, caps };
}

// ---------------------------------------------------------------- 渲染循环
let last = performance.now(), acc = 0, frames = 0;
let safety = null, safetyAt = 0;

function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / Math.max(1, h);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(stage);

function updateRobot() {
  const F = linkFrames(joints);
  for (let i = 0; i < 7; i++) {
    const m4 = toMatrix4(F[i]);
    linkMeshes[i].position.setFromMatrixPosition(m4);
    linkMeshes[i].quaternion.setFromRotationMatrix(m4);
  }
  updateHand();
}

function updateProxyLines(caps) {
  if (!proxyLines) return;
  const arr = proxyLines.geometry.attributes.position.array;
  let k = 0;
  for (const c of caps) {
    arr[k++] = c.a[0]; arr[k++] = c.a[1]; arr[k++] = c.a[2];
    arr[k++] = c.b[0]; arr[k++] = c.b[1]; arr[k++] = c.b[2];
  }
  proxyLines.geometry.attributes.position.needsUpdate = true;
}

function updateNearMarker(ev) {
  if (!ev || !ev.a || !ev.b || ev.min > 0.35) { nearGroup.visible = false; return; }
  nearGroup.visible = true;
  const arr = nearLine.geometry.attributes.position.array;
  arr[0] = ev.a[0]; arr[1] = ev.a[1]; arr[2] = ev.a[2];
  arr[3] = ev.b[0]; arr[4] = ev.b[1]; arr[5] = ev.b[2];
  nearLine.geometry.attributes.position.needsUpdate = true;
  nearBallA.position.set(ev.a[0], ev.a[1], ev.a[2]);
  nearBallB.position.set(ev.b[0], ev.b[1], ev.b[2]);
}

function tick(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;

  if (!following) {
    const k = 1 - Math.exp(-dt * 12);
    for (let i = 0; i < 6; i++) joints[i] += (target[i] - joints[i]) * k;
  }
  try {
    updateRobot();
    controls.update();
    renderer.render(scene, camera);
  } catch (err) {
    $("readout").textContent = "渲染错误：" + (err && err.message || err);
    return;
  }

  // 安全评估 8 Hz，够快也不占渲染
  if (now - safetyAt > 120) {
    safetyAt = now;
    safety = evaluate(joints);
    updateHud(safety);
    if (proxyLines && proxyLines.visible) updateProxyLines(safety.caps);
    updateNearMarker(safety);
  }

  acc += dt; frames++;
  if (acc > 0.5) {
    $("fps").textContent = (frames / acc).toFixed(0) + " fps";
    acc = 0; frames = 0;
  }
  updateReadout();
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- HUD
function fmtMm(v) {
  if (!isFinite(v)) return "—";
  return (v * 1000).toFixed(0) + " mm";
}
function updateHud(ev) {
  if (!ev) return;
  const mm = ev.min * 1000;
  const el = $("hudDist");
  if (!isFinite(mm)) {
    el.textContent = "—";
    $("hudState").textContent = "无代理";
    $("hudPair").textContent = "proxies.json 没加载";
    $("sMin").textContent = "—";
    $("sPair").textContent = "—";
    return;
  }
  el.textContent = fmtMm(ev.min);
  const state = ev.min <= 0 ? ["碰撞", "err"] : (mm < TH.safe_mm ? ["危险", "err"]
    : (mm < TH.warn_mm ? ["注意", "warn"] : ["安全", "ok"]));
  $("hudState").textContent = state[0];
  $("hudState").style.color = state[1] === "ok" ? "#4ade80" : (state[1] === "warn" ? "#fbbf24" : "#f87171");
  el.style.color = state[1] === "ok" ? "#4ade80" : (state[1] === "warn" ? "#fbbf24" : "#f87171");
  $("hudPair").textContent = ev.pair[0] + " ↔ " + ev.pair[1];
  const frac = Math.max(0, Math.min(1, mm / (TH.warn_mm * 2)));
  const bar = $("hudBar");
  bar.style.width = (frac * 100).toFixed(0) + "%";
  bar.style.background = state[1] === "ok" ? "#4ade80" : (state[1] === "warn" ? "#fbbf24" : "#f87171");

  $("sMin").textContent = fmtMm(ev.min);
  $("sPair").textContent = ev.pair[0] + " ↔ " + ev.pair[1];
  $("sHuman").textContent = fmtMm(ev.human);

  const ssm = ssmRequired(tcpSpeed);
  $("sSsm").textContent = fmtMm(ssm);
  $("sSpeed").textContent = (tcpSpeed * 1000).toFixed(0) + " mm/s";
  $("sSsm").style.color = ev.human < ssm ? "#f87171" : "#cbd6f0";
}

function ssmRequired(v) {
  // ISO/TS 15066 的简化式：v·t反应 + v²/(2a) + C，参数见 docs / cell.py
  const t = 0.20, a = 3.0, c = 0.10;
  return v * t + (v * v) / (2 * a) + c;
}

function updateCable(c) {
  if (!c || !c.twist_deg) return;
  const lim = Number(c.limit_deg || 360);
  for (const [id, key] of [["tw4", "J4"], ["tw6", "J6"]]) {
    const el = $(id);
    const v = c.twist_deg[key];
    if (v == null) { el.textContent = "—"; continue; }
    el.textContent = (v >= 0 ? "+" : "") + Number(v).toFixed(0) + "° / ±" + lim.toFixed(0) + "°";
    const frac = Math.min(1, Math.abs(Number(v)) / lim);
    el.style.color = frac > 0.9 ? "#f87171" : (frac > 0.7 ? "#fbbf24" : "#cbd6f0");
  }
}

function updateReadout() {
  const F = linkFrames(joints);
  const p = F[6].p;
  $("readout").textContent =
    "J  " + joints.map(v => (v >= 0 ? " " : "") + v.toFixed(1)).join(" ") + "\n" +
    "XYZ " + [p[0]*1000, p[1]*1000, p[2]*1000].map(v => v.toFixed(0).padStart(5)).join(" ") + "\n" +
    "最小间距 " + fmtMm(safety ? safety.min : NaN) + "  ·  " + (safety ? safety.pair.join(" ↔ ") : "");
}

function poseOf(F) {
  const R = F[6].R;
  const sp = Math.max(-1, Math.min(1, -R[2][0]));
  const p = F[6].p;
  return [p[0]*1000, p[1]*1000, p[2]*1000,
          Math.atan2(R[2][1], R[2][2]) / DEG, Math.asin(sp) / DEG, Math.atan2(R[1][0], R[0][0]) / DEG];
}
function updatePoseTable(pose, fromServer) {
  const rows = [["X","mm",pose[0]],["Y","mm",pose[1]],["Z","mm",pose[2]],
                ["RX","°",pose[3]],["RY","°",pose[4]],["RZ","°",pose[5]]];
  $("pose").innerHTML = rows.map(([k,u,v]) =>
    `<tr><td>${k}</td><td>${Number(v).toFixed(1)} ${u}</td></tr>`).join("");
  $("poseSrc").textContent = fromServer ? "来自真机 30004" : "模型正解（与真机差 ~1.6 mm）";
}

// ---------------------------------------------------------------- 保存的姿态
let savedPoses = [];
let savingPose = false;

/** 「当前姿态」指哪个：跟随时是真机实测角，手动 / 仿真时是滑块上正在摆的那个。 */
function poseToSave() {
  const src = (following && live) ? joints : target;
  return src.map(v => +Number(v).toFixed(2));
}

function renderSavedPoses() {
  $("poseCount").textContent = savedPoses.length + " 个";
  const box = $("savedPoses");
  box.innerHTML = "";
  if (!savedPoses.length) {
    const span = document.createElement("span");
    span.className = "empty";
    span.textContent = "还没有保存过姿态：把臂摆到想要的位置，点「保存当前姿态」。";
    box.appendChild(span);
    return;
  }
  savedPoses.forEach(p => {
    const chip = document.createElement("span");
    chip.className = "chip";
    const open = document.createElement("button");
    open.textContent = p.name;
    open.title = "回填到滑块：J " + (p.joints || []).map(v => Number(v).toFixed(1)).join(" / ")
      + "\n末端 " + (p.pose || []).map(v => Number(v).toFixed(0)).join(", ");
    open.onclick = () => recallPose(p);
    const del = document.createElement("button");
    del.className = "del";
    del.textContent = "×";
    del.title = "删除「" + p.name + "」";
    del.onclick = () => removeSavedPose(p.name);
    chip.appendChild(open);
    chip.appendChild(del);
    box.appendChild(chip);
  });
}

async function loadSavedPoses(announce) {
  try {
    const j = await (await fetch("/arm/poses")).json();
    savedPoses = j.poses || [];
    $("posePath").textContent = j.path || "—";
    $("posePath").title = j.path || "";
    renderSavedPoses();
    if (announce) {
      $("saveMsg").textContent = "已重新读取：" + savedPoses.length + " 个姿态";
      $("saveMsg").style.color = "";
    }
  } catch (e) {
    savedPoses = [];
    renderSavedPoses();
    $("saveMsg").textContent = "读不到姿态文件（服务没起来？）";
    $("saveMsg").style.color = "#fca5a5";
  }
}

async function savePose() {
  if (savingPose) return;
  savingPose = true;
  const btn = $("btnSavePose");
  btn.disabled = true;
  const name = ($("poseName").value || "").trim();
  const values = poseToSave();
  const fromRobot = following && live;
  $("saveMsg").textContent = "保存中…";
  $("saveMsg").style.color = "";
  try {
    const r = await fetch("/arm/poses", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: name, joints: values,
                             source: fromRobot ? "robot" : "model" }),
    });
    const j = await r.json();
    if (!j.ok) {
      $("saveMsg").textContent = "保存失败：" + (j.error || "未知错误");
      $("saveMsg").style.color = "#fca5a5";
      return;
    }
    savedPoses = j.poses || savedPoses;
    renderSavedPoses();
    $("poseName").value = "";
    const p = j.pose || {};
    $("saveMsg").textContent = (j.replaced ? "已更新「" : "已保存「") + (p.name || name) + "」："
      + (p.joints || values).map(v => Number(v).toFixed(1)).join(" / ")
      + (fromRobot ? "（真机实测）" : "（滑块）")
      + (j.clamped ? "；有角度被限位夹过" : "");
    $("saveMsg").style.color = "#86efac";
  } catch (e) {
    $("saveMsg").textContent = "请求失败：" + ((e && e.message) || e);
    $("saveMsg").style.color = "#fca5a5";
  } finally {
    savingPose = false;
    btn.disabled = false;
  }
}

function recallPose(p) {
  setFollow(false);
  target = (p.joints || []).slice();
  syncSliders();
  checkedPath = null;
  $("btnSend").disabled = true;
  $("sendMsg").textContent = "已回填「" + p.name + "」，先校验到当前姿态";
  $("sendMsg").style.color = "";
  $("saveMsg").textContent = "已回填「" + p.name + "」到滑块（J "
    + (p.joints || []).map(v => Number(v).toFixed(1)).join(" / ") + "）";
  $("saveMsg").style.color = "#86efac";
}

async function removeSavedPose(name) {
  if (!confirm("删除保存的姿态「" + name + "」？姿态文件里也会删掉。")) return;
  try {
    const j = await (await fetch("/arm/poses/delete", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: name }),
    })).json();
    if (!j.ok) {
      $("saveMsg").textContent = "删除失败：" + (j.error || "未知错误");
      $("saveMsg").style.color = "#fca5a5";
      return;
    }
    savedPoses = j.poses || [];
    renderSavedPoses();
    $("saveMsg").textContent = "已删除「" + name + "」";
    $("saveMsg").style.color = "";
  } catch (e) {
    $("saveMsg").textContent = "请求失败：" + ((e && e.message) || e);
    $("saveMsg").style.color = "#fca5a5";
  }
}

// ---------------------------------------------------------------- UI
const POSES = {
  home:  [0, 0, 0, 0, 0, 0],
  ready: [0, -25, -40, 0, 65, 0],
  reach: [0, -60, -55, 0, 20, 0],
  side:  [80, -35, -50, 0, 60, 0],
  fold:  [0, -60, -120, 120, 80, 0],
};

function buildJoints() {
  const box = $("joints");
  box.innerHTML = "";
  for (let i = 0; i < 6; i++) {
    const lim = MODEL.limits[i];
    const row = document.createElement("div");
    row.className = "joint";
    row.innerHTML = `<b>${MODEL.names[i]}</b>
      <input type="range" min="${lim[0]}" max="${lim[1]}" step="0.5" value="${target[i]}" data-i="${i}">
      <output>${target[i].toFixed(1)}°</output>`;
    box.appendChild(row);
    row.querySelector("input").addEventListener("input", (e) => {
      if (following) setFollow(false);
      target[i] = parseFloat(e.target.value);
      row.querySelector("output").textContent = target[i].toFixed(1) + "°";
      checkedPath = null;
      $("btnSend").disabled = true;
      $("sendMsg").textContent = "姿态变了，先重新校验";
    });
  }
}

function syncSliders() {
  document.querySelectorAll("#joints input").forEach((input, i) => {
    input.value = target[i];
    input.parentElement.querySelector("output").textContent = Number(target[i]).toFixed(1) + "°";
  });
}

function setFollow(on) {
  following = on;
  $("btnFollow").classList.toggle("on", on);
  $("btnSim").classList.toggle("on", !on);
  $("srcText").textContent = on ? "真机 30004（在线时）" : "仿真（滑块）";
}

function updateEnableUI() {
  $("enDot").className = "dot " + (!live ? "" : (armEnabled ? "ok" : "warn"));
  $("enText").textContent = !live ? "真机不在线" : (armEnabled ? "已使能" : "未使能");
  $("enMode").textContent = live ? (armModeName || "—") : "仿真";
  $("enSpeedNow").textContent = live
    ? (Number(armSpeedScaling).toFixed(0) + " %") : "—";
  $("btnEnable").disabled = armBusy || !live || armEnabled;
  $("btnDisable").disabled = armBusy || !live || !armEnabled;
  $("enSpeed").disabled = armBusy || !live || armEnabled;
  $("btnDrag").disabled = armBusy || !live || (!armEnabled && !armDragging);
  $("btnDrag").textContent = armDragging ? "退出拖动（保持姿态）" : "拖动示教";
  $("dragDot").className = "dot " + (!live ? "" : (armDragging ? "ok" : ""));
  $("dragText").textContent = !live ? "真机不在线" : (armDragging ? "拖动中 · 用手掰" : "未进入");
  $("dragText").style.color = armDragging ? "#86efac" : "";
}

async function setArmDrag(on) {
  if (on && !confirm("即将进入拖动示教：电机变柔顺，你就可以用手直接掰臂了。\n确认手边没有夹住的线缆、工作空间里没有别人？")) return;
  armBusy = true;
  updateEnableUI();
  $("dragText").textContent = on ? "进入中…" : "退出中…";
  try {
    const r = await fetch("/arm/drag", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ on: !!on }),
    });
    const j = await r.json();
    if (j.ok) {
      armDragging = !!on;
      $("dragText").textContent = on
        ? "已进入拖动示教，现在用手掰"
        : "已退出，姿态保持" + (j.mode_name ? "（" + j.mode_name + "）" : "");
      $("dragText").style.color = "#86efac";
      armModeName = j.mode_name || armModeName;
    } else {
      $("dragText").textContent = "失败：" + (j.error || "未知错误");
      $("dragText").style.color = "#fca5a5";
    }
  } catch (e) {
    $("dragText").textContent = "请求失败：" + ((e && e.message) || e);
    $("dragText").style.color = "#fca5a5";
  } finally {
    armBusy = false;
    updateEnableUI();
    poll();
  }
}

async function setArmEnabled(on) {
  const pct = Math.max(1, Math.min(100, Math.round(Number($("enSpeed").value) || 20)));
  if (on && !confirm("即将上使能：电机通电、抱闸松开，臂保持在当前位置。确认工作空间里没有人、手不在夹具里？")) return;
  armBusy = true;
  updateEnableUI();
  $("enMsg").textContent = on ? "使能中…" : "下使能中…";
  $("enMsg").style.color = "";
  try {
    const r = await fetch("/arm/enable", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(on ? { on: true, speed: pct } : { on: false }),
    });
    const j = await r.json();
    if (j.ok) {
      $("enMsg").textContent = on
        ? ("已使能，速度因子 " + Number(j.speed_scaling || pct).toFixed(0) + " %")
        : "已下使能（抱闸抱住当前位置）";
      $("enMsg").style.color = "#86efac";
      if (!on) {
        checkedPath = null;
        $("btnSend").disabled = true;
      }
    } else {
      $("enMsg").textContent = "失败：" + (j.error || "未知错误");
      $("enMsg").style.color = "#fca5a5";
    }
  } catch (e) {
    $("enMsg").textContent = "请求失败：" + ((e && e.message) || e);
    $("enMsg").style.color = "#fca5a5";
  } finally {
    armBusy = false;
    updateEnableUI();
    poll();
  }
}

async function poll() {
  try {
    const s = await (await fetch("/arm/state")).json();
    live = !!s.online;
    armEnabled = !!s.enabled;
    armModeName = s.mode_name || "";
    armSpeedScaling = Number(s.speed_scaling || 0);
    updateEnableUI();
    const badge = $("src");
    badge.textContent = live ? "真机在线" : "仿真";
    badge.className = "badge " + (live ? "live" : "sim");
    $("mode").textContent = live
      ? (s.mode_name + (s.enabled ? " · 已使能" : " · 未使能"))
      : "模型";
    tcpSpeed = s.tcp_speed_m_s || 0;

    if (following && live) {
      joints = s.joints.slice();
      target = s.joints.slice();
      syncSliders();
      updatePoseTable(s.pose, true);
    } else {
      updatePoseTable(poseOf(linkFrames(joints)), false);
    }
    const canCheck = !following;
    $("btnCheck").disabled = !canCheck;
    if (!canCheck || !live || !armEnabled) $("btnSend").disabled = true;
    // 强制下发不要求「校验通过」，只要求：手动模式 + 真机在线 + 已使能
    $("btnForce").disabled = !canCheck || !live || !armEnabled;

    if (s.hand) {
      handCurl = s.hand.curl;
      $("handSrc").textContent = s.hand.source || "—";
      $("handShape").textContent = s.hand.shape || "—";
    }
    if (s.cable) updateCable(s.cable);
  } catch (e) { /* 服务没起来也不白屏 */ }
}

function drawTrail(q0, q1) {
  if (trailLine) { world.remove(trailLine); trailLine.geometry.dispose(); trailLine = null; }
  if (!q0 || !q1) return;
  const pts = [];
  const n = 40;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const q = q0.map((v, k) => v + (q1[k] - v) * t);
    const p = linkFrames(q)[6].p;
    pts.push(new THREE.Vector3(p[0], p[1], p[2]));
  }
  const g = new THREE.BufferGeometry().setFromPoints(pts);
  trailLine = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x54e6ff,
    transparent: true, opacity: 0.85 }));
  trailLine.visible = $("cbTrail").checked;
  world.add(trailLine);
}

async function checkPath() {
  const btn = $("btnCheck");
  btn.disabled = true; btn.textContent = "校验中…";
  try {
    const r = await fetch("/arm/check", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        joints: target.map(v => +v.toFixed(2)),
        from: joints.map(v => +v.toFixed(2)),
      }),
    });
    const j = await r.json();
    if (!j.ok) {
      $("sendMsg").textContent = "校验失败：" + (j.error || "未知");
      $("sendMsg").style.color = "#fca5a5";
      return;
    }
    checkedPath = target.slice();
    const mm = (j.min_m * 1000).toFixed(0);
    const cableNote = (j.cable && j.cable.notes && j.cable.notes.length)
      ? "；" + j.cable.notes.join("；") : "";
    $("sendMsg").textContent = (j.safe
      ? `路径安全：最小 ${mm} mm（${j.pair[0]} ↔ ${j.pair[1]}，${j.samples} 个采样点）`
      : `不通过：最小 ${mm} mm（${j.pair[0]} ↔ ${j.pair[1]}），低于红线`) + cableNote;
    $("sendMsg").style.color = j.safe ? "#86efac" : "#fca5a5";
    if (j.safe && !armEnabled) $("sendMsg").textContent += "；先使能机械臂";
    $("btnSend").disabled = !j.safe || !live || !armEnabled;
    drawTrail(joints, j.adjusted_joints || target);
  } catch (e) {
    $("sendMsg").textContent = "请求失败";
  } finally {
    btn.disabled = false; btn.textContent = "校验到当前滑块的路径";
  }
}

async function sendToRobot(force) {
  const warn = force
    ? "⚠ 强制下发（force=true）：会跳过碰撞红线和视觉时效检查，服务端仍会拦手腕超缠、"
      + "未使能、报警、运动中。\n\n确认工作空间里没有人，而且你清楚这条路径的间距？"
    : "真机会按 MovJ 走到这个姿态，确认工作空间里没有人？";
  if (!confirm(warn)) return;
  $("sendMsg").textContent = force ? "强制下发中…" : "下发中…";
  $("sendMsg").style.color = "";
  try {
    const r = await fetch("/arm/joints", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ joints: target.map(v => +v.toFixed(2)), apply: true,
                             force: !!force }),
    });
    const j = await r.json();
    const mm = j.clearance && j.clearance.min_m != null
      ? "，路径最小 " + (j.clearance.min_m * 1000).toFixed(0) + " mm" : "";
    $("sendMsg").textContent = j.moved
      ? ((force ? "已强制下发 MovJ（force=true，跳过了红线）" : "已下发 MovJ") + mm)
      : ("未执行：" + (j.reason || j.error || ""));
    $("sendMsg").style.color = j.moved ? "#86efac" : "#fca5a5";
  } catch (e) {
    $("sendMsg").textContent = "请求失败";
  }
}

// ---------------------------------------------------------------- 组动作
let armActBusy = false;
let actTimer = null;

async function loadArmActions() {
  try {
    const j = await (await fetch("/arm/actions")).json();
    const row = $("actRow");
    row.innerHTML = "";
    (j.actions || []).forEach(a => {
      const b = document.createElement("button");
      b.textContent = a.label;
      b.title = a.description || "";
      b.dataset.act = a.name;
      b.onclick = () => runArmAction(a.name);
      row.appendChild(b);
    });
  } catch (e) { /* 服务没起来：按钮区空着就行 */ }
}

// 仿真里按路点走一遍：三维视图在 sim 模式下就是按 joints 画的。
function animateArmSteps(steps) {
  if (actTimer) clearTimeout(actTimer);
  setFollow(false);
  let i = 0;
  const tick = () => {
    if (i >= steps.length) {
      $("actMsg").textContent = "预览完（真机没动）";
      return;
    }
    const s = steps[i++];
    joints = (s.joints || joints).slice();
    target = joints.slice();
    syncSliders();
    updatePoseTable(poseOf(linkFrames(joints)), false);
    $("actMsg").textContent = "预览 " + (s.label || "") + "（" + i + "/" + steps.length + "）";
    actTimer = setTimeout(tick, Math.max(260, (s.hold_ms || 300)));
  };
  tick();
}

async function runArmAction(name) {
  if (armActBusy) return;
  const real = live && armEnabled;
  if (real && !confirm("真机会从当前姿态开始做这个组动作，确认工作空间里没有人？")) return;
  armActBusy = true;
  if (actTimer) { clearTimeout(actTimer); actTimer = null; }
  $("actMsg").textContent = real ? "真机执行中…（约几秒）" : "生成预览…";
  $("actMsg").style.color = "";
  try {
    const r = await fetch("/arm/action", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name: name, apply: real,
        speed: Math.max(1, Math.min(100, Number($("enSpeed").value) || 20)),
      }),
    });
    const j = await r.json();
    if (!j.ok) {
      let msg = "未执行：" + (j.reason || j.error || "未知");
      if (j.steps_done && j.steps) msg += "（已做 " + j.steps_done.length + "/" + j.steps.length + " 段）";
      $("actMsg").textContent = msg;
      $("actMsg").style.color = "#fca5a5";
      poll();
      return;
    }
    if (j.moved) {
      const mm = (j.min_clearance_mm != null) ? "，最小间距 " + j.min_clearance_mm + " mm" : "";
      $("actMsg").textContent = (j.complete ? "已执行完" : "已执行")
        + " " + (j.steps_done || []).length + "/" + j.steps.length + " 段"
        + "，预计 " + j.estimate_s + " s" + mm;
      $("actMsg").style.color = j.complete ? "#86efac" : "#fca5a5";
      poll();
      return;
    }
    if (!live) {
      animateArmSteps(j.steps);
    } else {
      $("actMsg").textContent = "真机在线但未使能：先「使能」才会动（计划已生成）";
      $("actMsg").style.color = "#fca5a5";
    }
  } catch (e) {
    $("actMsg").textContent = "请求失败：" + ((e && e.message) || e);
    $("actMsg").style.color = "#fca5a5";
  } finally {
    armActBusy = false;
  }
}

// ---------------------------------------------------------------- 启动
async function main() {
  MODEL = await (await fetch("/arm/model")).json();
  const cellAll = await (await fetch("/arm/cell")).json();
  CELL = cellAll.cell;
  PROXIES = cellAll.proxies || [];
  TOOLCAPS = (CELL.tool && CELL.tool.capsules) || [];
  TH = { safe_mm: cellAll.thresholds.safe_mm, warn_mm: cellAll.thresholds.warn_mm };
  $("thSafe").textContent = TH.safe_mm;
  $("thWarn").textContent = TH.warn_mm;
  $("sObs").textContent = (CELL.obstacles || []).map(o => o.name).join(" · ");

  buildCell();
  BASE_OBS = (CELL.obstacles || []).slice();
  setupCamPlane();
  await buildRobot();
  buildProxyLines();
  buildJoints();

  target = POSES.ready.slice();
  joints = POSES.ready.slice();
  syncSliders();
  setFollow(true);
  resize();

  const q = new URLSearchParams(location.search);
  if (q.has("joints")) {
    const arr = q.get("joints").split(",").map(Number);
    if (arr.length === 6 && arr.every(Number.isFinite)) {
      joints = arr.slice(); target = arr.slice(); setFollow(false); syncSliders();
    }
  }

  $("btnFollow").onclick = () => setFollow(true);
  $("btnSim").onclick = () => setFollow(false);
  $("btnEnable").onclick = () => setArmEnabled(true);
  $("btnDisable").onclick = () => setArmEnabled(false);
  $("btnDrag").onclick = () => setArmDrag(!armDragging);
  updateEnableUI();
  $("btnAttach").onclick = async () => {
    const btn = $("btnAttach");
    btn.disabled = true; btn.textContent = "连接中…";
    try {
      const j = await (await fetch("/arm/attach", { method: "POST",
        headers: { "Content-Type": "application/json" }, body: "{}" })).json();
      $("srcText").textContent = j.ok ? "真机 30004（已重连）" : "连不上：" + (j.error || "");
    } finally {
      btn.disabled = false; btn.textContent = "重连真机";
      poll();
    }
  };
  document.querySelectorAll("[data-pose]").forEach(b => b.onclick = () => {
    setFollow(false);
    target = POSES[b.dataset.pose].slice();
    syncSliders();
    checkedPath = null;
    $("btnSend").disabled = true;
    $("sendMsg").textContent = "姿态变了，先重新校验";
  });
  $("btnCheck").onclick = checkPath;
  $("btnSavePose").onclick = savePose;
  $("btnReloadPoses").onclick = () => loadSavedPoses(true);
  $("poseName").addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); savePose(); }
  });
  $("btnCableMark").onclick = async () => {
    try {
      const j = await (await fetch("/arm/cable/mark", { method: "POST",
        headers: { "Content-Type": "application/json" }, body: "{}" })).json();
      if (j.ok) updateCable(j);
      $("sendMsg").textContent = "已把当前姿态标记为「没缠」的基准（累计清零）";
      $("sendMsg").style.color = "#86efac";
    } catch (e) {
      $("sendMsg").textContent = "标记失败";
    }
  };
  $("btnSend").onclick = () => sendToRobot(false);
  $("btnForce").onclick = () => sendToRobot(true);
  $("cbProxies").onchange = (e) => {
    if (proxyLines) proxyLines.visible = e.target.checked;
    if (e.target.checked && safety) updateProxyLines(safety.caps);
  };
  $("cbKeepout").onchange = (e) => {
    keepoutMeshes.forEach(m => { m.visible = e.target.checked; });
  };
  $("cbTrail").onchange = (e) => {
    if (trailLine) trailLine.visible = e.target.checked;
  };
  $("cbCam").onchange = (e) => {
    camOn = e.target.checked;
    const vis = !!(camOn && SCENE && SCENE.calibrated
                   && SCENE.camera && SCENE.camera.running);
    if (camPlane) {
      camPlane.visible = vis;
      if (vis) applyCamUV();
    }
  };

  requestAnimationFrame(tick);
  poll();
  setInterval(poll, 150);
  loadArmActions();
  loadSavedPoses(false);
  pollScene();
  setInterval(pollScene, 400);
  setInterval(pollCamFrame, 450);
}

main().catch(err => {
  $("readout").textContent = "启动失败：" + (err && err.message || err);
});
</script>
</body>
</html>
"""
