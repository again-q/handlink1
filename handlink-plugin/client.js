/**
 * handlink-terminal · 浏览器半
 *
 * 在 DSH 主面板里画一个「机械手控制终端」：左边是手的实时姿态，右边是逐指控制、
 * 预设点位、动作、录制、自然语言技能。所有请求都打同源的 /handlink/api/*，
 * 由宿主半转给本机 python 服务 —— 页面里没有跨域，也没有第二套状态。
 *
 * 约定：
 *   - id 必须等于 package.json 的 name（handlink-terminal），否则模块表对不上。
 *   - 只用 ctx.slots，不碰 DSH 的 DOM、样式表和状态机。
 *   - 颜色一律走主题变量，写死会跟亮/暗两套皮肤打架。
 */
window.__ModuleLoader__.load({
  id: "@club/handlink-terminal",
  factory: function (require) {
    var module = { exports: {} };
    var exports = module.exports;
    var React = require("react");

    var inject = ["slots"];

    function h(tag, props) {
      var args = [tag, props];
      for (var i = 2; i < arguments.length; i++) args.push(arguments[i]);
      return React.createElement.apply(null, args);
    }

    var FONT = "-apple-system,BlinkMacSystemFont,'PingFang SC','Microsoft YaHei',sans-serif";
    var MONO = "ui-monospace,SFMono-Regular,Consolas,'Liberation Mono',monospace";

    var CSS = [
      // 页面自己滚：DSH 主面板是固定高度、overflow 由宿主决定，卡片一多（尤其加了手套卡）
      // 就会顶到下面出不来。这里同时兼顾「父级是定高块」和「父级是 flex 列」两种情况。
      ".hl-page{flex:1 1 auto;height:100%;max-height:100vh;min-height:0;box-sizing:border-box;overflow-y:auto;overflow-x:hidden;overscroll-behavior:contain;padding:22px 24px 56px;color:var(--dsw-alias-label-primary,#14161a);font-family:" + FONT + ";-webkit-font-smoothing:antialiased;line-height:1.5}",
      ".hl-inner{max-width:1080px;margin:0 auto}",
      ".hl-head{display:flex;flex-wrap:wrap;align-items:flex-end;gap:12px;margin-bottom:16px}",
      ".hl-title{font-size:16px;font-weight:600;margin:0;letter-spacing:.01em}",
      ".hl-sub{font-size:12px;color:var(--dsw-alias-label-secondary,#5c6270);margin:2px 0 0}",
      ".hl-head-right{margin-left:auto;display:flex;flex-wrap:wrap;align-items:center;gap:8px}",
      ".hl-badge{display:inline-flex;align-items:center;gap:5px;font-size:11.5px;padding:2px 9px;border-radius:999px;border:1px solid var(--dsw-alias-border-l1,#e4e7ee);color:var(--dsw-alias-label-secondary,#5c6270);white-space:nowrap}",
      ".hl-badge.ok{color:var(--dsw-alias-state-success-primary,#0f9d6a);border-color:var(--dsw-alias-state-success-primary,#0f9d6a)}",
      ".hl-badge.err{color:var(--dsw-alias-state-error-primary,#d9363e);border-color:var(--dsw-alias-state-error-primary,#d9363e)}",
      ".hl-badge.warn{color:var(--dsw-alias-state-warn-primary,#b7791f);border-color:var(--dsw-alias-state-warn-primary,#b7791f)}",
      ".hl-dot{width:6px;height:6px;border-radius:50%;background:currentColor}",
      ".hl-btn{font-family:inherit;font-size:12.5px;line-height:1.4;padding:5px 11px;border-radius:8px;cursor:pointer;color:var(--dsw-alias-label-primary,#14161a);background:transparent;border:1px solid var(--dsw-alias-border-l2,#d3d8e2);transition:background .12s ease}",
      ".hl-btn:hover{background:var(--dsw-alias-bg-layer-2,#f1f3f7)}",
      ".hl-btn:disabled{opacity:.45;cursor:not-allowed}",
      ".hl-btn.primary{background:var(--dsw-alias-brand-primary,#4d6bfe);border-color:var(--dsw-alias-brand-primary,#4d6bfe);color:#fff}",
      ".hl-btn.primary:hover{filter:brightness(1.06)}",
      ".hl-btn.ghost{border-color:transparent;color:var(--dsw-alias-label-secondary,#5c6270);padding:5px 7px}",
      ".hl-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:14px;align-items:start}",
      ".hl-col{display:flex;flex-direction:column;gap:14px;min-width:0}",
      ".hl-card{background:var(--dsw-alias-bg-layer-1,#f7f8fa);border:1px solid var(--dsw-alias-border-l1,#e4e7ee);border-radius:12px;padding:13px 15px}",
      ".hl-card h2{font-size:12px;font-weight:600;color:var(--dsw-alias-label-secondary,#5c6270);margin:0 0 10px;letter-spacing:.02em}",
      ".hl-row{display:flex;flex-wrap:wrap;gap:7px;align-items:center}",
      ".hl-muted{font-size:11.5px;color:var(--dsw-alias-label-secondary,#5c6270)}",
      ".hl-frow{display:grid;grid-template-columns:58px minmax(80px,1fr) 54px 92px;gap:9px;align-items:center;padding:5px 0;border-top:1px solid var(--dsw-alias-border-l1,#e4e7ee)}",
      ".hl-frow:first-child{border-top:none}",
      ".hl-fname{font-size:12.5px}",
      ".hl-fname span{display:block;font-size:10.5px;color:var(--dsw-alias-label-secondary,#5c6270);font-family:" + MONO + "}",
      ".hl-slider{width:100%;accent-color:var(--dsw-alias-brand-primary,#4d6bfe);height:16px}",
      ".hl-angle{font-size:12px;font-family:" + MONO + ";text-align:right}",
      ".hl-cur{display:flex;flex-direction:column;gap:3px}",
      ".hl-bar{height:5px;background:var(--dsw-alias-bg-layer-2,#e8ebf1);border-radius:999px;overflow:hidden}",
      ".hl-bar i{display:block;height:100%;border-radius:999px;background:var(--dsw-alias-brand-primary,#4d6bfe);transition:width .15s linear}",
      ".hl-amp{font-size:10.5px;font-family:" + MONO + ";color:var(--dsw-alias-label-secondary,#5c6270);text-align:right}",
      ".hl-table{width:100%;border-collapse:collapse;font-size:11.5px}",
      ".hl-table th{text-align:right;font-weight:500;color:var(--dsw-alias-label-secondary,#5c6270);padding:3px 5px;border-bottom:1px solid var(--dsw-alias-border-l1,#e4e7ee)}",
      ".hl-table th:first-child,.hl-table td:first-child{text-align:left}",
      ".hl-table td{text-align:right;padding:3px 5px;font-family:" + MONO + ";border-bottom:1px solid var(--dsw-alias-border-l1,#e4e7ee)}",
      ".hl-input{flex:1 1 200px;min-width:0;font-family:inherit;font-size:12.5px;padding:6px 9px;border-radius:8px;border:1px solid var(--dsw-alias-border-l2,#d3d8e2);background:var(--dsw-alias-bg-base,#fff);color:var(--dsw-alias-label-primary,#14161a);outline:none}",
      ".hl-input:focus{border-color:var(--dsw-alias-brand-primary,#4d6bfe)}",
      ".hl-log{margin:0;padding:8px 10px;border-radius:8px;background:var(--dsw-alias-bg-base,#fff);border:1px solid var(--dsw-alias-border-l1,#e4e7ee);max-height:190px;overflow:auto;font-family:" + MONO + ";font-size:11px;line-height:1.55;color:var(--dsw-alias-label-secondary,#5c6270);white-space:pre-wrap;word-break:break-word}",
      ".hl-log b{color:var(--dsw-alias-state-success-primary,#0f9d6a);font-weight:500}",
      ".hl-log i{color:var(--dsw-alias-state-error-primary,#d9363e);font-style:normal}",
      ".hl-log .w{color:var(--dsw-alias-state-warn-primary,#b7791f)}",
      ".hl-alert{font-size:12px;line-height:1.6;color:var(--dsw-alias-state-error-primary,#d9363e);background:var(--dsw-alias-bg-layer-2,#f1f3f7);border-radius:8px;padding:9px 11px;margin:0 0 10px;word-break:break-word}",
      ".hl-list{display:flex;flex-wrap:wrap;gap:6px}",
      ".hl-kv{display:flex;flex-wrap:wrap;gap:4px 16px;font-size:11.5px;color:var(--dsw-alias-label-secondary,#5c6270);margin-top:8px}",
      ".hl-kv b{font-weight:500;color:var(--dsw-alias-label-primary,#14161a);font-family:" + MONO + "}",
      ".hl-hand{display:block;margin:0 auto;max-width:290px;width:100%}",
      ".hl-empty{font-size:12px;color:var(--dsw-alias-label-secondary,#5c6270);padding:10px 0}",
      ".hl-details summary{cursor:pointer;font-size:12px;font-weight:600;color:var(--dsw-alias-label-secondary,#5c6270);outline:none}",
      ".hl-details[open] summary{margin-bottom:9px}",
      ".hl-notes{margin:8px 0 0;padding-left:18px;font-size:11.5px;line-height:1.65;color:var(--dsw-alias-label-secondary,#5c6270)}",
      ".hl-diag{margin-bottom:14px}",
    ].join("\n");

    // 六指 / 六个主动自由度，角度表与厂家点位一致。
    // 画法：弯曲 = 手指变短 + 指节往下折（不是整根甩向侧面，那样中间态会认错成别的手势）。
    var MOTORS = [
      { id: 102, name: "拇指1", max: 100, x: 74, y: 228, len1: 30, len2: 20, tilt: -54, thumb: true },
      { id: 107, name: "拇指2", max: 45, hidden: true },
      { id: 103, name: "食指", max: 180, x: 93, y: 174, len1: 44, len2: 30 },
      { id: 104, name: "中指", max: 180, x: 115, y: 170, len1: 52, len2: 34 },
      { id: 105, name: "无名指", max: 180, x: 137, y: 174, len1: 46, len2: 30 },
      { id: 106, name: "小指", max: 180, x: 156, y: 178, len1: 36, len2: 24 },
    ];

    var POINT_LABELS = { rock: "石头", paper: "布", scissors: "剪刀" };

    var POLL_STATUS = 2000;
    var POLL_STATE = 250;
    var POLL_HAND = 1200;
    var POLL_GLOVE = 700;

    function post(path, body) {
      return fetch(path, {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify(body || {}),
      }).then(function (r) {
        return r.json().then(function (j) { return { status: r.status, j: j || {} }; });
      });
    }

    function callPy(method, path, body) {
      return post("/handlink/api/py", { method: method, path: path, body: body }).then(function (o) {
        return { ok: o.status < 400 && o.j.ok !== false, status: o.status, j: o.j };
      });
    }

    function short(obj) {
      try {
        var s = JSON.stringify(obj);
        return s && s.length > 110 ? s.slice(0, 110) + "…" : s;
      } catch (e) { return ""; }
    }

    function problemOf(res) {
      var j = res && res.j;
      if (!j) return "HTTP " + (res && res.status);
      if (j.issues && j.issues[0] && j.issues[0].message) return j.issues[0].message;
      return j.error || ("HTTP " + res.status);
    }

    function Badge(props) {
      return h("span", { className: "hl-badge " + (props.kind || "") },
        props.dot ? h("i", { className: "hl-dot" }) : null,
        props.children);
    }

    function Card(props) {
      return h("div", { className: "hl-card " + (props.className || "") },
        props.title ? h("h2", null, props.title) : null,
        props.children);
    }

    // ---------------------------------------------------------------- 手
    //
    // 弯曲的表达方式很关键：整根手指旋转会把"半弯"画成手指横甩，剪刀看起来就像竖中指。
    // 这里用「近节变短 + 远节往下折」来表达弯曲，中间态始终是"手指在收"，不会认错。

    function handGeometry(m, angle) {
      var flex = Math.min(1, Math.max(0, (angle || 0) / m.max));
      var l1 = Math.max(8, m.len1 * (1 - 0.55 * flex));
      var l2 = Math.max(7, m.len2 * (1 - 0.35 * flex));
      var tilt = m.thumb
        ? (m.tilt || 0) + flex * 34          // 拇指：向外张开 → 收向掌心
        : flex * 7;                          // 其余四指：只轻微外张
      var bend = flex * (m.thumb ? 150 : 165); // 远节往掌心折回去；接近 180° 才是"收起来"而不是"甩到一边"
      return { flex: flex, l1: l1, l2: l2, tilt: tilt, bend: bend };
    }

    function HandView(props) {
      var motors = (props.snap && props.snap.motors) || {};
      var fingers = MOTORS.filter(function (m) { return !m.hidden && !m.thumb; });
      var thumb = MOTORS.filter(function (m) { return m.thumb; })[0];

      function drawFinger(m, index) {
        var st = motors[String(m.id)] || { angle: 0 };
        var g = handGeometry(m, st.angle);
        return h("g", { key: m.id, transform: "translate(" + m.x + "," + m.y + ") rotate(" + g.tilt + ")" },
          h("rect", {
            x: -9, y: -g.l1, width: 18, height: g.l1, rx: 9,
            fill: "var(--dsw-alias-border-l1,#e4e7ee)", stroke: "var(--dsw-alias-border-l2,#d3d8e2)",
          }),
          h("g", { transform: "translate(0," + (-g.l1) + ") rotate(" + g.bend + ")" },
            h("rect", {
              x: -8, y: -g.l2, width: 16, height: g.l2, rx: 8,
              fill: "var(--dsw-alias-bg-layer-2,#e8ebf1)", stroke: "var(--dsw-alias-border-l2,#d3d8e2)",
            })));
      }

      return h("svg", { viewBox: "0 0 260 320", className: "hl-hand", role: "img", "aria-label": "手部姿态" },
        h("rect", {
          x: 76, y: 170, width: 106, height: 100, rx: 26,
          fill: "var(--dsw-alias-bg-layer-2,#e8ebf1)", stroke: "var(--dsw-alias-border-l2,#d3d8e2)",
        }),
        h("rect", {
          x: 100, y: 264, width: 58, height: 16, rx: 8,
          fill: "var(--dsw-alias-bg-layer-2,#e8ebf1)", stroke: "var(--dsw-alias-border-l2,#d3d8e2)",
        }),
        fingers.map(drawFinger),
        thumb ? drawFinger(thumb) : null);
    }

    // ---------------------------------------------------------------- 页面

    function TerminalPage() {
      var [status, setStatus] = React.useState(null);
      var [snap, setSnap] = React.useState(null);
      var [hand, setHand] = React.useState(null);
      var [draft, setDraft] = React.useState({});
      var [busy, setBusy] = React.useState("");
      var [actions, setActions] = React.useState([]);
      var [points, setPoints] = React.useState([]);
      var [tracks, setTracks] = React.useState([]);
      var [stats, setStats] = React.useState(null);
      var [recording, setRecording] = React.useState(false);
      var [aiText, setAiText] = React.useState("");
      var [aiPlan, setAiPlan] = React.useState(null);
      var [log, setLog] = React.useState([]);
      var [connectError, setConnectError] = React.useState("");
      var [diag, setDiag] = React.useState(null);
      var [diagBusy, setDiagBusy] = React.useState(false);
      var [glove, setGlove] = React.useState(null);
      var [gloveMissing, setGloveMissing] = React.useState(false);
      var [gloveUdp, setGloveUdp] = React.useState(null);
      var [gloveSide, setGloveSide] = React.useState(null);
      var [gloveBusy, setGloveBusy] = React.useState("");

      var phase = (status && status.phase) || "idle";
      var ready = phase === "ready";
      var port = status && status.port;

      function push(text, kind) {
        setLog(function (prev) {
          var t = new Date().toLocaleTimeString();
          return [{ t: t, kind: kind || "", text: text }].concat(prev).slice(0, 60);
        });
      }

      React.useEffect(function () {
        var alive = true;
        function tick() {
          fetch("/handlink/api/status", { cache: "no-store" })
            .then(function (r) { return r.json(); })
            .then(function (s) { if (alive) setStatus(s); })
            .catch(function () { if (alive) setStatus({ phase: "error", error: "连不上 DSH 宿主（页面被刷掉了？）" }); });
        }
        tick();
        var id = setInterval(tick, POLL_STATUS);
        return function () { alive = false; clearInterval(id); };
      }, []);

      React.useEffect(function () {
        if (!ready) { setGlove(null); return; }
        var alive = true;
        function gloveTick() {
          callPy("GET", "/glove/state").then(function (res) {
            if (!alive) return;
            if (res.status === 404) {
              setGloveMissing(true);
              setGlove(null);
              return;
            }
            setGloveMissing(false);
            if (res.ok) setGlove(res.j);
          }).catch(function () {});
        }
        gloveTick();
        var gid = setInterval(gloveTick, POLL_GLOVE);
        return function () { alive = false; clearInterval(gid); };
      }, [ready, port]);

      React.useEffect(function () {
        if (!ready) { setSnap(null); setHand(null); return; }        var alive = true;
        function stateTick() {
          callPy("GET", "/state").then(function (res) { if (alive && res.ok) setSnap(res.j); }).catch(function () {});
        }
        function handTick() {
          callPy("GET", "/hand/status").then(function (res) { if (alive) setHand(res.j || null); }).catch(function () {});
        }
        stateTick();
        handTick();
        var a = setInterval(stateTick, POLL_STATE);
        var b = setInterval(handTick, POLL_HAND);
        return function () { alive = false; clearInterval(a); clearInterval(b); };
      }, [ready, port]);

      React.useEffect(function () {
        if (!ready) return;
        var alive = true;
        callPy("GET", "/points").then(function (res) { if (alive && res.j && res.j.points) setPoints(res.j.points); }).catch(function () {});
        callPy("GET", "/tracks").then(function (res) { if (alive && res.j && res.j.tracks) setTracks(res.j.tracks); }).catch(function () {});
        callPy("GET", "/actions").then(function (res) { if (alive && res.j && res.j.actions) setActions(res.j.actions); }).catch(function () {});
        return function () { alive = false; };
      }, [ready, port]);

      function act(label, method, path, body) {
        setBusy(label);
        return callPy(method, path, body)
          .then(function (res) {
            setBusy("");
            if (res.ok) push("✓ " + label + "  " + short(res.j), "ok");
            else push("✗ " + label + "  " + problemOf(res), "err");
            return res;
          })
          .catch(function (e) {
            setBusy("");
            push("✗ " + label + "  " + (e && e.message ? e.message : e), "err");
            return { ok: false, status: 0, j: { error: String(e) } };
          });
      }

      function control(action) {
        setBusy(action);
        post("/handlink/api/control", { action: action })
          .then(function (o) { return o.j; })
          .then(function (j) {
            setBusy("");
            push("· " + action + "  " + (j && j.message ? j.message : (j && j.ok ? "已提交" : (j && j.error) || "")), "");
          })
          .catch(function (e) { setBusy(""); push("✗ " + action + "  " + e, "err"); });
      }

      function commit(motor, value) {
        setDraft(function (d) {
          var n = {};
          for (var k in d) if (Object.prototype.hasOwnProperty.call(d, k) && k !== String(motor.id)) n[k] = d[k];
          return n;
        });
        var motors = {};
        motors[motor.id] = { angle: value, speed: 60, torque: 0.3 };
        act("逐指 " + motor.name + " → " + value + "°", "POST", "/pose", { motors: motors, reason: "panel:slider" });
      }

      function ai(run) {
        var text = (aiText || "").trim();
        if (!text) return;
        act(run ? "执行：" + text : "只看计划：" + text, "POST", "/ai", { text: text, dry_run: !run })
          .then(function (res) {
            if (res && res.j && (res.j.plan || res.j.explain)) setAiPlan(res.j);
          });
      }

      function runDiag() {
        setDiagBusy(true);
        post("/handlink/api/diag", {})
          .then(function (o) {
            setDiagBusy(false);
            if (o.status === 404 || o.status === 405) {
              push("环境自检接口要重启 DSH 后才生效（当前宿主还是旧版）", "");
              return null;
            }
            setDiag(o.j || null);
            push(o.j && o.j.ok ? "✓ 环境自检完成" : ("✗ 环境自检失败 " + ((o.j && o.j.error) || "")), o.j && o.j.ok ? "ok" : "err");
            return null;
          })
          .catch(function (e) { setDiagBusy(false); push("✗ 环境自检失败 " + e, "err"); });
      }

      function connectHand() {
        setConnectError("");
        var body = { bus: "can" };
        // 手走 ZLG USBCAN：把包内自带的驱动绝对路径带上，别让 Python 去猜
        if (status && status.dataRoot) {
          body.dll = String(status.dataRoot).replace(/\\/g, "/") + "/src/main/resources/native/Windows-x86_64/ECanVci64.dll";
        }
        act("连接机械手", "POST", "/hand/attach", body).then(function (res) {
          if (!res || !res.ok) {
            setConnectError(problemOf(res));
            runDiag();
          }
        });
      }

      var connected = !!(hand && hand.connected);
      var busName = (hand && hand.bus) || (snap && snap.bus) || (status && status.health && status.health.bus) || "?";
      var busLabel = busName === "can" ? "真机 CAN" : "仿真总线";

      function serviceBadge() {
        if (phase === "ready") return h(Badge, { kind: "ok", dot: true }, "服务已就绪 · 127.0.0.1:" + port);
        if (phase === "starting") return h(Badge, { kind: "warn", dot: true }, "正在启动服务…");
        if (phase === "stopped") return h(Badge, { dot: true }, "服务已停止");
        if (phase === "idle") return h(Badge, null, "等待激活");
        return h(Badge, { kind: "err", dot: true }, "服务未就绪");
      }

      function gloveLabel() {
        if (gloveMissing) return "手套接口未加载";
        if (!glove || !glove.running) return "数据手套 · 未启动";
        if (glove.mode === "code") return "代码模式 · 手套已隔断";
        return glove.live ? "手套操控 · 跟随中" : "手套操控 · 等数据";
      }

      function toggleGlove() {
        if (!glove || !glove.running) {
          push("数据手套还没启动：在「数据手套」卡片里点「启动监听」", "err");
          return;
        }
        gloveCall("切换手套模式", "/glove/toggle", {});
      }

      function gloveCall(label, path, body) {
        setGloveBusy(label);
        return callPy("POST", path, body || {}).then(function (res) {
          setGloveBusy("");
          if (res.ok) {
            setGlove(function (g) { return Object.assign({}, g || {}, res.j); });
            push("✓ " + label + "  " + (res.j && res.j.mode ? "模式 " + res.j.mode : short(res.j)), "ok");
          } else {
            push("✗ " + label + "  " + problemOf(res), "err");
          }
          return res;
        }).catch(function (e) {
          setGloveBusy("");
          push("✗ " + label + "  " + (e && e.message ? e.message : e), "err");
          return { ok: false, status: 0, j: {} };
        });
      }

      var header = h("header", { className: "hl-head" },
        h("div", null,
          h("h1", { className: "hl-title" }, "机械手控制终端"),
          h("p", { className: "hl-sub" }, "中科灵犀 LG-603 · 六指 6 主动自由度 · 点位 / 动作 / 录制 / 技能都走同一套接口")),
        h("div", { className: "hl-head-right" },
          serviceBadge(),
          ready ? h(Badge, { kind: connected ? "ok" : "", dot: true }, busLabel + (connected ? " · 已连接" : " · 未接真手")) : null,
          h("button", {
            className: "hl-btn" + (glove && glove.running && glove.mode === "glove" ? " primary" : ""),
            onClick: toggleGlove,
            title: "在数据手套跟手与代码模式之间切换（隔断/恢复手套下发）",
          }, gloveLabel()),
          ready ? h("button", { className: "hl-btn primary", disabled: !!busy, onClick: connectHand }, "连接机械手") : null,
          ready ? h("button", { className: "hl-btn", disabled: diagBusy, onClick: runDiag }, diagBusy ? "自检中…" : "环境自检") : null,
          ready ? h("button", { className: "hl-btn", disabled: !!busy, onClick: function () { act("停止（归零）", "POST", "/stop", {}); } }, "停止") : null,
          h("button", { className: "hl-btn", disabled: busy === "restart", onClick: function () { control("restart"); } }, "重启服务"),
          ready && status && status.origin ? h("a", { className: "hl-btn", href: status.origin + "/demo", target: "_blank", rel: "noreferrer" }, "完整页面 ↗") : null));

      function bootPanel() {
        var s = status || {};
        var deps = s.deps || {};
        var missing = Object.keys(deps).filter(function (k) { return deps[k] === false; });
        var installing = s.install && s.install.running;
        return h(Card, { title: phase === "starting" ? "正在启动本地服务" : "本地服务没有跑起来" },
          s.error ? h("p", { className: "hl-alert" }, s.error) : (phase === "starting" ? h("p", { className: "hl-muted" }, "正在拉起 python -m handlink serve，第一次启动要一两秒…") : null),
          missing.length ? h("p", { className: "hl-muted" }, "缺少可选依赖：" + missing.join("、") + "（摄像头/视觉/串口才需要，核心控制不受影响）") : null,
          h("div", { className: "hl-row" },
            h("button", { className: "hl-btn primary", disabled: installing, onClick: function () { control("restart"); } }, "重新启动"),
            h("button", { className: "hl-btn", disabled: installing, onClick: function () { control("install"); } }, installing ? "安装中…" : "安装依赖"),
            h("button", { className: "hl-btn ghost", disabled: installing, onClick: function () { control("stop"); } }, "停止服务")),
          h("div", { className: "hl-kv" },
            h("span", null, "Python ", h("b", null, s.python || "未找到")),
            h("span", null, "源码 ", h("b", null, s.projectDir || "-")),
            h("span", null, "数据 ", h("b", null, s.dataRoot || "-"))),
          (s.install && s.install.log && s.install.log.length)
            ? h("pre", { className: "hl-log", style: { marginTop: "10px" } }, s.install.log.slice(-12).join("\n"))
            : null,
          h("pre", { className: "hl-log", style: { marginTop: "10px" } },
            (s.log && s.log.length) ? s.log.slice(-25).map(function (l) { return l.text; }).join("\n") : "还没有日志"));
      }

      function setGloveGain(delta) {
        var cur = (glove && glove.gain) || 1.0;
        var next = Math.max(0.3, Math.min(2.0, Math.round((cur + delta) * 10) / 10));
        if (next === cur) return;
        gloveCall("增益 → " + next, "/glove/gain", { gain: next });
      }

      function gloveCard() {
        var g = glove || null;
        var running = !!(g && g.running);
        var live = !!(g && g.live);
        var muted = !!(g && g.mode === "code");
        var cfg = (status && status.glove) || {};
        var udpShown = gloveUdp !== null ? gloveUdp : (running ? g.udp : (cfg.udp || 5001));
        var sideShown = gloveSide !== null ? gloveSide : (running ? g.side : (cfg.side || "right"));
        var motors = (g && g.motors) || {};
        var raw = (g && g.raw) || {};

        function stateBadge() {
          if (gloveMissing) return h(Badge, { kind: "warn", dot: true }, "接口未加载");
          if (!g) return h(Badge, null, "等待状态…");
          if (!running) return h(Badge, null, "未启动");
          if (muted) return h(Badge, { kind: "warn", dot: true }, "代码模式 · 已隔断");
          if (live) return h(Badge, { kind: "ok", dot: true }, "跟随中 · " + (g.fps || 0) + " Hz");
          return h(Badge, { kind: "warn", dot: true }, "监听中 · 无数据");
        }

        return h(Card, { title: "数据手套（FOHEART HMax）" },
          h("div", { className: "hl-row" },
            stateBadge(),
            running ? h("span", { className: "hl-muted" }, "帧 " + g.frames + " · 已下发 " + g.applied) : null,
            running && g.frame_age_s !== null && g.frame_age_s !== undefined
              ? h("span", { className: "hl-muted" }, "最近 " + g.frame_age_s + " s") : null),
          (function () {
            var br = (status && status.bridge) || {};
            var text;
            if (!br.wanted) text = "数据源：MotionVenus 转发（UDP）";
            else if (br.running && br.sn) text = "数据源：直连 SDK · SN 0x" + br.sn;
            else if (br.running) text = "数据源：直连 SDK · 等接收器（MotionVenus 还开着？）";
            else if (br.error) text = "数据源：MotionVenus 转发（直连不可用：" + br.error + "）";
            else text = "数据源：MotionVenus 转发（UDP）";
            return h("div", { className: "hl-row", style: { marginTop: "6px" } },
              h("span", { className: "hl-muted", title: (br.log || []).join("\n") }, text));
          })(),
          h("div", { className: "hl-row", style: { marginTop: "9px" } },
            h("span", { className: "hl-muted" }, "UDP"),
            h("input", {
              className: "hl-input", style: { flex: "0 0 82px" }, type: "number", min: 1, max: 65535,
              value: udpShown, disabled: running || !ready,
              onChange: function (e) { setGloveUdp(Number(e.target.value) || 0); },
            }),
            h("select", {
              className: "hl-input", style: { flex: "0 0 92px" }, value: sideShown,
              disabled: running || !ready,
              onChange: function (e) { setGloveSide(e.target.value); },
            },
              h("option", { value: "right" }, "右手"),
              h("option", { value: "left" }, "左手")),
            running
              ? h("button", {
                className: "hl-btn", disabled: !!gloveBusy,
                onClick: function () { gloveCall("停止手套监听", "/glove/stop", {}); },
              }, "停止监听")
              : h("button", {
                className: "hl-btn primary", disabled: !ready || !!gloveBusy,
                onClick: function () {
                  gloveCall("启动手套监听", "/glove/start",
                    { udp: udpShown, side: sideShown, gain: (cfg.gain || 1) });
                },
              }, "启动监听")),
          running ? h("div", { className: "hl-row", style: { marginTop: "8px" } },
            h("button", {
              className: "hl-btn" + (muted ? "" : " primary"), disabled: !!gloveBusy,
              onClick: toggleGlove,
            }, muted ? "恢复手套操控" : "切到代码模式"),
            h("button", {
              className: "hl-btn", disabled: !!gloveBusy,
              onClick: function () { gloveCall("重新零点", "/glove/zero", {}); },
            }, "重新零点"),
            h("span", { className: "hl-muted", style: { marginLeft: "6px" } }, "增益"),
            h("button", {
              className: "hl-btn ghost", disabled: !!gloveBusy,
              onClick: function () { setGloveGain(-0.1); },
            }, "－"),
            h("b", { style: { fontSize: "12px", fontFamily: MONO } },
              (g && typeof g.gain === "number" ? g.gain.toFixed(1) : "1.0")),
            h("button", {
              className: "hl-btn ghost", disabled: !!gloveBusy,
              onClick: function () { setGloveGain(0.1); },
            }, "＋")) : null,
          h("div", { style: { marginTop: "9px" } },
            MOTORS.map(function (m) {
              var v = motors[String(m.id)];
              var angle = typeof v === "number" ? v : 0;
              var r = raw[String(m.id)];
              return h("div", { className: "hl-frow", key: "g-" + m.id },
                h("div", { className: "hl-fname" }, m.name, h("span", null, m.id)),
                h("div", { className: "hl-bar", style: { height: "7px" } },
                  h("i", { style: { width: Math.max(0, Math.min(100, 100 * angle / m.max)) + "%" } })),
                h("div", { className: "hl-angle" }, typeof v === "number" ? Math.round(angle) + "°" : "-"),
                h("div", { className: "hl-amp" }, r === undefined ? "" : "原始 " + Math.round(r) + "°"));
            })),
          (g && g.hint) ? h("p", {
            className: g.start_error ? "hl-alert" : "hl-muted",
            style: { marginTop: "8px" },
          }, g.hint) : null,
          gloveMissing ? h("p", { className: "hl-alert", style: { marginTop: "8px" } },
            "服务里还没有手套接口：包内 handlink 是旧版，先点「重启服务」；还是这样就说明装的是旧插件。") : null,
          h("p", { className: "hl-muted", style: { marginTop: "6px" } },
            "默认走插件自带的直连小桥（加载厂家 h2sdk），不用开 MotionVenus；小桥起不来才退回 MotionVenus 的 UDP 转发。跟手时滑块/点位会被下一帧覆盖，要独占手就切「代码模式」。"));
      }

      function handCard() {
        return h(Card, { title: "手部状态（实时）" },
          h(HandView, { snap: snap }),
          h("div", { className: "hl-kv" },
            h("span", null, "电机 ", h("b", null, 6)),
            h("span", null, "已运行 ", h("b", null, (status && status.health && status.health.uptime_s) || 0), " s"),
            hand && hand.detail ? h("span", null, hand.detail) : null));
      }

      function fingerCard() {
        var motors = (snap && snap.motors) || {};
        return h(Card, { title: "逐指控制（松手即下发 /pose）" },
          MOTORS.map(function (m) {
            var st = motors[String(m.id)] || null;
            var shown = draft[m.id] !== undefined ? draft[m.id] : (st ? Math.round(st.angle) : 0);
            var ratio = st && st.current_limit ? Math.min(100, (100 * st.current) / st.current_limit) : 0;
            return h("div", { className: "hl-frow", key: m.id },
              h("div", { className: "hl-fname" }, m.name, h("span", null, m.id)),
              h("input", {
                className: "hl-slider",
                type: "range",
                min: 0,
                max: m.max,
                step: 1,
                value: shown,
                disabled: !ready,
                onChange: function (e) {
                  var v = Number(e.target.value);
                  setDraft(function (d) { var n = Object.assign({}, d); n[m.id] = v; return n; });
                },
                onPointerUp: function () { commit(m, shown); },
                onKeyUp: function () { commit(m, shown); },
              }),
              h("div", { className: "hl-angle" }, shown + "°"),
              h("div", { className: "hl-cur" },
                h("div", { className: "hl-bar" }, h("i", { style: { width: ratio + "%" } })),
                h("div", { className: "hl-amp" }, st ? st.current.toFixed(2) + "A" : "-")));
          }),
          h("p", { className: "hl-muted", style: { marginTop: "8px" } }, "只改一根手指时，其余电机保持原位；速度固定 60、电流上限 0.3A。"),
          (glove && glove.running && glove.mode === "glove")
            ? h("p", { className: "hl-muted" }, "手套正在跟手：这里的指令下发后会被下一帧手套数据覆盖。")
            : null);
      }

      function poseCard() {
        return h(Card, { title: "预设点位 / 动作" },
          points.length
            ? h("div", { className: "hl-list" }, points.map(function (name) {
              return h("button", {
                key: "p-" + name,
                className: "hl-btn",
                disabled: !!busy,
                onClick: function () { act("点位 " + name, "POST", "/point", { name: name }); },
              }, POINT_LABELS[name] || name);
            }))
            : h("div", { className: "hl-empty" }, "数据目录里还没有点位"),
          tracks.length ? h("div", { className: "hl-list", style: { marginTop: "9px" } },
            tracks.map(function (name) {
              return h("button", {
                key: "t-" + name,
                className: "hl-btn primary",
                disabled: !!busy,
                onClick: function () { act("播放 " + name, "POST", "/track", { name: name }); },
              }, "播放 " + name);
            })) : null,
          actions.length ? h("div", { className: "hl-list", style: { marginTop: "9px" } },
            actions.map(function (a) {
              return h("button", {
                key: "a-" + a.name,
                className: "hl-btn",
                title: (a.poses || []).join(" → "),
                disabled: !!busy,
                onClick: function () { act("动作 " + (a.label || a.name), "POST", "/action", { name: a.name }); },
              }, a.label || a.name);
            })) : null);
      }

      function recordCard() {
        var s = stats && stats.motors ? stats : null;
        return h(Card, { title: "录制与分析" },
          h("div", { className: "hl-row" },
            h("button", {
              className: "hl-btn primary",
              disabled: !!busy || recording,
              onClick: function () {
                act("开始录制", "POST", "/record/start", { hz: 20 }).then(function (res) { if (res.ok) setRecording(true); });
              },
            }, recording ? "录制中…" : "开始录制"),
            h("button", {
              className: "hl-btn",
              disabled: !!busy || !recording,
              onClick: function () {
                act("停止录制", "POST", "/record/stop", {}).then(function (res) {
                  setRecording(false);
                  if (res.ok && res.j && res.j.stats) setStats(res.j.stats);
                });
              },
            }, "停止并分析")),
          s ? h("div", { style: { marginTop: "10px" } },
            h("div", { className: "hl-muted" }, (s.samples || 0) + " 帧 / " + (s.duration_s || 0) + " 秒"),
            h("table", { className: "hl-table" },
              h("thead", null, h("tr", null,
                ["手指", "行程", "峰速", "峰值电流", "占用"].map(function (t) { return h("th", { key: t }, t); }))),
              h("tbody", null, Object.keys(s.motors).map(function (id) {
                var m = s.motors[id];
                return h("tr", { key: id },
                  h("td", null, m.name),
                  h("td", null, m.rom + "°"),
                  h("td", null, m.speed_max),
                  h("td", null, m.current_max + "A"),
                  h("td", null, Math.round((m.current_peak_ratio || 0) * 100) + "%"));
              })))) : h("p", { className: "hl-muted", style: { marginTop: "9px" } }, "停止录制后这里出行程、峰值电流和占用率。"),
          h("p", { className: "hl-muted", style: { marginTop: "6px" } }, "录制文件写在数据目录的 recordings/ 下，可用 python -m handlink analyze 离线复算。"));
      }

      function aiCard() {
        return h(Card, { title: "AI 控制（自然语言 → 技能）" },
          h("div", { className: "hl-row" },
            h("input", {
              className: "hl-input",
              value: aiText,
              placeholder: "先张开，再握拳，然后食指指一下",
              onChange: function (e) { setAiText(e.target.value); },
              onKeyDown: function (e) { if (e.key === "Enter") ai(true); },
            }),
            h("button", { className: "hl-btn primary", disabled: !!busy, onClick: function () { ai(true); } }, "执行"),
            h("button", { className: "hl-btn", disabled: !!busy, onClick: function () { ai(false); } }, "只看计划")),
          aiPlan ? h("p", { className: "hl-muted", style: { marginTop: "8px" } },
            (aiPlan.plan && aiPlan.plan.source ? "来源 " + aiPlan.plan.source + " · " : "") + (aiPlan.explain || "")) : null,
          h("p", { className: "hl-muted", style: { marginTop: "6px" } },
            "技能白名单：张开 / 握拳 / 捏 / 剪刀 / 食指指 / 指定角度 / 保持。"));
      }

      function diagCard() {
        if (!connectError && !diag) return null;
        var s = diag && diag.scan;
        var com = s && s.com_ports ? s.com_ports.length : null;
        var can = s && s.can_devices ? s.can_devices : null;
        var cams = s && s.cameras ? s.cameras.filter(function (c) { return c.present; }).length : null;
        var canText = can ? (can.opened ? "已打开" : (can.error || "没有应答")) : "未知";
        return h(Card, { title: "连接诊断", className: "hl-diag" },
          connectError ? h("p", { className: "hl-alert" }, connectError) : null,
          s ? h("div", { className: "hl-kv" },
            h("span", null, "COM 口 ", h("b", null, com)),
            h("span", null, "CAN 适配器 ", h("b", null, canText)),
            h("span", null, "摄像头 ", h("b", null, cams))) : null,
          s && s.can_dll ? h("p", { className: "hl-muted", style: { marginTop: "6px", wordBreak: "break-all" } }, "驱动 DLL ", s.can_dll) : null,
          s && s.notes && s.notes.length
            ? h("ul", { className: "hl-notes" }, s.notes.map(function (n, i) { return h("li", { key: i }, n); }))
            : null,
          connectError ? h("ul", { className: "hl-notes" },
            h("li", null, "盒子插了吗：设备管理器里要出现「USB CAN → GCAN USBCAN Device」（或 ZLG USBCAN）。"),
            h("li", null, "驱动装了吗：包内已带 ECanVci64.dll；盒子自己那套驱动（GCAN 的 USBCAN 驱动）也要装。"),
            h("li", null, "被占用吗：厂家的 LG-603.exe 开着时会独占盒子，先关掉它。"),
            h("li", null, "接线对吗：手要有 12V 供电，CAN_H / CAN_L 别接反，总线两端各 120 Ω（量起来约 60 Ω）。"),
            h("li", null, "换口试试：USBCAN 对 USB3 口 / 延长线比较挑，优先主板后置 USB2 口，别走 Hub。")) : null,
          diag && diag.raw ? h("pre", { className: "hl-log", style: { marginTop: "8px" } }, String(diag.raw).slice(0, 1200)) : null,
          h("div", { className: "hl-row", style: { marginTop: "10px" } },
            h("button", { className: "hl-btn", disabled: diagBusy, onClick: runDiag }, diagBusy ? "自检中…" : "重新自检"),
            h("button", { className: "hl-btn primary", disabled: !!busy, onClick: connectHand }, "重试连接")));
      }

      var workspace = h("div", null,
        h("div", { className: "hl-grid" },
          h("div", { className: "hl-col" },
            handCard(),
            gloveCard(),
            poseCard()),
          h("div", { className: "hl-col" },
            fingerCard(),
            aiCard(),
            recordCard())),
        busy ? h("p", { className: "hl-muted", style: { marginTop: "10px" } }, "· " + busy + " …") : null);

      var logs = h("div", { className: "hl-card hl-details", style: { marginTop: "14px" } },
        h("details", null,
          h("summary", null, "接口日志 / 服务输出"),
          h("div", { className: "hl-grid", style: { marginTop: "10px" } },
            h("pre", { className: "hl-log" },
              log.length ? log.map(function (l, i) {
                var mark = l.kind === "ok" ? "b" : (l.kind === "err" ? "i" : "span");
                return h(mark === "span" ? "div" : mark, { key: i }, l.t + "  " + l.text);
              }) : "等待操作…"),
            h("pre", { className: "hl-log" },
              (status && status.log && status.log.length)
                ? status.log.slice(-40).map(function (l) { return l.t + " [" + l.stream + "] " + l.text; }).join("\n")
                : "服务还没有输出"))));

      // 渲染期真出问题也要看得见：白屏最没法查，宁可把错误原文摆出来
      try {
        return h("div", { className: "hl-page" },
          h("style", null, CSS),
          h("div", { className: "hl-inner" },
            header,
            diagCard(),
            ready ? workspace : bootPanel(),
            logs));
      } catch (err) {
        return h("div", { className: "hl-page" },
          h("style", null, CSS),
          h("div", { className: "hl-inner" },
            h("div", { className: "hl-card" },
              h("h2", null, "终端页面渲染出错"),
              h("p", { className: "hl-alert" }, String((err && err.message) || err)),
              h("pre", { className: "hl-log" }, String((err && err.stack) || "")),
              h("p", { className: "hl-muted" },
                "服务可能本身没问题：打开 http://127.0.0.1:" + (port || "8765") + "/health 看 JSON。" +
                "把这段错误发给我，我照着修。"))));
      }
    }

    // ---------------------------------------------------------------- 侧栏入口

    function PanelIcon(props) {
      var size = props.size || 18;
      return h("svg", {
        width: size, height: size, viewBox: "0 0 96 96", fill: "none",
        "aria-hidden": true, style: { display: "block", opacity: props.active ? 1 : 0.82 },
      },
        h("g", { stroke: "currentColor", strokeWidth: 6.5, strokeLinecap: "round" },
          h("path", { d: "M34 62V34" }),
          h("path", { d: "M44 62V27" }),
          h("path", { d: "M54 62V27" }),
          h("path", { d: "M64 62V34" }),
          h("path", { d: "M28 64c-3-6-6-12-4-16 2-3 6-2 7 2l3 7" }),
          h("path", { d: "M70 64c3-6 6-12 4-16-2-3-6-2-7 2l-3 7" })),
        h("path", {
          d: "M30 64h36a6 6 0 0 1 6 6v8a10 10 0 0 1-10 10H34a10 10 0 0 1-10-10v-8a6 6 0 0 1 6-6z",
          fill: "none", stroke: "currentColor", strokeWidth: 6.5, strokeLinejoin: "round",
        }));
    }

    // 子组件渲染期抛错时的兜底：DSH 会把崩掉的插槽条目置空，那就是一片白
    class ErrorBoundary extends React.Component {
      constructor(props) {
        super(props);
        this.state = { error: null };
      }
      componentDidCatch(error) {
        this.setState({ error: error });
      }
      render() {
        var err = this.state.error;
        if (!err) return this.props.children;
        return h("div", { className: "hl-page" },
          h("style", null, CSS),
          h("div", { className: "hl-inner" },
            h("div", { className: "hl-card" },
              h("h2", null, "终端页面崩溃"),
              h("p", { className: "hl-alert" }, String(err.message || err)),
              h("pre", { className: "hl-log" }, String(err.stack || "")),
              h("p", { className: "hl-muted" }, "把这段错误发给我；服务本身可以用 /health 单独确认。"))));
      }
    }

    function TerminalShell() {
      return h(ErrorBoundary, null, h(TerminalPage));
    }

    function apply(ctx) {
      ctx.effect(function () {
        return ctx.slots.inject("sidebar.panellist", function () {
          return ctx.slots.register(
            { name: "sidebar.panellist", id: "handlink", order: 20, label: "机械手" },
            PanelIcon);
        });
      }, "handlink: 侧栏入口");

      ctx.effect(function () {
        return ctx.slots.inject("main", function () {
          return ctx.slots.register({ name: "main", key: "handlink" }, TerminalShell);
        });
      }, "handlink: 主面板");

      // 备用入口：主面板那条如果被宿主判成「未知面板」，设置里这一页照样能开
      ctx.effect(function () {
        return ctx.slots.inject("settings.section", function () {
          return ctx.slots.register(
            { name: "settings.section", id: "handlink", order: 60, label: "机械手" },
            TerminalShell);
        });
      }, "handlink: 设置页");
    }

    exports.apply = apply;
    exports.inject = inject;
    return module.exports;
  },
});
