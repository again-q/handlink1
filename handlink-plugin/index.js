/**
 * handlink-terminal · 宿主半
 *
 * 干三件事：
 *   1. 找到一台能用的 Python，把包内自带的 handlink 源码以 `-m handlink serve` 跑起来；
 *   2. 用 ctx.webServer 开一组同源接口（/handlink/api/*），浏览器那半直接 fetch，
 *      不跨域、不用 RPC、不用改 handlink 的 Python 代码；
 *   3. 把服务状态（端口、日志、缺哪个依赖）如实报给页面，页面上能重启、能装依赖。
 *
 * 刻意的地方：
 *   - 服务没起来时页面不会白屏：phase=error 时把 stderr 尾部原样给出来。
 *   - 装依赖装进包内 python/site-packages（pip --target），不动系统 Python。
 *   - 任何注册失败都只记日志，绝不让 DSH 起不来。
 */
import { spawn, spawnSync } from 'node:child_process'
import { copyFileSync, cpSync, existsSync, mkdirSync, readFileSync, readdirSync, statSync } from 'node:fs'
import { createServer } from 'node:net'
import { homedir } from 'node:os'
import { delimiter, dirname, join, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = dirname(fileURLToPath(import.meta.url))
const PACKAGE = (() => {
  try { return JSON.parse(readFileSync(join(HERE, 'package.json'), 'utf8')) } catch { return {} }
})()

export const name = '@club/handlink-terminal'
/** webServer：开同源接口。少声明一个，取不到就是初始化失败。 */
export const inject = ['webServer']

const LOG_LIMIT = 240
const READY_TIMEOUT_MS = 30000

let live = null // 当前服务的运行时状态，一个 profile 只留一份

function nowStamp() {
  const d = new Date()
  const p = (n, w = 2) => String(n).padStart(w, '0')
  return `${p(d.getDate())}${p(d.getMonth() + 1)}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`
}

function json(res, status, body) {
  let data
  try { data = Buffer.from(JSON.stringify(body), 'utf8') } catch { data = Buffer.from('{"ok":false,"error":"响应无法序列化"}', 'utf8') }
  try {
    res.writeHead(status, {
      'content-type': 'application/json; charset=utf-8',
      'cache-control': 'no-store',
      'content-length': data.length,
    })
    res.end(data)
  } catch { /* 客户端已断开 */ }
}

function readJson(req, limit = 256 * 1024) {
  return new Promise((resolvePromise, reject) => {
    let size = 0
    const chunks = []
    req.on('data', (c) => {
      size += c.length
      if (size > limit) { reject(new Error('请求体太大')); req.destroy(); return }
      chunks.push(c)
    })
    req.on('end', () => {
      const text = Buffer.concat(chunks).toString('utf8').trim()
      if (!text) return resolvePromise({})
      try { resolvePromise(JSON.parse(text)) } catch (e) { reject(new Error('请求体不是合法 JSON')) }
    })
    req.on('error', reject)
  })
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)) }

// ---------------------------------------------------------------- 路径与解释器

function makeState(config) {
  const projectDir = config.projectDir ? resolve(String(config.projectDir)) : join(HERE, 'python')
  const dataRoot = config.dataRoot ? resolve(String(config.dataRoot)) : join(HERE, 'data')
  return {
    config,
    projectDir,
    dataRoot,
    recordDir: join(dataRoot, 'recordings'),
    sitePackages: join(projectDir, 'site-packages'),
    phase: 'idle',       // idle | starting | ready | error | stopped
    port: null,
    origin: '',
    python: '',
    deps: null,
    error: '',
    log: [],
    pid: null,
    startedAt: null,
    health: null,
    install: { running: false, message: '', log: [] },
    child: null,
    generation: 0,
    // 手套直连小桥（32 位，加载厂家 h2sdk，不走 MotionVenus）
    bridge: { pid: null, child: null, sn: '', error: '', waiting: false, log: [], startedAt: null },
  }
}

function pushLog(st, stream, text) {
  const lines = String(text).split(/\r?\n/)
  for (const line of lines) {
    if (!line) continue
    st.log.push({ t: nowStamp(), stream, text: line.slice(0, 600) })
  }
  if (st.log.length > LOG_LIMIT) st.log.splice(0, st.log.length - LOG_LIMIT)
}

function extraPaths(st) {
  const cfg = st.config || {}
  const out = [st.sitePackages, join(HERE, 'site-packages')]
  const fromEnv = process.env.HANDLINK_PATHS
  if (fromEnv) out.push(...fromEnv.split(delimiter).filter(Boolean))
  for (const p of cfg.extraPaths || []) if (p) out.push(resolve(String(p)))
  // 开发布局：项目根目录旁边就放着解压好的依赖（这台机器上的 .tmp/pylibs 就是）
  out.push(join(HERE, '.tmp', 'pylibs'), join(dirname(HERE), '.tmp', 'pylibs'))
  out.push(join(st.projectDir, '..', '.tmp', 'pylibs'))
  const seen = new Set()
  const kept = []
  for (const p of out) {
    const abs = resolve(p)
    if (seen.has(abs) || !existsSync(abs)) continue
    seen.add(abs)
    kept.push(abs)
  }
  return kept
}

function childEnv(st) {
  const env = {
    ...process.env,
    PYTHONUNBUFFERED: '1',
    PYTHONIOENCODING: 'utf-8',
    // 不往包目录里写 __pycache__：包可能只读，也没必要留运行产物
    PYTHONDONTWRITEBYTECODE: '1',
    HANDLINK_ROOT: st.dataRoot,
  }
  const merged = [st.projectDir, ...extraPaths(st), process.env.PYTHONPATH].filter(Boolean)
  env.PYTHONPATH = merged.join(delimiter)
  return env
}

function probePython(command, prefix) {
  try {
    const r = spawnSync(command, [...prefix, '-c',
      'import sys;print(sys.executable);print("%d.%d.%d"%sys.version_info[:3])'], {
      timeout: 8000, windowsHide: true, encoding: 'utf8', env: process.env,
    })
    if (r.error || r.status !== 0) return null
    const lines = String(r.stdout || '').trim().split(/\r?\n/).map((s) => s.trim()).filter(Boolean)
    const version = lines.pop() || ''
    const exe = lines.pop() || ''
    const [maj, min] = version.split('.').map(Number)
    if (!(maj > 3 || (maj === 3 && min >= 8))) return null
    // 记下解释器的绝对路径：后面一律用它启动，避免 PATH 解析出来的 ENOENT
    return { command, prefix, version, exe: exe && existsSync(exe) ? exe : '' }
  } catch { return null }
}

function detectPython(st) {
  const tries = []
  if (st.config.pythonPath) tries.push({ command: String(st.config.pythonPath), prefix: [] })
  if (process.platform === 'win32') {
    tries.push({ command: 'python', prefix: [] })
    tries.push({ command: 'py', prefix: ['-3'] })
    tries.push({ command: 'python3', prefix: [] })
  } else {
    tries.push({ command: 'python3', prefix: [] })
    tries.push({ command: 'python', prefix: [] })
  }
  const dshRuntime = join(homedir(), '.dsh', 'dsh-runtimes', 'dsh-primary-runtime', 'dependencies', 'python',
    process.platform === 'win32' ? 'python.exe' : 'bin/python3')
  if (existsSync(dshRuntime)) tries.push({ command: dshRuntime, prefix: [] })

  for (const t of tries) {
    const hit = probePython(t.command, t.prefix)
    if (hit) return hit
  }
  return null
}

const DEP_MODULES = ['numpy', 'cv2', 'PIL', 'mediapipe', 'sounddevice', 'serial']

/** 解释器的绝对路径（可能为空，空则退回命令名 + 前缀参数）。 */
function pyExe(py) { return py.exe || py.command }
function pyPrefix(py) { return py.exe ? [] : py.prefix }

function probeDeps(st, py) {
  const code = [
    'import importlib.util as u, json',
    `out = {}`,
    `for m in ${JSON.stringify(DEP_MODULES)}:`,
    '    try: out[m] = u.find_spec(m) is not None',
    '    except Exception: out[m] = False',
    'print(json.dumps(out))',
  ].join('\n')
  try {
    const r = spawnSync(pyExe(py), [...pyPrefix(py), '-c', code], {
      timeout: 15000, windowsHide: true, encoding: 'utf8', env: childEnv(st),
    })
    if (r.error || r.status !== 0) return null
    return JSON.parse(String(r.stdout || '{}').trim() || '{}')
  } catch { return null }
}

function portFree(port) {
  return new Promise((res) => {
    const srv = createServer()
    srv.once('error', () => res(false))
    srv.once('listening', () => srv.close(() => res(true)))
    srv.listen(port, '127.0.0.1')
  })
}

async function freePort(start) {
  for (let p = start; p < start + 20; p++) {
    if (await portFree(p)) return p
  }
  return start
}

function killChild(st) {
  const child = st.child
  st.child = null
  if (!child || child.killed) return
  try {
    if (process.platform === 'win32' && child.pid) {
      spawnSync('taskkill', ['/pid', String(child.pid), '/T', '/F'], { windowsHide: true, stdio: 'ignore' })
    } else {
      child.kill('SIGTERM')
    }
  } catch { /* 已经退出 */ }
}

function explainFailure(st, code, signal) {
  const joined = st.log.map((l) => l.text).join('\n')
  const missing = [...new Set([...joined.matchAll(/No module named '([A-Za-z0-9_.]+)'/g)].map((m) => m[1]))]
  if (missing.length) {
    st.deps = st.deps || {}
    for (const m of missing) st.deps[m === 'cv2' ? 'cv2' : m] = false
    return `服务启动失败：缺少 Python 依赖 ${missing.join(', ')}。点「安装依赖」自动装到本包内，或手动执行 pip install -r requirements.txt。`
  }
  const trace = [...st.log].reverse().find((l) => l.stream === 'stderr' && l.text.trim())
  if (trace) return `服务进程退出（code=${code ?? '?'}）：${trace.text.trim()}`
  return `服务进程退出（code=${code ?? '?'}${signal ? `, signal=${signal}` : ''}），日志里没有更多信息。`
}

async function waitReady(st, gen, timeoutMs) {
  const deadline = Date.now() + timeoutMs
  while (Date.now() < deadline) {
    if (gen !== st.generation) return false
    if (st.phase === 'error') return false
    try {
      const r = await fetch(`${st.origin}/health`, { signal: AbortSignal.timeout(1500) })
      if (r.ok) {
        st.health = await r.json().catch(() => null)
        return true
      }
    } catch { /* 还没起来 */ }
    await sleep(250)
  }
  return false
}

async function start(st) {
  killChild(st)
  const gen = ++st.generation
  st.phase = 'starting'
  st.error = ''
  st.health = null

  const py = detectPython(st)
  if (!py) {
    st.phase = 'error'
    st.error = '没找到可用的 Python 3.8+。装上 Python 后重开 DSH，或在插件配置里写 pythonPath。'
    return
  }
  st.python = `${py.version} · ${py.command}`
  st.deps = probeDeps(st, py)

  // 源码目录可能被移动/删掉（尤其目录链接安装时被卸载带走）：先确认，再退回包内那份
  if (!existsSync(join(st.projectDir, 'handlink', '__init__.py'))) {
    const fallback = join(HERE, 'python')
    if (existsSync(join(fallback, 'handlink', '__init__.py'))) {
      pushLog(st, 'info', `源码目录不可用（${st.projectDir}），改用包内：${fallback}`)
      st.projectDir = fallback
      st.sitePackages = join(fallback, 'site-packages')
    } else {
      st.phase = 'error'
      st.error = `找不到 handlink 源码：${st.projectDir} 里没有 handlink/__init__.py，包内 python/ 也没有。重装一次插件，或把 projectDir 指到正确的源码目录。`
      return
    }
  }

  // 最新版默认端口是 8766（旧版 8765）；配置里写了就以配置为准，被占用再往后找
  const configured = Number(st.config.port) || 8766
  const port = await freePort(configured)
  if (gen !== st.generation) return
  st.port = port
  st.origin = `http://127.0.0.1:${port}`

  try {
    mkdirSync(st.recordDir, { recursive: true })
  } catch { /* dataRoot 只读也能跑，只是录制会失败 */ }

  // --root 是全局参数，必须写在子命令前面
  const args = [...pyPrefix(py), '-m', 'handlink', '--root', st.dataRoot, 'serve',
    '--host', '127.0.0.1', '--port', String(port)]
  st.log = []
  pushLog(st, 'info', `启动：${pyExe(py)} ${args.join(' ')}`)
  pushLog(st, 'info', `源码：${st.projectDir}`)
  pushLog(st, 'info', `数据：${st.dataRoot}`)

  let child
  try {
    child = spawn(pyExe(py), args, {
      cwd: st.projectDir,
      env: childEnv(st),
      windowsHide: true,
      stdio: ['ignore', 'pipe', 'pipe'],
    })
  } catch (e) {
    st.phase = 'error'
    st.error = `无法启动 Python：${e?.message ?? e}`
    return
  }
  st.child = child
  st.pid = child.pid || null
  st.startedAt = Date.now()

  child.stdout?.setEncoding('utf8')
  child.stderr?.setEncoding('utf8')
  child.stdout?.on('data', (d) => pushLog(st, 'out', d))
  child.stderr?.on('data', (d) => pushLog(st, 'err', d))
  child.once('error', (e) => {
    if (gen !== st.generation) return
    st.child = null
    st.phase = 'error'
    st.error = `无法启动 Python：${e?.message ?? e}`
  })
  child.once('exit', (code, signal) => {
    if (gen !== st.generation) return
    st.child = null
    st.pid = null
    if (st.phase === 'starting' || st.phase === 'ready') {
      st.phase = 'error'
      st.error = explainFailure(st, code, signal)
    }
  })

  const ok = await waitReady(st, gen, READY_TIMEOUT_MS)
  if (gen !== st.generation) return
  if (ok) {
    st.phase = 'ready'
    st.error = ''
    pushLog(st, 'info', `服务就绪：${st.origin}`)
    ensureGlove(st, gen)
      .catch(() => {})
      .finally(() => { if (gen === st.generation) startHmaxBridge(st) })
  } else {
    st.phase = 'error'
    if (!st.error) st.error = `服务在 ${Math.round(READY_TIMEOUT_MS / 1000)} 秒内没有响应，看下面的日志。`
  }
}

/** 数据手套参数：跟服务里的 glove.py 对齐。 */
function gloveConfig(st) {
  const c = st.config || {}
  const source = ['auto', 'sdk', 'udp'].includes(String(c.gloveSource || '').toLowerCase())
    ? String(c.gloveSource).toLowerCase()
    : 'auto'
  return {
    auto: c.gloveAuto !== false,
    source,                                  // auto/sdk = 插件自己开直连小桥；udp = 只等 MotionVenus 转发
    bridge: source !== 'udp',
    udp: Number(c.gloveUdp) || 5001,
    side: String(c.gloveSide || '').toLowerCase() === 'left' ? 'left' : 'right',
    gain: Number(c.gloveGain) || 1.0,
    hz: Math.max(1, Math.min(240, Number(c.gloveHz) || 60)),
  }
}

/** 找厂家运行目录：里面要有 h2sdk_Release_v120_x86.dll。 */
function findHmaxDir(st) {
  const explicit = String(st.config.hmaxDir || '').trim()
  if (explicit && existsSync(join(explicit, 'h2sdk_Release_v120_x86.dll'))) return explicit
  const roots = [process.env['ProgramFiles(x86)'], process.env.ProgramFiles].filter(Boolean)
  for (const root of roots) {
    try {
      for (const name of readdirSync(root)) {
        if (!/^MotionVenus_HMax/i.test(name)) continue
        const dir = join(root, name)
        if (existsSync(join(dir, 'h2sdk_Release_v120_x86.dll'))) return dir
      }
    } catch { /* 目录读不了就跳过 */ }
  }
  return ''
}

function hmaxWorkDir() {
  const base = process.env.LOCALAPPDATA || join(homedir(), 'AppData', 'Local')
  return join(base, 'handlink', 'hmax')
}

/** 把小桥和厂家配置摆到可写目录（SDK 会在 exe 旁边找 Resources\h2GloveConfig）。 */
function prepareHmax(st) {
  const vendor = findHmaxDir(st)
  if (!vendor) return { ok: false, error: '没找到 MotionVenus 安装目录（h2sdk_Release_v120_x86.dll）' }
  const srcExe = join(HERE, 'tools', 'hmax', 'HmaxBridge.exe')
  if (!existsSync(srcExe)) return { ok: false, error: '插件里缺 tools/hmax/HmaxBridge.exe' }
  const work = hmaxWorkDir()
  try {
    mkdirSync(work, { recursive: true })
    const dstExe = join(work, 'HmaxBridge.exe')
    const a = statSync(srcExe)
    const b = existsSync(dstExe) ? statSync(dstExe) : null
    if (!b || a.size !== b.size || a.mtimeMs > b.mtimeMs) copyFileSync(srcExe, dstExe)
    const resSrc = join(vendor, 'Resources')
    const resDst = join(work, 'Resources')
    mkdirSync(resDst, { recursive: true })
    for (const rel of ['h2GloveConfig', 'x2suitmgr.json']) {
      const from = join(resSrc, rel)
      if (existsSync(from)) cpSync(from, join(resDst, rel), { recursive: true, force: true })
    }
  } catch (e) {
    return { ok: false, error: `准备 HMax 运行目录失败：${e?.message ?? e}` }
  }
  return { ok: true, vendor, work, exe: join(work, 'HmaxBridge.exe') }
}

function pushBridgeLog(st, text) {
  for (const line of String(text).split(/\r?\n/)) {
    const clean = line.trim()
    if (!clean) continue
    st.bridge.log.push(clean.slice(0, 400))
    if (st.bridge.log.length > 60) st.bridge.log.splice(0, st.bridge.log.length - 60)
  }
}

function stopHmaxBridge(st) {
  const child = st.bridge.child
  st.bridge.child = null
  st.bridge.pid = null
  st.bridge.waiting = false
  if (!child || child.killed) return
  try {
    if (process.platform === 'win32' && child.pid) {
      spawnSync('taskkill', ['/pid', String(child.pid), '/T', '/F'], { windowsHide: true, stdio: 'ignore' })
    } else {
      child.kill('SIGTERM')
    }
  } catch { /* 已经退出 */ }
}

/** 启动直连小桥：32 位 exe 加载 h2sdk，把官方 ASCII 流发到 UDP <gloveUdp>。 */
function startHmaxBridge(st) {
  const cfg = gloveConfig(st)
  stopHmaxBridge(st)
  st.bridge.error = ''
  st.bridge.sn = ''
  if (!cfg.bridge) {
    pushLog(st, 'info', '手套数据源：只等 MotionVenus 的 UDP 转发（gloveSource=udp）')
    return
  }
  const prep = prepareHmax(st)
  if (!prep.ok) {
    st.bridge.error = prep.error
    pushLog(st, 'info', `手套直连没起来：${prep.error}；退回 UDP（MotionVenus 开着就能用）`)
    return
  }
  const args = ['--dir', prep.vendor, '--udp', `127.0.0.1:${cfg.udp}`, '--hz', String(cfg.hz)]
  let child
  try {
    child = spawn(prep.exe, args, {
      cwd: prep.work, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'],
    })
  } catch (e) {
    st.bridge.error = String(e?.message ?? e)
    pushLog(st, 'err', `手套直连启动失败：${st.bridge.error}`)
    return
  }
  st.bridge.child = child
  st.bridge.pid = child.pid || null
  st.bridge.startedAt = Date.now()
  pushLog(st, 'info', `手套直连：${prep.exe}（SDK：${prep.vendor}，→ UDP ${cfg.udp}，${cfg.hz} Hz）`)
  child.stdout?.setEncoding('utf8')
  child.stderr?.setEncoding('utf8')
  child.stdout?.on('data', (d) => {
    pushBridgeLog(st, d)
    const sn = /sn=0x([0-9a-f]+)/i.exec(String(d))
    if (sn) st.bridge.sn = sn[1].toUpperCase()
    if (/devices=0/.test(String(d)) && !st.bridge.waiting) {
      st.bridge.waiting = true
      pushLog(st, 'info', '手套直连在等接收器：COM 口被占时（MotionVenus 还开着）会一直等，关掉它就会自动接上')
    }
  })
  child.stderr?.on('data', (d) => pushBridgeLog(st, d))
  child.once('error', (e) => {
    st.bridge.error = String(e?.message ?? e)
    pushLog(st, 'err', `手套直连进程起不来：${st.bridge.error}`)
    st.bridge.child = null
    st.bridge.pid = null
  })
  child.once('exit', (code) => {
    if (st.bridge.child !== child) return
    st.bridge.child = null
    st.bridge.pid = null
    pushLog(st, 'info', `手套直连退出（code=${code}）`)
  })
}

/** 服务就绪后把 UDP 监听打开。手套这块是「服务内置」，插件只负责按配置打开它。 */
async function ensureGlove(st, gen) {
  const cfg = gloveConfig(st)
  if (!cfg.auto || st.phase !== 'ready' || gen !== st.generation) return
  try {
    const r = await fetch(st.origin + '/glove/start', {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ udp: cfg.udp, side: cfg.side, gain: cfg.gain }),
      signal: AbortSignal.timeout(5000),
    })
    if (r.status === 404) {
      pushLog(st, 'info', '包内的 handlink 还没有 /glove/start（旧版源码），手套要先手动跑 tools/glove_bridge.py')
      return
    }
    const j = await r.json().catch(() => ({}))
    if (j.ok) pushLog(st, 'info', `数据手套监听：UDP ${j.bind}:${j.udp} · ${j.side === 'left' ? '左' : '右'}手 · gain ${j.gain}`)
    else pushLog(st, 'err', `数据手套没起来：${j.error || ('HTTP ' + r.status)}${j.hint ? ' —— ' + j.hint : ''}`)
  } catch (e) {
    pushLog(st, 'info', `数据手套没起来：${e?.message ?? e}`)
  }
}

function publicStatus(st) {
  return {
    ok: st.phase === 'ready',
    phase: st.phase,
    port: st.port,
    origin: st.origin,
    python: st.python,
    projectDir: st.projectDir,
    dataRoot: st.dataRoot,
    error: st.error,
    deps: st.deps,
    health: st.health,
    pid: st.pid,
    uptimeS: st.startedAt ? Math.round((Date.now() - st.startedAt) / 1000) : null,
    log: st.log.slice(-80),
    install: st.install,
    glove: gloveConfig(st),
    bridge: {
      wanted: gloveConfig(st).bridge,
      running: !!st.bridge.pid,
      pid: st.bridge.pid,
      sn: st.bridge.sn,
      waiting: st.bridge.waiting,
      error: st.bridge.error,
      log: st.bridge.log.slice(-10),
    },
    version: PACKAGE.version || '',
  }
}

const ALLOW_GET = new Set(['/state', '/health', '/hand/status', '/actions', '/points', '/tracks',
  '/record/stats', '/limits', '/glove/state', '/glove/limits'])
const ALLOW_POST = new Set(['/pose', '/point', '/track', '/action', '/stop', '/hand/attach',
  '/record/start', '/record/stop', '/ai',
  '/glove/start', '/glove/stop', '/glove/mode', '/glove/toggle', '/glove/zero', '/glove/gain'])

function allowed(method, path) {
  if (method === 'GET') {
    return ALLOW_GET.has(path) || /^\/points\/[^/]+$/.test(path) || /^\/tracks\/[^/]+$/.test(path)
  }
  return ALLOW_POST.has(path)
}

async function forward(st, method, path, payload) {
  let body = payload
  if (path === '/record/start') {
    mkdirSync(st.recordDir, { recursive: true })
    body = { ...(payload || {}), path: (payload && payload.path) || join(st.recordDir, `live-${nowStamp()}.jsonl`) }
  }
  if (path === '/hand/attach' && !(payload && payload.dll)) {
    // 手走 ZLG USBCAN：Python 那边默认按相对路径找 ECanVci64.dll，包里的目录结构对不上，
    // 这里直接把包内自带的驱动绝对路径给它（config.canDll 可覆盖）。
    const dll = st.config.canDll
      ? resolve(String(st.config.canDll))
      : join(HERE, 'data', 'src', 'main', 'resources', 'native', 'Windows-x86_64', 'ECanVci64.dll')
    if (existsSync(dll)) body = { ...(payload || {}), dll }
  }
  const slow = path === '/ai' || path === '/track' || path === '/action'
  const r = await fetch(st.origin + path, {
    method,
    headers: { 'content-type': 'application/json' },
    body: method === 'POST' ? JSON.stringify(body || {}) : undefined,
    signal: AbortSignal.timeout(slow ? 120000 : 30000),
  })
  const text = await r.text()
  return { status: r.status, text }
}

async function installDeps(st) {
  if (st.install.running) return { ok: false, error: '依赖安装已经在跑了' }
  const py = detectPython(st)
  if (!py) return { ok: false, error: '没找到 Python，装不了依赖' }
  st.install = { running: true, message: '正在安装…', log: [] }
  const args = [...pyPrefix(py), '-m', 'pip', 'install', '--disable-pip-version-check', '--no-input',
    '-r', join(HERE, 'requirements.txt'), '--target', st.sitePackages]
  st.install.log.push('$ ' + [pyExe(py), ...args].join(' '))
  await new Promise((resolvePromise) => {
    let child
    try {
      child = spawn(pyExe(py), args, { windowsHide: true, env: { ...process.env } })
    } catch (e) {
      st.install.log.push(String(e?.message ?? e))
      resolvePromise()
      return
    }
    const onData = (d) => {
      for (const line of String(d).split(/\r?\n/)) {
        if (!line.trim()) continue
        st.install.log.push(line.slice(0, 400))
        if (st.install.log.length > 120) st.install.log.splice(0, st.install.log.length - 120)
      }
    }
    child.stdout?.on('data', onData)
    child.stderr?.on('data', onData)
    child.once('error', (e) => { st.install.log.push(String(e?.message ?? e)); resolvePromise() })
    child.once('exit', () => resolvePromise())
  })
  const ok = existsSync(st.sitePackages)
  st.install.running = false
  st.install.message = ok ? '安装完成，重启服务后生效' : '安装没成功，看日志'
  st.deps = probeDeps(st, py)
  return { ok, message: st.install.message, log: st.install.log.slice(-40), deps: st.deps }
}

/** 环境自检：跑一遍 `handlink scan`，把「有没有 CAN 适配器 / COM 口」摆到页面上。
 *  连不上真手时，这一步比红徽章有用得多。 */
async function runScan(st) {
  if (st.scanRunning) return st.scanResult || { ok: false, error: '自检正在进行' }
  const py = detectPython(st)
  if (!py) return { ok: false, error: '没找到 Python，跑不了自检' }
  st.scanRunning = true
  try {
    const out = await new Promise((resolveOut) => {
      const chunks = []
      let child
      try {
        child = spawn(pyExe(py), [...pyPrefix(py), '-m', 'handlink', '--root', st.dataRoot, 'scan'], {
          cwd: st.projectDir, env: childEnv(st), windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'],
        })
      } catch (e) {
        return resolveOut({ code: -1, text: String(e?.message ?? e) })
      }
      const timer = setTimeout(() => { try { child.kill() } catch { /* noop */ } }, 25000)
      child.stdout?.setEncoding('utf8')
      child.stderr?.setEncoding('utf8')
      child.stdout?.on('data', (d) => chunks.push(d))
      child.stderr?.on('data', (d) => chunks.push(d))
      child.once('close', (code) => { clearTimeout(timer); resolveOut({ code, text: chunks.join('') }) })
      child.once('error', (e) => { clearTimeout(timer); resolveOut({ code: -1, text: String(e?.message ?? e) }) })
    })
    const at = out.text.indexOf('{')
    let parsed = null
    if (at >= 0) { try { parsed = JSON.parse(out.text.slice(at)) } catch { /* 不是 JSON */ } }
    const result = parsed
      ? { ok: true, scan: parsed }
      : { ok: false, error: '自检输出解析不了', raw: out.text.slice(0, 2000) }
    st.scanResult = result
    return result
  } finally {
    st.scanRunning = false
  }
}

function installRoutes(ctx, st) {
  const reg = (path, handler) => ctx.effect(
    () => ctx.webServer.register({ kind: 'exact', path, handler }),
    `handlink: ${path}`,
  )

  reg('/handlink/api/status', (req, res) => json(res, 200, publicStatus(st)))

  reg('/handlink/api/py', async (req, res) => {
    try {
      const body = await readJson(req)
      const method = String(body.method || 'GET').toUpperCase() === 'POST' ? 'POST' : 'GET'
      const path = String(body.path || '')
      if (st.phase !== 'ready') {
        return json(res, 503, { ok: false, error: '服务未就绪', phase: st.phase, detail: st.error })
      }
      if (!allowed(method, path)) return json(res, 400, { ok: false, error: `不允许的接口 ${method} ${path}` })
      const out = await forward(st, method, path, body.body)
      try {
        res.writeHead(out.status, { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'no-store' })
        res.end(out.text)
      } catch { /* 断开 */ }
    } catch (e) {
      json(res, 500, { ok: false, error: String(e?.message ?? e) })
    }
  })

  reg('/handlink/api/diag', async (req, res) => {
    try {
      return json(res, 200, await runScan(st))
    } catch (e) {
      json(res, 500, { ok: false, error: String(e?.message ?? e) })
    }
  })

  reg('/handlink/api/control', async (req, res) => {
    try {
      const body = await readJson(req)
      const action = String(body.action || '')
      if (action === 'restart') {
        start(st).catch(() => {})
        return json(res, 200, { ok: true, phase: 'starting' })
      }
      if (action === 'stop') {
        st.generation++
        stopHmaxBridge(st)
        killChild(st)
        st.phase = 'stopped'
        st.error = ''
        return json(res, 200, { ok: true, phase: 'stopped' })
      }
      if (action === 'install') {
        const result = installDeps(st)
        return json(res, 200, result)
      }
      return json(res, 400, { ok: false, error: `未知操作 ${action}` })
    } catch (e) {
      json(res, 500, { ok: false, error: String(e?.message ?? e) })
    }
  })
}

export function apply(ctx, config) {
  try {
    const st = makeState(config || {})
    live = st
    installRoutes(ctx, st)

    ctx.effect(() => {
      const auto = st.config.autoStart !== false
      if (auto) {
        start(st).catch((e) => {
          st.phase = 'error'
          st.error = String(e?.message ?? e)
        })
      } else {
        st.phase = 'stopped'
        st.error = ''
      }
      return () => {
        st.generation++
        stopHmaxBridge(st)
        killChild(st)
        st.phase = 'stopped'
        if (live === st) live = null
      }
    }, 'handlink: 本地服务')

    ctx.logger?.info?.(`[handlink] 插件已加载，源码 ${st.projectDir}，数据 ${st.dataRoot}`)
  } catch (e) {
    // 插件**不许**把 DSH 弄挂：注册失败只记一条日志。
    try { ctx.logger?.error?.('[handlink] 加载失败（DSH 本身照常跑）：' + String(e?.message ?? e)) } catch { /* noop */ }
  }
}
