# 机械手控制终端（handlink-terminal）

把 `handlink` 那套机械手控制能力（逐指下发、点位、动作、录制分析、自然语言技能）
做成了 DSH 的**一个插件**：装一次，侧栏多一个「机械手」，点开就是控制终端页面。

- **不用另开网页**：页面在 DSH 主面板里，跟着主题走亮/暗两套皮肤。
- **不用手动起服务**：宿主半自动找到 Python、拉起包内自带的 `handlink` 服务。
- **数据手套也长在里面**：FOHEART HMax 默认**直连**（插件自带 32 位小桥加载厂家
  `h2sdk`），不用开 MotionVenus；收帧、换算、零点都在服务里，页面直接开关和调参。
- **没接真机也能用**：默认跑仿真总线，姿态、动作、录制、分析全部可跑通。
- **接口仍然是一套**：页面只是调用本机 `http://127.0.0.1:<端口>` 的同一组 JSON 接口，
  命令行、脚本、Notebook 调的是同一个服务。

```
装之前：python -m handlink serve ... 手动起服务，另开浏览器
装之后：打开 DSH → 侧栏「机械手」→ 拖滑块 / 点石头剪刀布 / 播放动作 / 录制分析
        戴手套 → 页面「启动监听」→ 手跟着动，抬手就能跟
```

## 装完是什么样

主面板里是一页终端：

| 区域 | 能做什么 |
| --- | --- |
| 顶部 | 服务状态、手部连接状态、手套模式、「连接机械手」、「停止」、「重启服务」、「完整页面 ↗」 |
| 手部状态 | 六指实时的 SVG 姿态，跟着 `/state` 刷新 |
| 数据手套 | 启动/停止 UDP 监听、右手/左手、跟手/代码模式、重新零点、增益、六指实时弯曲 |
| 逐指控制 | 六根手指各一条滑块，松手即下发 `/pose`（其余手指保持原位） |
| 预设点位 / 动作 | 石头 / 剪刀 / 布、`动作一` 播放、厂家预设动作组 |
| AI 控制 | 一句话拆成技能序列（张开 / 握拳 / 捏 / 剪刀 / 食指指 / 指定角度 / 保持），可只看计划 |
| 录制与分析 | 开录 / 停止并出表：行程、峰速、峰值电流、电流占用率 |
| 接口日志 / 服务输出 | 每次操作对应的真实 HTTP 调用；服务起不来时原样显示 stderr |

「完整页面 ↗」会打开原来的 `/demo` 全页（还有 `/arm3d` 三维臂、`/camera` 摄像头页）。

## 安装

前提：DSH 桌面端；插件装进当前 profile（和别的插件一样，所有会话都能用）。

**方式一：从打包文件装（推荐，发给别人也用这个）**

先在本目录打包，再把 tgz 发给对方（也适用于自己机器）：

```bash
pnpm pack        # 产出 handlink-terminal-<版本>.tgz（1.0.2 约 2.5 MB）
```

然后：

```text
plugin_manager → action: install_bundle → target: <本目录>/handlink-terminal-1.0.2.tgz
```

> tgz 是**分发件**，装完也别删：以后重装、给别人都还用它。
> profile 里装的是从它解出来的独立副本，tgz 挪走不影响已装好的插件。

**方式二：从目录装（只在自己改代码时用）**

```text
plugin_manager → action: install_bundle → target: <本目录的绝对路径>
```

> ⚠️ Windows 上目录安装是**目录联接（junction）**，不是拷贝。实测踩过：
> 移除 bundle 时 pnpm 会顺着联接把**源目录的内容一起删掉**，源目录只剩空文件夹。
> 要长期用、要发给别人，用方式一；非要目录安装，先确认源目录有版本库或备份。

装完重启一次 DSH（第一次装新 bundle 要重新加载模块表），侧栏就会出现「机械手」。
`install_bundle` 返回 `application: applied` 才算生效；返回 `restart-required` 就重启。

用方式一装时，运行的是 profile 里的**独立副本**，源目录挪走、删掉都不影响；
要更新就改源目录 → 重新 `pnpm pack` → 再装一次。

## 配置

配置写在插件行上（`cordis.patch.yml` 的 `config`），也可以在插件详情里改。
默认值已经能跑，一般不用动：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `autoStart` | `true` | 激活时自动拉起本地服务 |
| `port` | `8766` | 服务端口；被占用会自动往后找（页面顶部显示实际端口） |
| `pythonPath` | `''` | 指定解释器；留空依次找 `python` / `py -3` / `python3` |
| `projectDir` | `''` | 用哪份 handlink 源码；留空用包内 `python/` |
| `dataRoot` | `''` | 点位 / 动作 / 录制目录；留空用包内 `data/` |
| `extraPaths` | `[]` | 额外的依赖目录（会拼进 `PYTHONPATH`） |
| `canDll` | `''` | ZLG USBCAN 驱动 DLL；留空用包内 `data/src/.../ECanVci64.dll` |
| `gloveAuto` | `true` | 服务就绪后自动收手套 |
| `gloveSource` | `auto` | `auto`/`sdk` 走内置直连小桥；`udp` 只等 MotionVenus 转发 |
| `gloveUdp` | `5001` | 手套 UDP 端口（小桥也发到这里） |
| `gloveSide` | `right` | 戴的是哪只手：`right` / `left` |
| `gloveGain` | `1.0` | 手套→电机行程的增益（握不满调大，一下就到底调小） |
| `gloveHz` | `60` | 直连小桥的出帧率 |
| `hmaxDir` | `''` | MotionVenus 安装目录（取 h2sdk DLL）；留空自动找 |

改完保存就生效：宿主半在 `ctx.effect` 里起服务，卸载/重载会连带把子进程杀掉，
不会留下孤儿进程。

## 第一次启动做了什么

1. 找 Python：`pythonPath` → `python` → `py -3` → `python3` → DSH 自带运行时，要求 ≥ 3.8；
2. 探依赖：`numpy / cv2 / PIL / mediapipe / sounddevice / serial` 谁在谁不在，写进状态；
3. 找依赖目录：包内 `python/site-packages`、`HANDLINK_PATHS`、`config.extraPaths`，
   以及开发布局下的 `.tmp/pylibs`；
4. 找端口：从配置端口往后试，避免和别的手起服务打架；
5. 起服务：`python -m handlink --root <dataRoot> serve --host 127.0.0.1 --port <port>`
   （用探测到的解释器绝对路径启动；`projectDir` 不可用时自动退回包内 `python/`）；
6. 等到 `/health` 通才算就绪，失败就把 stderr 尾部显示在页面上。

**核心控制不需要任何第三方包**（只用 Python 标准库）。`numpy / cv2 / PIL` 等只在
开摄像头、视觉识别、串口真机时才需要。

## 缺依赖怎么办

页面会直接列出缺哪个，两个选择：

- 点**「安装依赖」**：宿主执行
  `python -m pip install -r requirements.txt --target python/site-packages`，
  装进**本包内**，不动系统 Python；装完点「重启服务」。
- 或者手动装到任意目录，然后把该目录写进 `config.extraPaths`（或环境变量 `HANDLINK_PATHS`，
  多个用分号隔开）。

## 接真机

页面默认是仿真总线。真手走 **ZLG USBCAN**（ECanVci 接口）：

1. 插上 ZLG USBCAN 盒子，装好它的驱动。设备管理器里要能看到它；
   盒子被厂家的 `LG-603.exe` 占用时也打不开——两个程序不能同时拿这一个盒子。
2. 包内已经带了驱动 DLL：
   `data/src/main/resources/native/Windows-x86_64/ECanVci64.dll`，
   页面点「连接机械手」时会把它作为绝对路径传给 Python（`config.canDll` 可覆盖）。
3. 点页面右上角**「连接机械手」**（等价于 `POST /hand/attach {"bus":"can","dll":"…"}`）；
4. 徽章变绿才是真连上——判断标准是「最近 3 秒内还在收 CAN 帧」，
   线松了、手断电会立刻变红，不是「适配器打开过」就算连上。

连不上时页面会自动跑一次**环境自检**，把三件事摆出来：

| 现象 | 意思 |
| --- | --- |
| `VCI_OpenDevice 失败：适配器未插好或被占用` | 驱动加载成功，但盒子没应答：没插、驱动没装，或被 LG-603 抢占 |
| `COM 口 0` | 本机没有任何 USB-串口设备（手不是串口接法，或线没通） |
| `没找到 ECanVci64.dll` | 驱动文件不在（换了 `dataRoot` 又没配 `canDll`） |

设备管理器里如果连「未知设备/黄色感叹号」都没有，那就是这根线根本没通到这台电脑，
先解决物理连接，再谈协议。

协作臂（Dobot Nova 5）属于另一条链路，插件页暂时只给了「完整页面」入口，
可以在 `python -m handlink serve --arm 192.168.5.1` 下看 `/arm3d`。

## 数据手套（FOHEART HMax）

手套默认**直连**，不用开 MotionVenus。插件自带一个 32 位小桥（`tools/hmax/HmaxBridge.exe`，
源码 `HmaxBridge.cs` 就在旁边），加载厂家的 `h2sdk_Release_v120_x86.dll`，把官方的
`Glove1 … fn …` 文本流直接发到本地 UDP 5001，服务里的 `glove.py` 照旧解析、换算、下发。

```
直连（默认）：手套 → USB 接收器 → HmaxBridge.exe（h2sdk）→ UDP 127.0.0.1:5001 → glove.py → CAN / 仿真
软件转发：    手套 → MotionVenus → UDP 5001                                 （gloveSource=udp 时）
```

用起来就一步：**戴上手套** → 卡片里出现「跟随中 · N Hz」。
MotionVenus 只在装驱动/DLL 时有用（`hmaxDir` 指它的安装目录），平时**不用启动**。

| 配置 | 默认 | 说明 |
| --- | --- | --- |
| `gloveSource` | `auto` | `auto`/`sdk` 用直连小桥；`udp` 只等 MotionVenus 转发 |
| `gloveUdp` | `5001` | 小桥发到哪个口，也就是服务的收帧口 |
| `gloveSide` | `right` | 戴的是哪只手 |
| `gloveGain` | `1.0` | 0.3~2.0，握不满调大 |
| `gloveHz` | `60` | 小桥出帧率（手套本身约 32 Hz） |
| `hmaxDir` | 空 | MotionVenus 安装目录；留空自动找 `Program Files (x86)\MotionVenus_HMax-*` |

小桥的运行目录是 `%LOCALAPPDATA%\handlink\hmax`：exe 从包里复制过去，厂家的
`Resources\h2GloveConfig`（含这只手套的 SN / 校准）每次启动从安装目录同步，所以
换手套、重新标定后重启一下服务就行。

**状态怎么看**：卡片「数据源」一行会写「直连 SDK · SN 0x1403403E」或「MotionVenus 转发」；
`GET /glove/state` 照旧（`running / mode / live / fps / frames / applied / raw / hint`）；
小桥自己的状态在 `GET /handlink/api/status` 的 `bridge` 字段（`running / pid / sn / waiting / log`）。

| 页面操作 | 等价接口 | 说明 |
| --- | --- | --- |
| 启动监听 | `POST /glove/start {"udp":5001,"side":"right","gain":1}` | 端口被占会回 422 + 提示 |
| 停止监听 | `POST /glove/stop` | 松开 UDP，手的姿态停在最后一帧 |
| 切到代码模式 | `POST /glove/mode {"mode":"code"}` | 手套照收不误，但不下发；滑块/脚本可以独占手 |
| 恢复手套操控 | `POST /glove/mode {"mode":"glove"}` / `POST /glove/toggle` | 下一帧手套数据立刻接管 |
| 重新零点 | `POST /glove/zero` | 把当前姿态记成摊平基线 |
| 增益 －/＋ | `POST /glove/gain {"gain":1.2}` | 0.1 一档，范围 0.3~2.0 |
| 换算方式 | `POST /glove/frame {"frame":"rel"}` | `rel` 减手腕（默认）/`rel-ext`/`abs`；换了会清基线 |
| 死区 | `POST /glove/deadband {"deadband":{"102":5}}` | 抹掉指定电机几度，拇指残抖用，别超 10° |
| 教屈曲轴 | `POST /glove/teach {"action":"start"}` → 握一次 → `{"action":"stop"}` | 2 秒；换人/重戴都要重教，轴存 `config/glove_axes.json` |

三个坑：

- **COM3 同时只能一个程序用**。MotionVenus 开着就占着接收器，小桥会显示
  「等接收器」，日志里 `devices=0`；关掉 MotionVenus，它 1 秒内自己接上，不用重启。
- **同一时刻只有一个程序能监听 5001**。老的 `tools/glove_bridge.py` 要是还在跑，
  「启动监听」会失败并提示，先关掉它。
- **"手不动、手臂转一圈，大拇指自己动"** 是 2026-11-20 修掉的：帧里的欧拉角是绝对姿态，
  老算法直接取横向分量，手臂一转就涨；拇指骨头垂直于小臂，投影最狠。现在默认
  `frame=rel` 先减掉手腕。真机上 `python tools/glove_probe.py --seconds 20` 量一次，
  手指别动只转手臂，表格里 rel 那一行合计该接近 0。
- **"拇指往外展"** 是同一天第二修：光减手腕不够，剩下的无符号转角会把拇指 CMC 的外展、
  骨头自转全算成"弯"。现在每节沿教出来的屈曲轴做**有符号**投影，并且拇指拆成
  `102←Thumb1`、`107←Thumb2..3` 不再重叠。**换人/重戴必须重教一次轴**（2 秒），
  否则它还在用上一个人的轴。
- **手套是逐帧覆盖的**。跟手模式下，滑块、点位、动作按钮下发后会被下一帧覆盖——
  要跑脚本/动作先切「代码模式」。

不想用插件时，命令行也一样：`python -m handlink serve --glove`（还有
`--glove-udp` / `--glove-side` / `--glove-gain`）。

## 数据放哪

```
handlink-plugin/
  data/
    points/        点位（paper / rock / scissors，厂家格式）
    tracks/        动作（动作一）
    recordings/    录制出来的 JSONL
```

点「开始录制」写的是 `data/recordings/live-<时间>.jsonl`；离线复算：

```bash
python -m handlink analyze data/recordings/live-xxxx.jsonl --markdown
```

要换成自己的数据目录，把 `dataRoot` 指过去即可（该目录下需要有 `points/` 和 `tracks/`）。
服务只监听 `127.0.0.1`，不对外网暴露。

## 常见问题

**页面显示「服务未就绪」**
页面里直接有原因和 stderr 尾部：缺 Python、缺依赖、端口被占、源码路径不对，都会写明。
先看「服务输出」。

**端口和手起的服务撞了**
插件会自动换端口（比如 8767），页面顶部写的就是实际端口。想固定就改 `port`。

**改了包内 Python 代码没生效**
服务是子进程，改完点「重启服务」。改了 `index.js` / `client.js` 需要重装或重启 DSH。

**手套卡片一直「监听中 · 无数据」**
UDP 已经在听，但 MotionVenus 没往这个端口发：检查它里面的转发目标是不是
`127.0.0.1`、端口是不是和卡片上一致；换了左手/右手要在卡片里切过来（`side`）。
还是没数据就 `python -m handlink.glove --dry-run --verbose` 单独收一次，
它有帧会按 50 帧打一行原始角度。

**「启动监听」报端口被占**
5001 被占着：多半是 `tools/glove_bridge.py` 还在跑（关掉它），或者上一条 MotionVenus
的转发进程没退。换个端口也行，但两边要改成一样。

**录制的表是空的**
先「开始录制」，做几个动作，再「停止并分析」；`samples` 为 0 说明录制期间没有采样。

**想停掉服务**
页面「停止」只停机械手（归零）；`POST /handlink/api/control {"action":"stop"}`
才是停整个 Python 进程；卸载插件也会把它带走。

## 卸载

插件管理器里移除 bundle，或在 profile 里禁用这一行。
包内的 `data/recordings` 是数据，删之前自己确认要不要留。

## 目录结构

```
handlink-plugin/
  package.json          dsh.bundle.patch + dsh.client 声明
  cordis.patch.yml      插入 handlink-terminal 这一行（含默认配置）
  index.js              宿主半：找 Python、起服务、同源代理 /handlink/api/*
  client.js             浏览器半：侧栏入口 + 主面板终端页
  locale/{zh,en}.json   插件管理器里显示的名称和描述
  icon.svg              侧栏 / 插件列表的图标
  python/handlink/      包内自带的 handlink 源码（含 glove.py、static/）
  data/                 点位 / 动作 / 录制
  requirements.txt      可选依赖（摄像头、视觉、串口）
```

宿主半开的接口（都只在 DSH 自己的源下）：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/handlink/api/status` | 服务状态、端口、Python、依赖、日志、手套配置 |
| POST | `/handlink/api/py` | 转发到本机服务，带白名单（`/state`、`/pose`、`/track`、`/glove/*`…） |
| POST | `/handlink/api/control` | `restart` / `stop` / `install` |
| POST | `/handlink/api/diag` | 跑一次 `handlink scan`（CAN 盒子/COM 口/摄像头） |
