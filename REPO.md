# 仓库说明（给接手的人 / 给未来的自己）

这个仓库是**一台机械手的工作现场**，不是库。所以它跟常见的开源项目不太一样，
先看这一页再动手，能少踩几个坑。

## 一、这是什么

中科灵犀 **LG-603** 六自由度灵巧手（CAN 总线）+ 越疆 **Dobot Nova 5** 协作臂
（TCP 29999 文本 / 30004 二进制）的一整套控制栈：点位、动作、录制、视觉、
三维视图、拟真仿真、回归测试。

两条链路是独立的：手走 CAN，臂走网口。手断了不影响臂，反之亦然。

```
handlink/            主库（唯一一份源码）
  arm.py             臂：SimArm（虚拟时钟）+ DobotArm（真机 29999/30004）
  armsim.py          VirtualDobot：真 socket 的虚拟控制器，会失败
  bus.py/canbus.py   手：SimBus / HandBus（CAN 帧）
  handsim.py         HandSim：带电流约束和心跳的仿真手
  faults.py          13 个故障场景（真机咬过人的那一批）
  service.py         HTTP 服务，所有上层（CLI / 网页 / 插件 / 脚本）共用的入口
  static/            三维视图资源（three.js + Nova STL）
  models/            mediapipe 手部模型（7.5 MB，随仓库走）
handlink-plugin/     DSH Web UI 插件（包装上面这套，供桌面端调用）
  python/handlink/   ← 不在这里，由 scripts/sync_plugin.py 生成
scripts/            一次性脚本 / 真机操作 / 仿真报告
tests/              522 条回归
docs/               设计文档与实测记录（现状.md 是所有数字的出处）
demo/               演示脚本与录制样本
pkg/                厂家原始资料（手册、CAN 头文件、jar）—— 大，见下
```

## 二、跑起来

只需要 Python 3.13，**核心功能零第三方依赖**（标准库够用）。
摄像头 / 视觉 / 串口才需要装：

```bash
python -m pip install -r handlink-plugin/requirements.txt
```

> 用 `python -m handlink selftest`，不要用 `python -m unittest discover -s tests`：
> 仓库路径含中文，unittest 从命令行拿路径会按 GBK 解码成乱码，报
> “Start directory is not importable”。`selftest` 是 Python 自己算的路径，没这问题。

自检和离线跑：

```bash
python -m handlink selftest                      # 全量回归（约 200 秒）
python -m handlink serve --port 8766             # 网页演示
python -m handlink serve --sim-arm               # 带仿真臂，不碰真机
python scripts/sim_report.py                     # 仿真 vs 真机数字对照
```

## 三、几条硬规矩

**1. 真机地址只有一个：`192.168.5.1`。**
所有碰真机的代码都走 `service.py` 的判断；`--sim-arm` 用系统分配端口，
现场开着真机时起仿真也不会打到它。写新脚本请沿用这套，不要自己写死 IP。

**2. `handlink-plugin/python/handlink/` 是生成物，不许手改。**
它是 `handlink/` 的副本，靠 `python scripts/sync_plugin.py` 同步。
改源码 → 同步 → `pnpm pack`，顺序错了装上的是旧代码。这个坑本项目真踩过：
插件里跑的是 2026-11-18 的 `arm.py`，源码已经改到 11-26 了。

**3. 大文件不进仓库。**
`pkg/`（554 MB 厂家资料）、`dist/`、`*.tgz`、`*.exe`、`.tmp/`（786 MB 实验残留）
都在 `.gitignore` 里。需要厂家手册请从 `pkg/` 单独取，别提交。

**4. 真机测试要手动开。**
`tests/` 里默认不连真机；要跑真机用例先看 `docs/现状.md`，
那是所有实测数字（GetPose 3.9 ms、ServoJ ACK 6.2 ms、0.6 A 卡 118°…）的出处。

## 四、动手之前先读哪几份

| 想知道 | 读 |
| --- | --- |
| 现在到底能干什么、哪些没通 | `docs/现状.md` |
| 仿真做到多真、哪些没建模 | `docs/拟真环境.md` |
| 臂的 TCP 协议细节 | `docs/现状.md` 第 3 节 + `.tmp/` 里的手册摘录（不在库中） |
| 团队约定 | `协作规矩.md` |
| 完整功能清单 | `README.md` |

## 五、已知还欠着的

- `pick.clenched()` 用 `angle ≥ target×0.6` 判合手，0.6 A 卡在 118.6° 时会误判通过。
  真 bug，已在 `tests/test_pick_realistic.py::TestKnownInterlockHole` 钉住，**待修**。
- `kin.ik` 对某些演示位姿（如 `(400,0,300,180,0,0)`）会落进奇异支管，误差几百毫米。
- J6 累积漂移在仿真里复现不出来（虚拟控制器没有 IK 分支记忆），防缠逻辑只能上机验。
- 接触力、摩擦、物体滑落、振动、温漂都没建模。
