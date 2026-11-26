# AI 控制接入指南（给社员）

目标：**社员写自己的 AI，让它"听懂人话"去操控灵巧手**，而不用碰 CAN、串口和角度。

## 0. 一分钟上手

```bash
curl -s -X POST http://127.0.0.1:8766/ai \
  -H "Content-Type: application/json" \
  -d "{\"text\":\"先张开，再握拳，然后食指指一下\"}"
```

返回里会有：解析出的步骤、来源（rule / llm）、执行结果。

只想看计划不动手：

```bash
curl -s -X POST http://127.0.0.1:8766/ai \
  -H "Content-Type: application/json" \
  -d "{\"text\":\"轻轻捏住，保持两秒\",\"dry_run\":true}"
```

## 1. 核心设计：AI 不写角度，只选技能

```
自然语言 → [规划器] → 技能序列(JSON) → [校验] → 总线 → 手
```

**为什么不让模型直接输出六个角度？**

| 直接写角度 | 只选技能 |
| --- | --- |
| 模型容易越界（999°） | 白名单里没有越界这个选项 |
| 出错不知道错在哪 | 每一步都能翻译成中文 |
| 换个模型行为全变 | 换模型不影响安全边界 |
| 没法测 | 技能可以单测 |

## 2. 技能白名单

| 技能 | 作用 | 关键参数 |
| --- | --- | --- |
| `open` / `paper` | 张开 | speed, torque, delay_ms |
| `close` / `rock` | 握拳、抓 | 同上 |
| `scissors` | 剪刀 | 同上 |
| `pinch` | 两指捏（拇指+食指） | 同上 |
| `tripod` | 三指捏 | 同上 |
| `point` | 食指伸出 | 同上 |
| `set_fingers` | 指定手指角度 | `angles: {"食指": 90}` |
| `hold` | 保持一段时间 | `ms` |

上限：速度 ≤120、电流 ≤0.5 A、单次 ≤12 步、总时长 ≤8 秒。角度的越界会被**夹到限位**，
不会报错中断（AI 场景要能跑下去）。

```bash
curl -s http://127.0.0.1:8766/ai/skills    # 机器可读的完整清单
```

## 3. 计划格式（模型就输出这个）

```json
{
  "steps": [
    {"skill": "open",  "args": {}, "delay_ms": 500},
    {"skill": "close", "args": {"torque": 0.4}, "delay_ms": 600},
    {"skill": "point", "args": {}, "delay_ms": 800}
  ]
}
```

## 4. 三种接入方式

**① HTTP**（任何语言都能接）

```
POST /ai          {"text": "...", "dry_run": false, "prefer_llm": false, "speed_scale": 1.0}
GET  /ai/skills   技能清单
GET  /state       六指实时状态
POST /pose        直接下发姿态（连续控制用这个）
POST /stop        停
```

**② Python**

```python
import json, urllib.request

def ai(text: str):
    body = json.dumps({"text": text}).encode()
    req = urllib.request.Request("http://127.0.0.1:8766/ai", data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read())

print(ai("先张开，再轻轻捏住，然后保持一秒"))
```

**③ 自己写规划器**（想接自己的模型）

```python
from handlink import ai, skills

class MyPlanner:
    def plan(self, text):
        # 你的模型：输出 {"steps":[{"skill":"open","args":{},"delay_ms":500}]}
        payload = my_llm(text, allowed=skills.schema())
        steps = [ai.Step(s["skill"], s.get("args", {}), int(s.get("delay_ms", 500)))
                 for s in payload["steps"]]
        return ai.Plan(text=text, steps=steps, source="my-model")

plan = MyPlanner().plan("把食指弯到 90 度")
print(ai.explain(plan))          # 食指 → 90°(400ms)
```

`ai.Executor(app.apply)` 负责执行；`app.apply` 里已经有校验和录制。

## 5. 接大模型

内置 `LLMPlanner` 走 **OpenAI 兼容**接口，环境变量配好即用：

```bash
set HANDLINK_LLM_BASE=https://api.deepseek.com/v1
set HANDLINK_LLM_KEY=sk-xxx
set HANDLINK_LLM_MODEL=deepseek-chat
python -m handlink ai "先张开再握拳" --llm
```

本地 Ollama：`HANDLINK_LLM_BASE=http://127.0.0.1:11434/v1`、KEY 随便填、MODEL 填模型名。

**模型失败自动退回规则规划器**——演示时断网也不至于开天窗。

## 6. 延迟的现实（别被"实时"骗了）

| 环节 | 典型耗时 |
| --- | --- |
| 规则解析 | < 1 ms |
| 大模型规划 | 1–3 秒（取决于模型和网络） |
| 下发 + 手到位 | 0.3–1 秒（看行程和速度） |
| 20 Hz 连续控制（`/pose`） | 50 ms/帧 |

所以：

- **"说一句话做一个动作"** → 走 `/ai`，1–3 秒可接受；
- **"我手动它手动"（手套/摄像头）** → 走 `/pose`，20 Hz，**不要**经过大模型；
  手套现在由服务自己收（`/glove/*`，见 README 第 5 节「数据手套」），不用再单跑桥接；
- 想要"边说边动"，把语言切成短句（"张开" / "握"），每句独立执行，体感就顺了。

## 7. 安全底线（写 AI 的人也要守）

- 只调白名单技能，不要自己拼 `/pose` 的角度去做没验证过的动作；
- 第一次跑新策略，**永远先 `dry_run`**，看 `explain` 对不对；
- 演示时电流 0.3–0.5 A，速度 ≤120；
- 有人在设备旁边时，手边留一个 `/stop`（网页上就是"停止"按钮）；
- 长序列（>8 秒）会被拒——这是故意的。

## 8. 调试清单

| 现象 | 查什么 |
| --- | --- |
| `/ai` 返回 422 | `error` 里写着"没听懂"，换个说法；先 `/ai/skills` 看白名单 |
| 计划对但手不动 | 是不是有动作在播放（返回里有提示）；`/state` 看角度有没有变 |
| 角度不对 | `dry_run` 看 `explain`，再看 `/limits` 的行程 |
| 模型总乱输出 | 降温度、把 `skills.schema()` 塞进 prompt、加"只能输出 JSON" |
| 想加新技能 | 在 `handlink/skills.py` 的 `POSE_LIB` 加姿态，`DESCRIPTIONS` 加说明，重启即可 |

## 9. 一句话

**AI 负责"想做什么"，连接器负责"能不能做、怎么做才安全"。** 这条线划清楚，社员可以随便换模型、
换玩法，设备不会因此受伤。
