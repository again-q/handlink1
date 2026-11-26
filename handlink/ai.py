"""自然语言 → 技能计划 → 执行。

两个规划器：

* :class:`RulePlanner` —— 离线、零延迟，靠关键词把中文拆成技能。没有模型也能用。
* :class:`LLMPlanner`  —— 走 OpenAI 兼容接口（DeepSeek / 通义 / 本地 Ollama 都行），
  模型只输出**技能 JSON**，不碰角度；解析失败自动退回规则规划器。

实时的边界：语言理解 1–3 秒（大模型）/ 几毫秒（规则），动作执行按每步 delay。
要做 20 Hz 的连续跟随（手套、摄像头），走 ``/pose``，别走这里——语言不是帧率。
"""
from __future__ import annotations

import json
import os
import re
import time
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from . import skills, spec

FINGER_WORDS = [
    (("拇指1", "拇指一", "拇指"), 102),
    (("拇指2", "拇指二"), 107),
    (("食指",), 103),
    (("中指",), 104),
    (("无名指", "无名"), 105),
    (("小指", "尾指"), 106),
]

SPLIT_RE = re.compile(r"然后|接着|之后|再|，|,|；|;|。|、|\n")
NUM_RE = re.compile(r"(\d+(?:\.\d+)?)\s*(度|°|ms|毫秒|秒|s)?", re.I)
CN_NUM = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
          "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def duration_ms(text: str) -> Optional[int]:
    """「两秒」「1.5 秒」「800 毫秒」→ 毫秒。"""
    m = NUM_RE.search(text)
    if m:
        v = float(m.group(1))
        unit = (m.group(2) or "").lower()
        return int(v * 1000) if unit in ("秒", "s") else int(v)
    for cn, val in CN_NUM.items():
        if f"{cn}秒" in text:
            return val * 1000
        if f"{cn}毫秒" in text:
            return val
    return None


@dataclass
class Step:
    skill: str
    args: Dict[str, Any] = field(default_factory=dict)
    delay_ms: int = 500
    note: str = ""


@dataclass
class Plan:
    text: str
    steps: List[Step]
    source: str = "rule"
    note: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "text": self.text,
            "source": self.source,
            "note": self.note,
            "steps": [{"skill": s.skill, "args": s.args, "delay_ms": s.delay_ms, "note": s.note}
                      for s in self.steps],
        }


# ---------------------------------------------------------------- 规则规划

def _speed_hint(text: str) -> Dict[str, float]:
    if any(w in text for w in ("轻轻", "慢", "温柔", "轻一点")):
        return {"speed": 40.0, "torque": 0.2}
    if any(w in text for w in ("快", "迅速", "马上")):
        return {"speed": 120.0, "torque": 0.4}
    return {}


def _fingers_in(text: str) -> List[int]:
    found: List[int] = []
    for words, mid in FINGER_WORDS:
        if any(w in text for w in words):
            found.append(mid)
    return found


class RulePlanner:
    """关键词规划器：没网、没模型也能"听懂"常见指令。"""

    name = "rule"

    def plan(self, text: str) -> Plan:
        segments = [s.strip() for s in SPLIT_RE.split(text) if s.strip()]
        steps: List[Step] = []
        hint = _speed_hint(text)

        for seg in segments:
            step = self._one(seg, hint)
            if step:
                steps.append(step)

        if not steps:
            raise ValueError(f"没听懂：{text!r}。可以说「张开」「握拳」「捏住」「剪刀」「食指 90 度」")

        return Plan(text=text, steps=steps, source=self.name)

    def _one(self, seg: str, hint: Dict[str, float]) -> Optional[Step]:
        args: Dict[str, Any] = dict(hint)

        # 保持 N 秒
        if any(w in seg for w in ("保持", "停住", "维持", "不动")):
            ms = duration_ms(seg) or 1000
            ms = max(0, min(ms, skills.MAX_TOTAL_MS))
            args["ms"] = ms
            return Step("hold", args, ms, f"保持 {ms} ms")

        # 指定角度：食指 90 度
        if "度" in seg or "°" in seg:
            fingers = _fingers_in(seg)
            m = NUM_RE.search(seg)
            if fingers and m:
                angles = {mid: float(m.group(1)) for mid in fingers}
                args["angles"] = angles
                names = "、".join(spec.MOTOR_BY_ID[f].name for f in fingers)
                return Step("set_fingers", args, 400, f"{names} → {m.group(1)}°")

        if any(w in seg for w in ("招手", "挥手", "挥一", "拜拜", "再见", "打招呼")):
            return Step("wave", {**args, "cycles": 2}, 700, "招手（五指开合）")
        if any(w in seg for w in ("勾手", "勾一勾", "勾勾", "过来", "招财猫")):
            return Step("beckon", {**args, "cycles": 3}, 600, "勾手（食指反复勾动）")

        if any(w in seg for w in ("捏", "夹")):
            skill = "tripod" if any(w in seg for w in ("三", "中", "三指")) else "pinch"
            return Step(skill, args, 500, skills.DESCRIPTIONS[skill])

        if "剪刀" in seg:
            return Step("scissors", args, 500, "剪刀")
        if any(w in seg for w in ("布", "张开", "打开", "摊开", "伸直", "张开手")):
            return Step("open", args, 500, "张开")
        if any(w in seg for w in ("握", "抓", "拳", "石头", "闭合", "合上")):
            return Step("close", args, 500, "握拳")
        if any(w in seg for w in ("指", "点一下", "指向前")):
            return Step("point", args, 500, "食指伸出")
        if any(w in seg for w in ("停", "急停", "松开")):
            return Step("open", args, 300, "松开")

        # 纯数字：当成「弯到这个角度」的全指动作
        m = NUM_RE.search(seg)
        if m and not _fingers_in(seg):
            angle = float(m.group(1))
            if 0 <= angle <= 180:
                args["angles"] = {mid: angle for mid in spec.MOTOR_IDS}
                return Step("set_fingers", args, 400, f"全指 → {angle:.0f}°")
        return None


# ---------------------------------------------------------------- 大模型规划

PROMPT = """你是灵巧手控制器的规划器。把用户的中文需求翻译成技能序列。

只能使用这些技能：
{skills}

输出严格 JSON（不要解释、不要 markdown）：
{{"steps": [{{"skill": "open", "args": {{}}, "delay_ms": 500}}]}}

规则：
- 最多 {max_steps} 步，总时长不超过 {max_ms} ms
- angles 里的键用手指名：拇指1/拇指2/食指/中指/无名指/小指
- 不确定就少做几步；不要发明技能
"""


class LLMPlanner:
    """OpenAI 兼容接口。环境变量：HANDLINK_LLM_BASE / KEY / MODEL。"""

    name = "llm"

    def __init__(self, base: Optional[str] = None, key: Optional[str] = None,
                 model: Optional[str] = None, timeout: float = 20.0) -> None:
        self.base = (base or os.environ.get("HANDLINK_LLM_BASE") or "https://api.deepseek.com/v1").rstrip("/")
        self.key = key or os.environ.get("HANDLINK_LLM_KEY") or ""
        self.model = model or os.environ.get("HANDLINK_LLM_MODEL") or "deepseek-chat"
        self.timeout = timeout

    @property
    def available(self) -> bool:
        return bool(self.key)

    def plan(self, text: str) -> Plan:
        if not self.available:
            raise RuntimeError("没有配置 HANDLINK_LLM_KEY")
        skill_text = "\n".join(
            f"- {s['skill']}: {s['description']}" for s in skills.schema()
        )
        system = PROMPT.format(skills=skill_text, max_steps=skills.MAX_STEPS,
                               max_ms=skills.MAX_TOTAL_MS)
        body = json.dumps({
            "model": self.model,
            "messages": [{"role": "system", "content": system},
                         {"role": "user", "content": text}],
            "temperature": 0.2,
            "response_format": {"type": "json_object"},
        }).encode()
        req = urllib.request.Request(
            f"{self.base}/chat/completions", data=body, method="POST",
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self.key}"},
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            data = json.loads(resp.read().decode())
        content = data["choices"][0]["message"]["content"]
        payload = json.loads(self._strip(content))
        steps = []
        for item in payload.get("steps", []):
            skill = item.get("skill", "")
            if skill not in skills.DESCRIPTIONS:
                raise ValueError(f"模型给了未知技能 {skill!r}")
            steps.append(Step(skill, item.get("args") or {}, int(item.get("delay_ms", 500)),
                              skills.DESCRIPTIONS.get(skill, skill)))
        if not steps:
            raise ValueError("模型没给步骤")
        return Plan(text=text, steps=steps, source=self.name)

    @staticmethod
    def _strip(content: str) -> str:
        content = content.strip()
        if content.startswith("```"):
            content = re.sub(r"^```[a-zA-Z]*\s*", "", content)
            content = re.sub(r"\s*```$", "", content)
        return content


def make_plan(text: str, prefer_llm: bool = False,
              planner: Optional[Any] = None) -> Plan:
    """优先用模型；失败或无配置就退回规则。返回的 Plan 一定可用。"""
    rule = RulePlanner()
    if planner is not None:
        return planner.plan(text)
    if prefer_llm:
        llm = LLMPlanner()
        if llm.available:
            try:
                return llm.plan(text)
            except Exception as exc:  # 网络/解析失败都退回规则
                plan = rule.plan(text)
                plan.note = f"大模型失败（{exc}），已退回规则"
                return plan
    return rule.plan(text)


# ---------------------------------------------------------------- 执行

class Executor:
    """把计划变成动作。所有下发都走 apply_fn（也就是服务里的安全检查）。"""

    def __init__(self, apply_fn: Callable[[spec.Pose, str], Dict[str, Any]],
                 stop_event: Optional[Any] = None) -> None:
        self.apply_fn = apply_fn
        self.stop_event = stop_event
        self.log: List[Dict[str, Any]] = []

    def run(self, plan: Plan, speed_scale: float = 1.0) -> Dict[str, Any]:
        scale = max(0.05, float(speed_scale))
        started = time.time()
        total = 0
        self.log = []
        for i, step in enumerate(plan.steps, 1):
            if self.stop_event is not None and self.stop_event.is_set():
                return {"ok": False, "stopped": True, "executed": len(self.log)}
            try:
                expanded = skills.steps_for(step.skill, step.args)
            except Exception as exc:
                return {"ok": False, "error": f"第 {i} 步无法展开: {exc}", "executed": len(self.log)}

            issues = skills.validate(expanded)
            errors = [x for x in issues if x["level"] == "error"]
            if errors:
                return {"ok": False, "error": errors[0]["message"], "executed": len(self.log)}

            for sub in expanded:
                if sub["pose"]:
                    result = self.apply_fn(sub["pose"], f"ai:{step.skill}")
                    if not result.get("ok", True):
                        return {"ok": False, "error": result.get("error", "下发被拒"),
                                "issues": result.get("issues", []), "executed": len(self.log)}
                delay = max(0.0, float(sub.get("delay_ms", 0)) / 1000.0 / scale)
                total += int(sub.get("delay_ms", 0))
                if total > skills.MAX_TOTAL_MS:
                    return {"ok": False, "error": "总时长超出上限", "executed": len(self.log)}
                if delay:
                    end = time.time() + delay
                    while time.time() < end:
                        if self.stop_event is not None and self.stop_event.is_set():
                            return {"ok": False, "stopped": True, "executed": len(self.log)}
                        time.sleep(min(0.02, max(0.0, end - time.time())))
            self.log.append({"step": i, "skill": step.skill, "note": step.note})

        return {"ok": True, "executed": len(self.log), "elapsed_ms": int((time.time() - started) * 1000),
                "log": self.log}


def explain(plan: Plan) -> str:
    return " → ".join(f"{s.note or s.skill}({s.delay_ms}ms)" for s in plan.steps)
