"""麦克风：列设备、看电平、录一段 WAV。

USB 摄像头大多自带麦克风（本机是 `USB2.0 MIC`），这个模块让「声音」和
「画面」一样能通过 HTTP 拿到：

* **列设备**：``list_inputs()`` 走 Windows 的 ``winmm``（``waveIn*``），
  不需要 sounddevice / pyaudio 这类第三方包。
* **看电平**：录制线程每 50 ms 收一块，算 RMS / 峰值，网页上的电平条就是它。
* **录一段**：``MicRecorder.start()`` 开录，``stop()`` 落成 16 kHz 单声道
  WAV（``wave`` 模块自己写头，时长/采样率都对）。

没有麦克风、被占用、非 Windows 时都是「返回错误 / 空列表」，不抛给调用方去
猜——服务里 GET 一下 /audio/devices 就知道这台机器能不能用。
"""
from __future__ import annotations

import ctypes
import math
import os
import threading
import time
import wave
from array import array
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------- 常量

IS_WINDOWS = os.name == "nt"

WHDR_DONE = 0x00000001
WAVE_FORMAT_PCM = 1
CALLBACK_NULL = 0x00000000

BLOCK_SECONDS = 0.05          # 每块 50 ms：够细的电平，又不至于调用太频繁
BLOCKS = 12                   # 同时在飞 12 块（0.6 s 缓冲）
MAX_SECONDS = 120             # 单次录制上限，防止忘了停把内存吃光
DEFAULT_RATE = 16000
DEFAULT_CHANNELS = 1

MMSYSERR = {
    1: "设备 ID 越界（没有这个 index）",
    2: "驱动没装好（MMSYSERR_NODRIVER）",
    4: "内存不足（MMSYSERR_NOMEM）",
    5: "不支持这种参数（MMSYSERR_NOTSUPPORTED）",
    6: "设备不存在（MMSYSERR_NODEVICE）",
    7: "设备被占用（MMSYSERR_ALLOCATED）",
    8: "已经打开过（MMSYSERR_INVALHANDLE）",
    11: "参数非法（MMSYSERR_INVALPARAM）",
    33: "不支持这种格式（WAVERR_BADFORMAT）",
    34: "还没准备好（WAVERR_STILLPLAYING）",
    35: "缓冲区没排好队（WAVERR_UNPREPARED）",
}


def _err(code: int, what: str) -> RuntimeError:
    hint = MMSYSERR.get(int(code), f"错误码 {code}")
    return RuntimeError(f"{what}失败：{hint}")


# ---------------------------------------------------------------- 结构体

class WAVEFORMATEX(ctypes.Structure):
    _fields_ = [
        ("wFormatTag", ctypes.c_ushort),
        ("nChannels", ctypes.c_ushort),
        ("nSamplesPerSec", ctypes.c_uint32),
        ("nAvgBytesPerSec", ctypes.c_uint32),
        ("nBlockAlign", ctypes.c_ushort),
        ("wBitsPerSample", ctypes.c_ushort),
        ("cbSize", ctypes.c_ushort),
    ]


class WAVEHDR(ctypes.Structure):
    pass


WAVEHDR._fields_ = [
    ("lpData", ctypes.c_void_p),
    ("dwBufferLength", ctypes.c_uint32),
    ("dwBytesRecorded", ctypes.c_uint32),
    ("dwUser", ctypes.c_size_t),
    ("dwFlags", ctypes.c_uint32),
    ("dwLoops", ctypes.c_uint32),
    ("lpNext", ctypes.POINTER(WAVEHDR)),
    ("reserved", ctypes.c_size_t),
]


class WAVEINCAPSW(ctypes.Structure):
    _fields_ = [
        ("wMid", ctypes.c_ushort),
        ("wPid", ctypes.c_ushort),
        ("vDriverVersion", ctypes.c_uint),
        ("szPname", ctypes.c_wchar * 32),
        ("dwFormats", ctypes.c_uint32),
        ("wChannels", ctypes.c_ushort),
        ("wReserved1", ctypes.c_ushort),
    ]


_winmm = None


def _mm():
    """winmm 句柄（只在 Windows 上存在）。"""
    global _winmm
    if not IS_WINDOWS:
        raise RuntimeError("只有 Windows 有 winmm：这个模块目前只能在 Windows 上录音")
    if _winmm is None:
        mm = ctypes.WinDLL("winmm")
        mm.waveInGetNumDevs.restype = ctypes.c_uint
        mm.waveInGetNumDevs.argtypes = []
        mm.waveInGetDevCapsW.restype = ctypes.c_uint
        mm.waveInGetDevCapsW.argtypes = [ctypes.c_size_t, ctypes.POINTER(WAVEINCAPSW), ctypes.c_uint]
        mm.waveInOpen.restype = ctypes.c_uint
        mm.waveInOpen.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint,
                                  ctypes.POINTER(WAVEFORMATEX), ctypes.c_size_t,
                                  ctypes.c_size_t, ctypes.c_uint]
        mm.waveInClose.restype = ctypes.c_uint
        mm.waveInClose.argtypes = [ctypes.c_void_p]
        mm.waveInPrepareHeader.restype = ctypes.c_uint
        mm.waveInPrepareHeader.argtypes = [ctypes.c_void_p, ctypes.POINTER(WAVEHDR), ctypes.c_uint]
        mm.waveInUnprepareHeader.restype = ctypes.c_uint
        mm.waveInUnprepareHeader.argtypes = [ctypes.c_void_p, ctypes.POINTER(WAVEHDR), ctypes.c_uint]
        mm.waveInAddBuffer.restype = ctypes.c_uint
        mm.waveInAddBuffer.argtypes = [ctypes.c_void_p, ctypes.POINTER(WAVEHDR), ctypes.c_uint]
        mm.waveInStart.restype = ctypes.c_uint
        mm.waveInStart.argtypes = [ctypes.c_void_p]
        mm.waveInStop.restype = ctypes.c_uint
        mm.waveInStop.argtypes = [ctypes.c_void_p]
        mm.waveInReset.restype = ctypes.c_uint
        mm.waveInReset.argtypes = [ctypes.c_void_p]
        _winmm = mm
    return _winmm


# ---------------------------------------------------------------- 设备

@dataclass
class InputDevice:
    index: int
    name: str
    channels: int
    formats: int

    def as_dict(self) -> Dict[str, Any]:
        return {"index": self.index, "name": self.name,
                "channels": self.channels, "formats": hex(self.formats)}


def list_inputs() -> List[InputDevice]:
    """录音输入设备（winmm 的枚举顺序，就是 ``start(index=...)`` 用的号）。"""
    if not IS_WINDOWS:
        return []
    try:
        mm = _mm()
    except OSError:
        return []
    out: List[InputDevice] = []
    for i in range(int(mm.waveInGetNumDevs())):
        caps = WAVEINCAPSW()
        if mm.waveInGetDevCapsW(i, ctypes.byref(caps), ctypes.sizeof(caps)) != 0:
            continue
        out.append(InputDevice(index=i, name=caps.szPname.strip() or f"输入设备 {i}",
                               channels=int(caps.wChannels), formats=int(caps.dwFormats)))
    return out


# ---------------------------------------------------------------- 电平

def block_rms(data: bytes) -> Tuple[float, float]:
    """一块 PCM16 的 (RMS, 峰值)，都归一化到 0～1。"""
    samples = array("h")
    usable = len(data) - (len(data) % 2)
    if usable <= 0:
        return 0.0, 0.0
    samples.frombytes(data[:usable])
    if not samples:
        return 0.0, 0.0
    total = 0
    peak = 0
    for s in samples:
        v = s if s >= 0 else -s
        total += int(s) * int(s)
        if v > peak:
            peak = v
    rms = math.sqrt(total / len(samples)) / 32768.0
    return rms, peak / 32768.0


def to_db(level: float) -> float:
    """线性幅度 → dBFS；0 给一个地板值，免得页面显示 -inf。"""
    return round(max(-90.0, 20.0 * math.log10(level)), 1) if level > 0 else -90.0


def to_meter(db: float) -> float:
    """dBFS → 0～1 的条，-60 dB 以下算 0。"""
    return round(max(0.0, min(1.0, (float(db) + 60.0) / 60.0)), 3)


# ---------------------------------------------------------------- 录音

class MicRecorder:
    """一路麦克风的录音线程 + 电平。同一时间只录一路。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._handle: Optional[ctypes.c_void_p] = None
        self._headers: List[Tuple[ctypes.Array, WAVEHDR]] = []
        self._wf: Optional[wave.Wave_write] = None
        self._path: Optional[Path] = None
        self._started_at = 0.0
        self._device = 0
        self._rate = DEFAULT_RATE
        self._channels = DEFAULT_CHANNELS
        self._max_seconds = 30.0
        self._frames = 0
        self._bytes = 0
        self._peak = 0.0
        self._level = 0.0
        self._level_db = -90.0
        self._peak_db = -90.0
        self._errors = 0
        self._last_error = ""
        self._last: Dict[str, Any] = {}

    # -- 状态 -------------------------------------------------------------
    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    @property
    def path(self) -> Optional[Path]:
        return self._path

    def seconds(self) -> float:
        align = max(1, self._channels * 2)
        return round(self._bytes / align / self._rate, 3)

    def state(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "running": self.running,
            "device": self._device,
            "rate": self._rate,
            "channels": self._channels,
            "seconds": self.seconds(),
            "frames": self._frames,
            "level": to_meter(self._level_db),
            "level_db": round(self._level_db, 1),
            "peak_db": round(self._peak_db, 1),
            "path": str(self._path) if self._path else None,
            "max_seconds": self._max_seconds,
            "errors": self._errors,
            "last_error": self._last_error,
            "last": self._last,
        }

    # -- 开录 -------------------------------------------------------------
    def start(self, index: int = 0, rate: int = DEFAULT_RATE, channels: int = DEFAULT_CHANNELS,
              out: Optional[Path] = None, max_seconds: float = 30.0) -> Dict[str, Any]:
        if self.running:
            return {"ok": False, "error": "已经在录了，先 stop()", "path": str(self._path)}
        rate = int(rate)
        channels = int(channels)
        if rate < 8000 or rate > 48000:
            return {"ok": False, "error": "rate 只支持 8000～48000"}
        if channels not in (1, 2):
            return {"ok": False, "error": "channels 只支持 1 或 2"}
        index = max(0, int(index))
        path = Path(out) if out else (Path("recordings") / time.strftime("audio-%Y%m%d-%H%M%S.wav"))
        path.parent.mkdir(parents=True, exist_ok=True)

        mm = _mm()
        fmt = WAVEFORMATEX(
            wFormatTag=WAVE_FORMAT_PCM, nChannels=channels, nSamplesPerSec=rate,
            nAvgBytesPerSec=rate * channels * 2, nBlockAlign=channels * 2,
            wBitsPerSample=16, cbSize=0,
        )
        handle = ctypes.c_void_p()
        rc = mm.waveInOpen(ctypes.byref(handle), index, ctypes.byref(fmt), 0, 0, CALLBACK_NULL)
        if rc != 0:
            raise _err(rc, f"打开录音设备 #{index} ")

        wf = wave.open(str(path), "wb")
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(rate)

        block_bytes = int(rate * BLOCK_SECONDS) * channels * 2
        headers: List[Tuple[ctypes.Array, WAVEHDR]] = []
        try:
            for _ in range(BLOCKS):
                buf = ctypes.create_string_buffer(block_bytes)
                hdr = WAVEHDR(lpData=ctypes.cast(buf, ctypes.c_void_p),
                              dwBufferLength=block_bytes)
                rc = mm.waveInPrepareHeader(handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
                if rc != 0:
                    raise _err(rc, "准备录音缓冲区 ")
                rc = mm.waveInAddBuffer(handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
                if rc != 0:
                    raise _err(rc, "挂载录音缓冲区 ")
                headers.append((buf, hdr))
        except Exception:
            wf.close()
            for _, hdr in headers:
                mm.waveInUnprepareHeader(handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
            mm.waveInClose(handle)
            raise

        with self._lock:
            self._handle = handle
            self._headers = headers
            self._wf = wf
            self._path = path
            self._device = index
            self._rate = rate
            self._channels = channels
            self._max_seconds = max(1.0, min(float(max_seconds), MAX_SECONDS))
            self._frames = 0
            self._bytes = 0
            self._peak = 0.0
            self._level = 0.0
            self._level_db = -90.0
            self._peak_db = -90.0
            self._errors = 0
            self._last_error = ""
            self._started_at = time.time()
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="mic", daemon=True)
            self._thread.start()
        rc = mm.waveInStart(handle)
        if rc != 0:
            self._stop.set()
            thread = self._thread
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
            self._teardown()
            raise _err(rc, "开始录音 ")
        return {"ok": True, **self.state()}

    # -- 采集线程 ---------------------------------------------------------
    def _run(self) -> None:
        mm = _mm()
        handle = self._handle
        while not self._stop.is_set():
            data: List[bytes] = []
            with self._lock:
                if self._handle is None:
                    return
                for buf, hdr in self._headers:
                    if hdr.dwFlags & WHDR_DONE:
                        n = int(hdr.dwBytesRecorded)
                        if n > 0:
                            data.append(ctypes.string_at(hdr.lpData, n))
                        mm.waveInUnprepareHeader(self._handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
                        hdr.dwFlags = 0
                        hdr.dwBytesRecorded = 0
                        mm.waveInPrepareHeader(self._handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
                        mm.waveInAddBuffer(self._handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
                if self.seconds() >= self._max_seconds:
                    self._stop.set()
            for chunk in data:
                self._consume(chunk)
            if self._stop.is_set():
                break
            time.sleep(0.01)
        # 收尾由 stop() 做，这里只退出

    def _consume(self, data: bytes) -> None:
        rms, peak = block_rms(data)
        db = to_db(rms)
        with self._lock:
            self._bytes += len(data)
            self._frames += len(data) // max(1, self._channels * 2)
            self._level = rms
            self._level_db = db
            if peak > self._peak:
                self._peak = peak
            self._peak_db = max(self._peak_db, to_db(peak))
            wf = self._wf
        if wf is not None:
            try:
                wf.writeframesraw(data)
            except Exception as exc:      # 磁盘满 / 文件被删
                with self._lock:
                    self._errors += 1
                    self._last_error = f"写文件失败: {exc}"

    # -- 停录 -------------------------------------------------------------
    def stop(self) -> Dict[str, Any]:
        if not self.running and self._handle is None:
            return {"ok": False, "error": "当前没有在录音"}
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        summary = self._teardown()
        return {"ok": True, **summary}

    def _teardown(self) -> Dict[str, Any]:
        mm = _mm()
        with self._lock:
            handle, self._handle = self._handle, None
            headers, self._headers = self._headers, []
            wf, self._wf = self._wf, None
            seconds = self.seconds()
            peak_db = round(self._peak_db, 1)
            seconds = round(seconds, 3)
            path = self._path
        if handle is not None:
            mm.waveInStop(handle)
            mm.waveInReset(handle)
            for _, hdr in headers:
                mm.waveInUnprepareHeader(handle, ctypes.byref(hdr), ctypes.sizeof(hdr))
            mm.waveInClose(handle)
        if wf is not None:
            try:
                wf.close()
            except Exception:
                pass
        info = {"path": str(path) if path else None,
                "seconds": seconds, "peak_db": peak_db, "rate": self._rate,
                "channels": self._channels, "device": self._device}
        with self._lock:
            self._last = info
            self._level_db = -90.0
            self._level = 0.0
        return info

    # -- 一次性试录 -------------------------------------------------------
    def test(self, index: int = 0, seconds: float = 2.0,
             out: Optional[Path] = None) -> Dict[str, Any]:
        """录一小段然后停掉，用来回答「这个麦到底响不响」。"""
        started = self.start(index=index, out=out, max_seconds=max(1.0, float(seconds)))
        if not started.get("ok"):
            return started
        time.sleep(max(0.2, float(seconds)))
        info = self.stop()
        info["signal"] = "有声音" if info.get("peak_db", -90) > -45 else "基本静音"
        info["level"] = to_meter(info.get("peak_db", -90.0))
        return info


# ---------------------------------------------------------------- 片段

def clips(folder: Path, limit: int = 20) -> List[Dict[str, Any]]:
    """列出目录里录过的 WAV（新的在前）。"""
    folder = Path(folder)
    if not folder.is_dir():
        return []
    items: List[Dict[str, Any]] = []
    for path in sorted(folder.glob("*.wav"), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]:
        try:
            with wave.open(str(path), "rb") as wf:
                seconds = round(wf.getnframes() / float(wf.getframerate() or 1), 2)
                rate = wf.getframerate()
                channels = wf.getnchannels()
        except Exception:
            seconds, rate, channels = 0.0, 0, 0
        items.append({"name": path.name, "path": str(path), "bytes": path.stat().st_size,
                      "seconds": seconds, "rate": rate, "channels": channels,
                      "mtime": round(path.stat().st_mtime, 1)})
    return items
