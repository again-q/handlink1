"""硬件接入（实验性）：串口、ZLG CAN、环境探测。

⚠️ 说明
-------
本机目前没有接机械手（没有串口、没有 CAN 设备、LG-603 也没安装），所以这里
的串口与 CAN 代码**没有在真机上跑过**。它们实现的是 Windows 标准调用和
ZLG 官方 VCI 接口，接上硬件后先跑 ``python -m handlink scan`` 与
``python -m handlink canmon`` 验证，再谈控制。

推荐的上手顺序见 README：先「旁听」再「接管」。
"""
from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import json
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from . import spec


# ---------------------------------------------------------------- 环境探测

def list_com_ports() -> List[Dict[str, str]]:
    """从注册表读串口（等价于设备管理器里的 COM 口）。"""
    ports: List[Dict[str, str]] = []
    if os.name != "nt":
        return ports
    try:
        import winreg

        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            i = 0
            while True:
                try:
                    name, value, _ = winreg.EnumValue(key, i)
                except OSError:
                    break
                ports.append({"device": str(value), "source": name})
                i += 1
    except OSError:
        pass
    return sorted(ports, key=lambda p: p["device"])


def probe_tcp(host: str, port: int, timeout: float = 0.4) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def find_vendor_root(start: Optional[Path] = None) -> Optional[Path]:
    """找 LG-603 软件包根目录。"""
    from . import store

    return store.find_root(start)


def find_can_dll(root: Optional[Path] = None) -> Optional[Path]:
    rel = Path("src/main/resources/native/Windows-x86_64/ECanVci64.dll")
    for base in filter(None, [root, find_vendor_root()]):
        p = Path(base)
        for cand in (p / rel, p / "LG-603" / rel, p.parent / rel):
            if cand.is_file():
                return cand
    return None


def scan() -> Dict[str, Any]:
    """一次把本机能看到的硬件/软件情况摸清楚。"""
    from . import camera as cam_mod

    root = find_vendor_root()
    dll = find_can_dll(root)
    result: Dict[str, Any] = {
        "os": os.name,
        "com_ports": list_com_ports(),
        "vendor_root": str(root) if root else None,
        "can_dll": str(dll) if dll else None,
        "cameras": cam_mod.list_devices(),
        "services": {},
        "notes": [],
    }
    for host, port, label in (
        ("127.0.0.1", 30603, "HandRspFastAPI(视觉推理)"),
        ("127.0.0.1", 5666, "LG-603 远程服务配置端口"),
    ):
        result["services"][f"{host}:{port}"] = {"label": label, "open": probe_tcp(host, port)}

    if not result["com_ports"]:
        result["notes"].append("没有发现串口设备（USB-串口 / 机械手未插？）")
    if not dll:
        result["notes"].append("没找到 ECanVci64.dll，无法做 CAN 旁听")
    else:
        dev = probe_can(dll)
        result["can_devices"] = dev
        if not dev.get("opened"):
            result["notes"].append("CAN 适配器没有应答（未插 ZLG USBCAN，或被 LG-603 占用）")
    return result


# ---------------------------------------------------------------- ZLG CAN

class _VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", wt.DWORD),
        ("AccMask", wt.DWORD),
        ("Reserved", wt.DWORD),
        ("Filter", wt.BYTE),
        ("Timing0", wt.BYTE),
        ("Timing1", wt.BYTE),
        ("Mode", wt.BYTE),
    ]


class _VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", wt.DWORD),
        ("TimeStamp", wt.DWORD),
        ("TimeFlag", wt.BYTE),
        ("SendType", wt.BYTE),
        ("RemoteFlag", wt.BYTE),
        ("ExternFlag", wt.BYTE),
        ("DataLen", wt.BYTE),
        ("Data", wt.BYTE * 8),
        ("Reserved", wt.BYTE * 3),
    ]


# 16MHz SJA1000 常用波特率（ZLG 手册表）
TIMING = {
    1000: (0x00, 0x14),
    500: (0x00, 0x1C),
    250: (0x01, 0x1C),
    125: (0x03, 0x1C),
}

DEVICE_TYPES = {"USBCAN-2E-U": 21, "USBCAN-E-U": 20}


class ZlgCan:
    """ZLG USBCAN 的最薄封装：打开、收、发、关。

    只用来「旁听」总线时不要调用 :meth:`send`。同一条 CAN 通道同一时间只能被
    一个进程打开，所以旁听要接在第二路通道/第二个适配器上。
    """

    def __init__(
        self,
        dll_path: Path,
        device_type: int = 21,
        device_index: int = 0,
        channel: int = 1,
        baud: int = 1000,
    ) -> None:
        self.dll_path = str(dll_path)
        self.device_type = int(device_type)
        self.device_index = int(device_index)
        self.channel = int(channel)
        self.baud = int(baud)
        self._dll: Any = None
        self._open = False

    def _lib(self):
        if self._dll is None:
            if os.name != "nt":
                raise RuntimeError("ZLG CAN 只能在 Windows 上使用")
            os.add_dll_directory(str(Path(self.dll_path).parent))
            self._dll = ctypes.WinDLL(self.dll_path)
        return self._dll

    def _fn(self, *names: str):
        """ZLG 各版本 DLL 导出名不一致：ControlCAN.dll 带 VCI_ 前缀，ECanVci64.dll 不带。"""
        dll = self._lib()
        for name in names:
            try:
                return getattr(dll, name)
            except AttributeError:
                continue
        raise AttributeError(f"{Path(self.dll_path).name} 未导出 {' / '.join(names)}")

    def open(self) -> bool:
        if self._fn("VCI_OpenDevice", "OpenDevice")(self.device_type, self.device_index, 0) != 1:
            raise RuntimeError("VCI_OpenDevice 失败：适配器未插好或被占用")
        cfg = _VCI_INIT_CONFIG(0, 0xFFFFFFFF, 0, 0, *TIMING[self.baud], 0)
        ok = self._fn("VCI_InitCAN", "InitCAN")(
            self.device_type, self.device_index, self.channel, ctypes.byref(cfg)
        )
        if ok != 1:
            self._fn("VCI_CloseDevice", "CloseDevice")(self.device_type, self.device_index)
            raise RuntimeError(f"VCI_InitCAN 失败（通道 {self.channel}, {self.baud} kbps）")
        if self._fn("VCI_StartCAN", "StartCAN")(self.device_type, self.device_index, self.channel) != 1:
            self._fn("VCI_CloseDevice", "CloseDevice")(self.device_type, self.device_index)
            raise RuntimeError("VCI_StartCAN 失败")
        self._open = True
        return True

    def close(self) -> None:
        if self._dll and self._open:
            self._fn("VCI_ResetCAN", "ResetCAN")(self.device_type, self.device_index, self.channel)
            self._fn("VCI_CloseDevice", "CloseDevice")(self.device_type, self.device_index)
        self._open = False

    def read(self, max_frames: int = 64, wait_ms: int = 10) -> List[Dict[str, Any]]:
        if not self._open:
            raise RuntimeError("CAN 未打开")
        buf = (_VCI_CAN_OBJ * max_frames)()
        n = self._fn("VCI_Receive", "Receive")(
            self.device_type, self.device_index, self.channel, buf, max_frames, wait_ms
        )
        if n <= 0:
            return []
        frames = []
        for i in range(n):
            o = buf[i]
            frames.append({
                "t": time.time(),
                "id": (o.ID & 0x1FFFFFFF),
                "ext": bool(o.ID & 0x80000000),
                "rtr": bool(o.RemoteFlag),
                "dlc": int(o.DataLen),
                "data": bytes(o.Data[: o.DataLen]),
            })
        return frames

    def send(self, can_id: int, data: bytes, extended: bool = False) -> bool:
        if not self._open:
            raise RuntimeError("CAN 未打开")
        if len(data) > 8:
            raise ValueError("经典 CAN 一帧最多 8 字节")
        obj = _VCI_CAN_OBJ()
        obj.ID = can_id | (0x80000000 if extended else 0)
        obj.ExternFlag = 1 if extended else 0
        obj.DataLen = len(data)
        for i, b in enumerate(data):
            obj.Data[i] = b
        return self._fn("VCI_Transmit", "Transmit")(
            self.device_type, self.device_index, self.channel, ctypes.byref(obj), 1
        ) == 1


def probe_can(dll_path: Path, device_type: int = 21, channel: int = 0) -> Dict[str, Any]:
    """尝试打开适配器然后马上关闭，只为了确认「在不在」。"""
    info: Dict[str, Any] = {"dll": str(dll_path), "opened": False}
    try:
        can = ZlgCan(dll_path, device_type=device_type, channel=channel)
        can.open()
        info["opened"] = True
        can.close()
    except Exception as exc:
        info["error"] = str(exc)
    return info


# ---------------------------------------------------------------- 串口

_GENERIC_READ = 0x80000000
_GENERIC_WRITE = 0x40000000
_OPEN_EXISTING = 3
_MAXDWORD = 0xFFFFFFFF


class _DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD),
        ("BaudRate", wt.DWORD),
        ("flags", wt.DWORD),
        ("wReserved", wt.WORD),
        ("XonLim", wt.WORD),
        ("XoffLim", wt.WORD),
        ("ByteSize", wt.BYTE),
        ("Parity", wt.BYTE),
        ("StopBits", wt.BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wt.WORD),
    ]


class _COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wt.DWORD),
        ("ReadTotalTimeoutMultiplier", wt.DWORD),
        ("ReadTotalTimeoutConstant", wt.DWORD),
        ("WriteTotalTimeoutMultiplier", wt.DWORD),
        ("WriteTotalTimeoutConstant", wt.DWORD),
    ]


class WinSerial:
    """最小可用的 Windows 串口（不依赖 pyserial）。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05) -> None:
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self._h = None
        self._k32 = ctypes.WinDLL("kernel32", use_last_error=True)

    def open(self) -> "WinSerial":
        if os.name != "nt":
            raise RuntimeError("WinSerial 仅支持 Windows")
        name = self.port if self.port.startswith("\\\\.\\") else f"\\\\.\\{self.port}"
        handle = self._k32.CreateFileW(
            name, _GENERIC_READ | _GENERIC_WRITE, 0, None, _OPEN_EXISTING, 0, None
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise OSError(ctypes.get_last_error(), f"打不开串口 {self.port}（被占用或不存在）")
        self._h = handle

        dcb = _DCB()
        dcb.DCBlength = ctypes.sizeof(_DCB)
        dcb.BaudRate = self.baud
        # fBinary=1 | fDtrControl=ENABLE | fRtsControl=ENABLE
        dcb.flags = 0x0001 | (1 << 4) | (1 << 12)
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        if not self._k32.SetCommState(handle, ctypes.byref(dcb)):
            self.close()
            raise OSError(ctypes.get_last_error(), f"SetCommState 失败: {self.port} @ {self.baud}")

        to = _COMMTIMEOUTS(_MAXDWORD, _MAXDWORD, int(self.timeout * 1000), 0, 0)
        self._k32.SetCommTimeouts(handle, ctypes.byref(to))
        self._k32.PurgeComm(handle, 0x000F)
        return self

    def write(self, data: bytes) -> int:
        if not self._h:
            raise RuntimeError("串口未打开")
        written = wt.DWORD(0)
        ok = self._k32.WriteFile(self._h, data, len(data), ctypes.byref(written), None)
        if not ok:
            raise OSError(ctypes.get_last_error(), f"写串口失败: {self.port}")
        return int(written.value)

    def read(self, size: int = 4096) -> bytes:
        if not self._h:
            raise RuntimeError("串口未打开")
        buf = ctypes.create_string_buffer(size)
        got = wt.DWORD(0)
        ok = self._k32.ReadFile(self._h, buf, size, ctypes.byref(got), None)
        if not ok:
            raise OSError(ctypes.get_last_error(), f"读串口失败: {self.port}")
        return buf.raw[: got.value]

    def close(self) -> None:
        if self._h:
            self._k32.CloseHandle(self._h)
            self._h = None

    def __enter__(self) -> "WinSerial":
        return self.open()

    def __exit__(self, *exc: Any) -> None:
        self.close()


class SerialTap:
    """串口「中间人」：接在 com0com 虚拟串口对上，双向转发并原样记录。

    典型接法::

        LG-603  ──COM5──  [虚拟对 COM5<->COM6]  ──COM6──  SerialTap  ──COM7──  机械手

    这样厂家软件以为自己在和手说话，实际所有报文都过了我们的手（可看、可记录、
    后续也可以改写注入）。
    """

    def __init__(self, listen: WinSerial, device: WinSerial, log_path: Optional[Path] = None,
                 baud: int = 115200) -> None:
        self.listen = listen
        self.device = device
        self.log_path = Path(log_path) if log_path else None
        self._stop = threading.Event()
        self._log = None
        self.packets: List[Dict[str, Any]] = []

    def start(self) -> "SerialTap":
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = self.log_path.open("a", encoding="utf-8")
        threading.Thread(target=self._pump, args=(self.listen, self.device, "app->hand"), daemon=True).start()
        threading.Thread(target=self._pump, args=(self.device, self.listen, "hand->app"), daemon=True).start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._log:
            self._log.close()
            self._log = None

    def _pump(self, src: WinSerial, dst: WinSerial, direction: str) -> None:
        while not self._stop.is_set():
            try:
                chunk = src.read()
            except OSError:
                break
            if not chunk:
                continue
            try:
                dst.write(chunk)
            except OSError:
                break
            packet = {
                "t": time.time(),
                "direction": direction,
                "hex": chunk.hex(" "),
                "text": chunk.decode("utf-8", "replace"),
            }
            self.packets.append(packet)
            if self._log:
                self._log.write(json.dumps(packet, ensure_ascii=False) + "\n")
                self._log.flush()
