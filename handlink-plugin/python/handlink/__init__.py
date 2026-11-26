"""handlink —— 中科灵犀 LG-603 灵巧手连接器。

分层
----
``spec``      规格、姿态模型、安全检查（不碰硬件）
``store``     点位 / 动作文件读写（与 LG-603 的 points、tracks 完全兼容）
``bus``       总线抽象 + 仿真手 + 动作播放
``hardware``  串口 / ZLG CAN / 环境探测（实验性，需真机验证）
``recorder``  JSONL 录制与离线分析
``service``   本地 HTTP JSON 接口
``cli``       命令行
"""
__version__ = "0.1.0"

from . import bus, hardware, recorder, spec, store  # noqa: F401

__all__ = ["spec", "store", "bus", "hardware", "recorder", "service", "__version__"]


def __getattr__(name: str):
    """``service`` 按需导入。

    service 会连带拉起 camera / scene / glove 一大串模块。``python -m handlink.glove``
    这类「直接跑子模块」的用法，包初始化时就把 glove 导进来，runpy 会警告
    "found in sys.modules after import of package"。改成访问时才导入，两边都干净。
    """
    if name == "service":
        from importlib import import_module

        return import_module(".service", __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
