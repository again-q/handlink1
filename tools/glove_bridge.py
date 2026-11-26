"""数据手套桥接（独立跑法）—— 逻辑已经并入 ``handlink/glove.py``。

插件里**不需要**这个脚本：服务自己就会收 UDP 5001，页面上的「数据手套」卡片
直接启动/停止监听、切手套/代码模式、重新零点。这里只是保留一条「服务没起、
或者想转发给别的程序」时还能用的命令行入口，参数和原来的桥接一致：

    python tools/glove_bridge.py --http http://127.0.0.1:8766
    python tools/glove_bridge.py --http http://127.0.0.1:8766 --gain 1.2 --side left
    python tools/glove_bridge.py --dry-run --verbose

实现见 ``handlink/glove.py``：帧解析、关节链求和、自动零点、速率上限都在那里，
``tests/test_glove.py`` 不接硬件也能跑。
"""
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from handlink.glove import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
