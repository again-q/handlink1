"""arm3d 页面的 id 自检：重构搬家时最怕「某个 id 搬丢了，按钮静默失效」。

不启浏览器，只做静态扫描：

1. 每个 ``$("xxx")`` 都要有对应的 ``id="xxx"``；
2. id 不能重复（重复时 ``$()`` 只拿到第一个，另一处就永远不更新）；
3. ``/trace`` 路由依赖的 ``</head>`` 必须还在（service 用字符串替换注入）。

    python tests/test_armview_ids.py
"""
from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import armview  # noqa: E402

LOOKUP = re.compile(r'\$\("([A-Za-z0-9_]+)"\)')
ID = re.compile(r'id="([A-Za-z0-9_]+)"')


class TestArmviewIds(unittest.TestCase):
    def setUp(self):
        self.html = armview.HTML

    def test_every_lookup_has_an_element(self):
        lookups = sorted(set(LOOKUP.findall(self.html)))
        ids = set(ID.findall(self.html))
        missing = [name for name in lookups if name not in ids]
        self.assertEqual(missing, [],
                         "页面里找不到这些 id，按钮会静默失效：%s" % missing)

    def test_ids_are_unique(self):
        names = ID.findall(self.html)
        dup = sorted({name for name in names if names.count(name) > 1})
        self.assertEqual(dup, [], "id 重复，$(...) 只会拿到第一个：%s" % dup)

    def test_trace_route_can_still_inject(self):
        # service 里 /trace 是对 HTML 做 </head> 字符串替换实现的
        self.assertIn("</head>", self.html)


if __name__ == "__main__":
    unittest.main(verbosity=2)
