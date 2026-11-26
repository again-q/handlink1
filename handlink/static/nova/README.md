# Nova 5 原厂网格与碰撞代理

这个目录是 `/arm3d` 用的几何，**不是手画的**：

| 文件 | 来源 | 说明 |
| --- | --- | --- |
| `base_link.STL` … `Link6.STL` | 越疆 `nova5_2f85_description` 的 `meshes/nova5/` | 每段连杆一个网格，坐标系就是 URDF 的 Link 系（米，Z 向上）；螺钉、线槽、关节罩都在 |
| `nova5_2f85.urdf` | 同上（`urdf/nova5_2f85.urdf`） | 只作参考：`handlink/kin.py` 的关节原点就是从它抄来的 |
| `proxies.json` | `scripts/build_nova_assets.py` 生成 | 每段连杆的胶囊链（P90 径向、外扩 2%），安全距离用 |

来源仓库：<https://github.com/Cyber-physical-Systems-Lab/Nova_ROS2>
（`nova5_2f85_description`，MIT）。原 URDF 里还带 Robotiq 2F-85 夹爪的网格，
这台机器末端装的是 LG-603 灵巧手，所以夹爪那部分没有放进来。

## 重新生成代理

```bash
python scripts/build_nova_assets.py
```

只依赖标准库。改了 STL 或想调保守程度（`bin_mm` / `percentile_q`）之后重跑，
`/arm3d` 和 `/arm/check` 会同时生效——两边读的是同一个 JSON。
