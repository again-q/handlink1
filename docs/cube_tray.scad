// cube_tray.scad —— 单手推层用的浅托盘（只卡住魔方最下层）
//
// 用法：改下面参数 → Render (F6) → 导出 STL。
// 说明：
//   * 内口 57.7 mm（57 + 0.35×2），深 19 mm，只卡 D 层；
//   * 左/右两片夹壁，前/后开口，给手指/拇指让位；
//   * 底部 1.5 mm 凹槽，贴硅胶垫；夹壁内侧可再贴 0.5 mm 硅胶；
//   * 打印后把夹壁上缘用砂纸倒 1–2 mm 角，方便魔方落进去。
//
// 打印：PETG/PLA，4 圈壁厚，30% 填充；底面朝下，不用支撑。

cube   = 57.0;   // 魔方边长
gap    = 0.35;   // 单边间隙
layer  = 19.0;   // 一层高度（只卡最下层）
wall   = 5.0;    // 夹壁厚度
base_t = 6.0;    // 底板厚度
pad_t  = 1.5;    // 硅胶垫厚度（底部凹槽）
plate  = 100.0;  // 底板边长

inner = cube + 2 * gap;          // 57.7
outer = inner + 2 * wall;        // 67.7

module jaw(side = 1) {
    // side = 1 右夹壁，-1 左夹壁
    x = side > 0 ? inner / 2 : -inner / 2 - wall;
    translate([x, -inner / 2, base_t])
        cube([wall, inner, layer]);
}

difference() {
    union() {
        // 底板
        translate([-plate / 2, -plate / 2, 0])
            cube([plate, plate, base_t]);
        // 左/右夹壁（前/后不封口）
        jaw(1);
        jaw(-1);
    }

    // 底部硅胶垫凹槽（57.7 × 57.7 × 1.5）
    translate([-inner / 2, -inner / 2, base_t - pad_t])
        cube([inner, inner, pad_t + 0.01]);

    // 4 × M4 固定孔
    for (x = [-40, 40], y = [-40, 40])
        translate([x, y, -1])
            cylinder(d = 4.5, h = base_t + 2, $fn = 24);
}

// 夹壁内侧先不加东西；如果推层时魔方打滑，再贴 0.2 mm 防滑胶带
// （槽口单边间隙只有 0.35 mm，贴厚了魔方放不进去）。
