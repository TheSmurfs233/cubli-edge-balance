# 版本备份 2026-11-11 · 站稳版（Code=89848）

本目录保存的是**第一个"能稳稳站住"的可回退固件**，供以后对照与回滚。

## 身份

| 项 | 值 |
|---|---|
| Keil 构建号 | `Code=89848` `RO-data=4156` `RW-data=804` `ZI-data=14596` |
| hex 生成时间 | 2026-11-11 23:29:00 |
| hex 大小 | 264 750 B |
| 源码提交 | `704e1fe fix: 起立时恢复姿态零点重标定，回到 11-07 能站的语义（Code=89848）` |
| 板上 `ver` 自报 | `build = Nov 11 2026 23;28;58 #1` |
| 文件 | `cubli.hex`（本目录） |

烧录：Keil 打开 `MDK-ARM\cubli.uvprojx` → Download，或用本目录 `cubli.hex`。

## 关键参数（`User/Cfg/board_cfg.h`）

| 宏 | 值 | 说明 |
|---|---|---|
| `ATT_OFFSET_DEG` | 44.25 | 仅影响空闲读数与武装窗；运行中来一次 engage 就重标 |
| `BAL_ARM_DEG` | 10.0 | 起立门，放宽后手扶姿态不会被拒 |
| `BAL_TRIM_MAX_DEG` | 12.0 | 参考角钳位，够得着 ±6° 的真实平衡点 |
| `BAL_TRIM_W_K` | 5.0e-5 | 零点游走增益（deg per rpm per second） |
| `BAL_WALK_DIV` / `BAL_WALK_DT_S` | 100 / 0.5 | 200 Hz 姿态帧 → 2 Hz 走一次 |
| `BAL_WALK_FILT_TAU_S` | 8.0 | 走法输入的一阶低通（见"已知限制"） |
| `BAL_WALK_DEADBAND_RPM` | 40.0 | 低于此轮速不走 |
| `BAL_WALK_PHI_GATE_DEG` | 2.0 | 回路误差超过此值冻结（防摔倒时乱走） |
| `BAL_HELD_DPS` / `MS` / `PHI_DEG` | 5.0 / 150 / 0.30 | "被扶着"判据 |
| `BAL_FF_MNM` / `BAL_FF_DEADBAND_RPM` | 2.05 / 40 | 摩擦前馈（带内线性，跨零连续） |

## 起立语义（这一版的核心）

1. 短按按钮 → 300 ms 后接管；
2. **接管时 `att_zero()`：把手上的姿态设为 0°**（这是 11-07 能站的原因，`app_main.c` 里 `app_balance_start_now()`）；
3. 起立第一帧参考角 = 0（零点刚标定，0 就是姿态）；
4. 之后零点游走自动把参考角挪到真实平衡点（2 Hz，见上表参数）。

## 实测（2026-11-11 23:30–23:36，COM13 只读 + 24 通道 wave）

| 条件 | `phi` 峰峰 | `wheel` | `tau` 峰峰 | 饱和 |
|---|---|---|---|---|
| 走法开（5e-5） | **2.638°**（−1.87…+0.77） | **±200…230 rpm，每 2 s 换向，周期 ≈4 s** | 14.4 mN·m | 0 帧 |
| `bal walk 0` 冻结后 | **0.314°**（−0.96…−0.64） | +0…+73 rpm，均值 +29.5 | 4.5 mN·m | 0 帧 |

- 冻结后连续 `window 242 s`、`trip none`、`saturated 0`、`stack peak 904 B / 2048 B`、`guard/ptrs/wp/sp` 全 0、`fault counts` 全 0。
- 用户拔掉 USB 后实测：**能稳稳站住**；但轮速有时降到接近 0、有时停在较高转速（取决于起立姿态与真实平衡点之差，见下）。

## 已知限制（下一版要解决）

1. **走法自己在 4 s 周期上晃 ±1°**：`BAL_WALK_FILT_TAU_S = 8` 对 4 s 模态的衰减只有 0.23，外环把摆动当偏差追。计划改 `TAU 8 → 30`（衰减 ≈0.02）并把 `BAL_TRIM_W_K 5e-5 → 1e-4` 补偿收敛速度。
2. **`bal walk 0` 只改 RAM**，复位后回到编译默认 5e-5。
3. **轮速残差 = 573 rpm/deg × (起立姿态与真实平衡点之差)**：冻结走法时这个残差永远留着；走法开着才会消掉（但要几分钟）。
4. **拔线会让平衡点移动约 2.6°**（`18#T22` 的定量结果），走法冻结时拔线会导致轮速冲向 1500 rpm。
5. **随机"指针高位被置"崩溃族未解决**（已发生 5 次，`bfar` 在 RAM 之外，`ctx 4` 平衡路径；`crash` 记录可跨复位读回）。
6. `trip` 后 LED 不清除（显示 bug）；无看门狗。

## 回滚方法

```powershell
cd D:\Embedded\STM32\cubli
git checkout 704e1fe -- User/ Core/
# 或整体回退：git checkout 704e1fe
```

然后用 Keil 重新 build + Download。
