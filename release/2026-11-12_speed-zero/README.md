# 版本备份 2026-11-12 · 转速收敛版（Code=90236）

本目录保存的是**第一个"轮速能自己收敛到 0"的可回退固件**，供以后对照与回滚。
上一个备份是 `release/2026-11-11_stand-89848/`（能站但轮速常转）。

## 身份

| 项 | 值 |
|---|---|
| Keil 构建号 | `Code=90236` `RO-data=4156` `RW-data=800` `ZI-data=14592` |
| hex 生成时间 | 2026-11-12 00:57:11 |
| hex 大小 | 265 830 B |
| 源码提交 | 本目录所在提交（`fix: 转速收敛版`），上一个提交是 `f1546e9` |
| 板上 `ver` 自报 | `build = Nov 12 2026 00;37;49 #1`（同一份源码在 GUI 里重编过，构建串会不同） |
| 文件 | `cubli.hex`（本目录） |

烧录：Keil 打开 `MDK-ARM\cubli.uvprojx` → Download，或用本目录 `cubli.hex`。

## 这一版修了什么（相对 89848）

| 改动 | 效果 |
|---|---|
| 走法 tick 从 `app_bal_update()`（主循环每圈）搬到 `app_balance_update()`（IMU 帧闸门） | 走法从 **≈560 Hz** 回到 2 Hz（`docs/00#098`） |
| 帧闸门 `stat.frames` → `stat.frames_ahrs` | 从 2.99 calls/s 变成 **1.98 calls/s**（`docs/00#099`） |
| `bal run` 新增 `walk = N calls (… per s)` | 节拍可现场自检，四位数即说明 tick 又跑回主循环 |
| `BAL_WALK_FILT_TAU_S 8 → 30`、`BAL_TRIM_W_K 5e-5 → 1e-4` | 不再有 4 s 摆动，`trim` 单调收敛 |

## 关键参数（`User/Cfg/board_cfg.h`）

| 宏 | 值 | 说明 |
|---|---|---|
| `ATT_OFFSET_DEG` | 44.25 | 仅影响空闲读数与武装窗；运行中来一次 engage 就重标 |
| `BAL_KP_NM_RAD` / `BAL_KD_NM_RPS` / `BAL_KW_NM_RPS` | 1.313830 / 0.175517 / 3.819719e-04 | LQR（σ_ω = 50 rpm；`omega_ss = 573 rpm/deg`） |
| `BAL_SCALE` / `BAL_MAX_MNM` / `BAL_SLEW_MNM_STEP` | 0.7 / 28.0 / 3.0 | 首测降额、上限、每帧 slew |
| `BAL_ARM_DEG` | 10.0 | 起立门 |
| `BAL_TRIM_MAX_DEG` | 12.0 | 参考角钳位 |
| `BAL_TRIM_W_K` | **1.0e-4** | 零点游走增益（deg per rpm per second） |
| `BAL_WALK_DIV` / `BAL_WALK_DT_S` | 100 / 0.5 | 200 Hz AHRS 帧 → **2 Hz** 走一次 |
| `BAL_WALK_FILT_TAU_S` | **30.0** | 走法输入一阶低通 |
| `BAL_WALK_STEP_DEG` | 0.05 | 每次更新硬上限 |
| `BAL_WALK_DEADBAND_RPM` / `PHI_GATE_DEG` | 40.0 / 2.0 | 死区与冻结门 |
| `BAL_FF_MNM` / `BAL_FF_DEADBAND_RPM` | 2.05 / 40 | 摩擦前馈（带内线性） |
| `BAL_HELD_MNM` / `DEG` / `DPS` / `MS` / `GRACE_MS` / `PHI_DEG` | 12 / 1.0 / 5.0 / 150 / 800 / 0.30 | "被扶着"判据 |
| `BAL_MAX_MS` | 0 | 不限时（0 = 一直跑） |
| `BAL_TRIP_DEG` / `DPS` / `RPM` | 12 / 200 / 1700 | 保护阈值 |

## 起立语义

1. 短按按钮 → 松开 300 ms 后接管；
2. **接管时 `att_zero()`：把手上的姿态设为 0°**；
3. 起立第一帧参考角 = 0；
4. 之后零点游走以 2 Hz 把 `trim` 挪到真实平衡点，轮速随之衰减到 0。

## 实测（2026-11-12 00:52–00:56，COM13 只读）

| 量 | 值 |
|---|---|
| `walk` | 1.98…1.99 calls/s（61 219 ms 窗口 122 次） |
| 起立瞬态 | 轮速 −1217 rpm（`peak 1432`），`saturated 7` 帧 |
| 轮速衰减 | −477 → −320 → −267 → −230 → −207 → −186 → −158 → −143 → −132 → −108 → −56 → **−26 rpm**（28 s） |
| 稳态 | 轮速 −11…−36 rpm、`phi` 2.22…2.28°、`trim` **2.19~2.20°** 停住 |
| `trim` 斜坡 | 1.61 → 2.10°（20 s，≈0.025 °/s） |
| 保护 | `trip none`、窗口 61.2 s、`fault` 计数全 0 |

拔掉 USB 线后用户复测站立无异常。

## 已知限制（回滚前必读）

1. **起立瞬态轮速会甩到 −1200 rpm 上下**（松手姿态与平衡点差 2° + 线缆外力），离 1700 跳闸线约 500 rpm；松手姿态越准越安全。
2. **`trip held`（12 mN·m / 1.0° / 150 ms）在插 USB 线时容易误判**：线会托住机身，"大指令不动"判据成立就断驱动。
3. `trim` 只在 RAM：复位后回到 `ATT_OFFSET_DEG`，靠又一次 engage + 游走重找平衡点。
4. 插线时 `trim` 收敛值 ≈ **+2.19°**，即为线缆把平衡点拉出的偏移（`18#T22`）。
5. 随机 BusFault 族仍未定位（`90#L-21`/`L-24`/`T28`），约 1 小时量级偶发。

## 回滚

```powershell
cd D:\Embedded\STM32\cubli
git tag -l                      # v0.9-stand / v0.10-speed-zero
git checkout v0.9-stand -- .    # 或直接用本目录/上一目录的 cubli.hex 烧录
```

烧录上一版：`release\2026-11-11_stand-89848\cubli.hex`（能站、轮速常转）。
