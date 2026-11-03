# test/ —— 测试与门禁

本目录是三方验证门禁的执行地（见 [../workflow/05_验证与提交规范.md](../workflow/05_验证与提交规范.md)）。
**每个阶段收尾、每次提交前**都要跑；退出码非 0 不允许提交。

| 项 | 命令 | 层级 | 现状 |
|---|---|---|---|
| 跨文件绑定检查 | `python test/check_binding.py` | 静态 | ✅ 11 项 |
| 纯算法离线对拍 | `bash test/l1/run.sh` | L1 | ✅ 15 项（`mid_fdilink`） |
| 主机测试（fake HAL） | `bash test/host/build.sh` | L2 | ✅ 594 项 |
| 文档一致性检查 | `python test/doc_check.py` | 静态 | ✅ PASS |
| 一键复现 | `bash test/verify.sh` | 汇总 | ✅ 4 道门全绿 |

固件构建（0 error / 0 warning）是 **Windows 独有**的门禁，不在 `verify.sh` 里：

```powershell
Start-Process "D:/installfile/Keil_v5/UV4/UV4.exe" `
  -ArgumentList '-r','cubli.uvprojx','-j0','-o','build_s1_step7.log' `
  -WorkingDirectory "D:\Embedded\STM32\cubli\MDK-ARM" -Wait -PassThru
```

返回码 0 = 成功，1 = 有警告，2 = 有错误；**每次换日志文件名**。

## 层级说明

- **L1（`test/l1/`）**：把 `User/Mid/`、`User/Alg/` 里**不依赖 HAL** 的源码原样拿到 PC 上用 gcc 编译，
  喂固定输入对拍。不需要硬件，不需要 Keil，几秒出结果。
- **L2（`test/host/`）**：用 fake HAL 编译真实固件源码，验证状态机、限幅、模式切换等跨模块逻辑。
- **L3**：上板 / 串口脚本，只在 L1、L2 都过之后才用。

## check_binding.py 为什么要存在

编译通过 ≠ 代码搬对了。两个历史案例（`docs/00_开发日志.md` #008、#009）：

1. `bsp_ssi.c` 把 SPI1_RX 的 DMA 写死成 `DMA1_Channel3`，而本工程是 CH5 ——
   `0 Error(s) 0 Warning(s)`，上板表现是编码器读数不动但 CRC 正常。
2. IMU 波特率宏与 CubeMX 的 `huart1.Init.BaudRate` 不一致 —— 同样编译通过，
   上板表现是"永远收不到完整帧"，且不会有任何错误计数。

这类错误**只能靠跨文件一致性检查抓**，所以检查器里每条断言都是"两个文件里的同一件事必须对得上"。

检查器自己也要被验证：每次改完规则，故意破坏一处配置，确认它真的报 FAIL、退出码 1，再还原。

## l1/run.sh 的环境自证

脚本开头会打印 `BASH_VERSION` 与 `gcc` 路径。原因见
[../workflow/04_仿真先行.md](../workflow/04_仿真先行.md)：在 Windows 下误用 msys2 的 bash
会导致 `undefined reference to 'cosf'` 之类的假错误，把真正的问题盖住。

本工程实测可用环境：**WSL Ubuntu-24.04 + gcc 13.3.0**（脚本保持 POSIX 写法，任何带 gcc 的环境都能跑）。
