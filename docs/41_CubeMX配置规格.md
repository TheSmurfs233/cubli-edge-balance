# CubeMX 工程配置规格（cubli.ioc）

> 用途：S1 第 1 步"建立 CubeMX 工程"的**逐项操作清单**。按本表配置，生成的 `Core/` 与本工程固件预期一致。
> 数据来源：`D:\Embedded\STM32\dsh_foc\myfoc.ioc`（已验收，`MxCube.Version=6.18.1`）+ 本工程新增的 USART2。
> 负责人：**用户**（在 CubeMX GUI 里操作）；生成完成后由开发方接管代码合并。
> 最后更新：2026-11-03

---

## 0. 环境

| 项 | 值 |
|---|---|
| CubeMX | `D:\installfile\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe`（6.18.1-RC2） |
| 固件包 | `STM32Cube_FW_G4_V1.6.3`（`C:\Users\17634\STM32Cube\Repository` 已存在） |
| 目标工程 | `D:\Embedded\STM32\cubli\cubli.ioc` |
| 工程名 | `cubli` |
| 工具链 | **MDK-ARM V5.32** |
| 库拷贝方式 | Copy only the necessary library files |

---

## 1. 两条路线，二选一

### 路线 A（推荐）：复制现成 `.ioc` 再改

1. CubeMX 打开 `D:\Embedded\STM32\dsh_foc\myfoc.ioc`
2. `File → Save Project As…` → 存为 `D:\Embedded\STM32\cubli\cubli.ioc`
3. `Project Manager` 改 **Project Name = `cubli`**，确认 Toolchain/IDE = **MDK-ARM V5.32**
4. 按第 3 节 **只做 4 处改动**：USART1 波特率、新增 USART2、新增 USART2 的两个 DMA 通道、USART2 中断
5. `GENERATE CODE`

> 路线 A 的好处：时钟树、TIM1 死区、ADC 注入组、SPI 相位这些"调对了但说不清"的项一次到位，不会漏。

### 路线 B：从零新建

`New Project` → 选 `STM32G431KBTx`（LQFP32）→ 严格按第 2 节全部参数表配置。
**任何一项与表不符都算配置错误**，生成后必须逐项对照第 4 节自检。

---

## 2. 完整参数表（从零建时全填，路线 A 只需核对）

### 2.1 MCU 与系统

| 项 | 值 |
|---|---|
| MCU | `STM32G431KBTx`，LQFP32，128 KB Flash / 32 KB RAM |
| Debug | **Serial Wire**（PA13/PA14） |
| Timebase Source | **SysTick** |
| Dead Battery signals | Disable |

### 2.2 时钟树（RCC）

| 项 | 值 |
|---|---|
| HSE | **不用**（无外部晶振） |
| PLL Source | **HSI**（16 MHz） |
| PLLM | `/4` → VCO 输入 **4 MHz** |
| PLLN | `×85` → VCO 输出 **340 MHz** |
| PLLP | `/2` → **170 MHz** |
| PLLQ / PLLR | `/2` |
| SYSCLK Source | **PLLCLK** |
| AHB / APB1 / APB2 预分频 | **/1**（全部 170 MHz） |
| TIM1 时钟 | 170 MHz（APB2 timer clock） |
| ADC 时钟 | 170 MHz（ADC12Freq_Value） |

> 核对点：Timers / APB2 = **170 MHz**，否则 20 kHz PWM 的 ARR 算错。

### 2.3 TIM1（三相 PWM，20 kHz 中心对齐）

| 项 | 值 |
|---|---|
| Clock Source | Internal Clock |
| **Channel1** | PWM Generation CH1 **CH1N** |
| **Channel2** | PWM Generation CH2 **CH2N** |
| **Channel3** | PWM Generation CH3 **CH3N** |
| **Channel4** | PWM Generation CH4 |
| Mode | **Center Aligned mode 1** |
| Prescaler | **0** |
| Counter Period (ARR) | **4249**（`4250-1`）→ 170 MHz / (2×4250) = **20.0 kHz** |
| Internal Clock Division | No Division |
| Repetition Counter | **0** |
| auto-reload preload | **Enable** |
| **Dead Time** | **4**（引脚死区 ≈23.5 ns，叠加 EG3033 内部 60 ns） |
| CH1/CH2/CH3 CHxN 模式 | **PWM mode 2** |
| CH4 模式 | **PWM mode 2**，Pulse = **4000** |
| OC1/OC2/OC3 Preload | **Enable** |
| Off State IDLE Mode（OSSI） | **Enable** |

### 2.4 ADC1（母线电压，规则组）

| 项 | 值 |
|---|---|
| 通道 | **IN2**（PA1，Single-ended） |
| 规则组 Rank 1 | **Channel 2**，采样时间 **47.5 cycles** |
| 偏移 | None |
| 其余 | 默认 |

### 2.5 ADC2（三相电流，注入组）

| 项 | 值 |
|---|---|
| Enable Injected Conversion | **Enable** |
| 注入组 Rank 1 | **Channel 17**（PA4 = ISU），**24.5 cycles** |
| 注入组 Rank 2 | **Channel 13**（PA5 = ISV），**24.5 cycles** |
| 注入组 Rank 3 | **Channel 3**（PA6 = ISW），**24.5 cycles** |
| **Injected External Trigger** | **TIM1_CH4**（`ADC_EXTERNALTRIGINJEC_T1_CC4`） |
| Injected Conversion Mode | None |
| EOC Selection | **EOC of sequence** |
| 规则组 Rank 1 | Channel 17（占位，未使用），47.5 cycles |

### 2.6 SPI1（MT6701 编码器，SSI 读）

| 项 | 值 |
|---|---|
| Mode | **Master**，Full-Duplex（2 Lines） |
| Data Size | **8 bit** |
| **Baud Rate Prescaler** | **16** → 10.625 Mbit/s |
| **Clock Phase** | **2 Edge** |
| Clock Polarity | Low |
| NSS | Software（片选由 PA15 GPIO 控制） |
| First Bit | MSB |

> 时钟相位必须是 **2 Edge**，与 dsh_foc 实测匹配，改成 1 Edge 会读错角度。

### 2.7 USART1（接 IMU，PB6/PB7 → CN3）

| 项 | 值 |
|---|---|
| Mode | **Asynchronous** |
| **Baud Rate** | **921600** Bits/s（⚠ 现 `.ioc` 里是 1000000，**必须改**） |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| Oversampling | 16 |
| DMA RX | **DMA1_Channel1**，**Circular**，Priority Low |
| DMA TX | **DMA1_Channel2**，**Normal**，Priority Low |
| NVIC | USART1 global interrupt **Enable**，抢占优先级 **3** |

### 2.8 USART2（调试口，PA2/PA3 → CH340N → USB-C）—— **本工程新增**

| 项 | 值 |
|---|---|
| Mode | **Asynchronous** |
| Baud Rate | **1000000** Bits/s |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| **DMA RX** | 任一空闲 DMA1 通道（实测派到 **DMA1_Channel3**），**Circular**，Priority Low |
| **DMA TX** | 另一空闲通道（实测 **DMA1_Channel4**），**Normal**，Priority Low |
| **NVIC** | USART2 global interrupt **Enable**，抢占优先级 **4** |

> 为什么必须先 DMA：调试口要跑命令台 + 日志 + 波形，阻塞收发会直接拖慢 20 kHz 电流环。
> DMA 通道选择：G431 的 **DMA1 只有 CH1~CH6，没有 CH7**（初版本文档写 CH6/CH7 是错的）。
> G4 带 DMAMUX，任意通道可映射任意请求，所以不必强求具体通道号；只要 CubeMX 不报冲突即可。
> 实测 CubeMX 把 USART2_RX 派到 CH3、USART2_TX 派到 CH4，并把原来的 SPI1_RX 从 CH3 自动挪到 **CH5**。
> 移植 `dsh_foc` 的代码时**只认句柄名**（`hdma_usart2_rx` 等），不要认通道号。

### 2.9 GPIO

| 引脚 | 模式 | 参数 |
|---|---|---|
| PF0-OSC_IN | **TIM1_CH3N**（PWM Generation3 CH3 CH3N） | — |
| PF1-OSC_OUT | GPIO_Output | Label = `LED`，低电平点亮 |
| PA0 | GPIO_Input | Label = `KEY`，**Pull-up**，按下为低 |
| PA1 | ADC1_IN2 | Single-ended |
| PA4 / PA5 / PA6 | ADC2_IN17 / IN13 / IN3 | Single-ended |
| PA7 | TIM1_CH1N | — |
| PB0 | TIM1_CH2N | — |
| PA8 / PA9 / PA10 | TIM1_CH1 / CH2 / CH3 | — |
| PA11 | TIM1_CH4 | — |
| PA15 | GPIO_Output | Label = `CSN`，**Speed High**，**初始电平 High** |
| PB3 | SPI1_SCK | Speed **High** |
| PB4 | SPI1_MISO | Speed Medium |
| PB5 | SPI1_MOSI | Speed Medium |
| PB6 / PB7 | USART1_TX / RX | — |
| PA2 / PA3 | USART2_TX / RX | — |
| PA13 / PA14 | SYS SWDIO / SWCLK | Serial Wire |

> PA12 空着（原 FDCAN_TX，本版不用）。

### 2.10 NVIC

优先级分组 **4 bit preempt / 0 bit sub**（`NVIC_PRIORITYGROUP_4`）。

| 中断 | 抢占优先级 | 使能 |
|---|---|---|
| ADC1_2_IRQn | **0** | ✅ |
| DMA1_Channel1_IRQn | 2 | ✅ |
| DMA1_Channel2_IRQn | 2 | ✅ |
| DMA1_Channel3_IRQn | 0 | ✅ |
| DMA1_Channel3_IRQn | 2 | ✅（USART2_RX 占用） |
| DMA1_Channel4_IRQn | 2 | ✅（USART2_TX 占用） |
| DMA1_Channel5_IRQn | 2 | ✅（SPI1_RX 占用） |
| USART1_IRQn | 3 | ✅ |
| USART2_IRQn | 4 | ✅（新增） |
| SysTick_IRQn | 15 | ✅ |

> "Force DMA interrupt vector in NVIC" 保持默认即可。

---

## 3. 若走路线 A，只改这 4 处

| # | 位置 | 改动 |
|---|---|---|
| 1 | Connectivity → USART1 → Parameter Settings | Baud Rate `1000000` → **`921600`** |
| 2 | Connectivity → USART2 | 开启 **Asynchronous**，PA2/PA3 自动分配，Baud **1000000** |
| 3 | USART2 → DMA Settings | 加 **USART2_RX（Circular）**、**USART2_TX（Normal）**，通道号让 CubeMX 自动分配 |
| 4 | System Core → NVIC | 勾选 **USART2 global interrupt**，优先级 **4** |

⚠ **不要动**：时钟树、TIM1、ADC1/ADC2、SPI1、USART1 的其它参数、GPIO 已有分配。这些是 `dsh_foc` 反复实测调过的。

---

## 4. 生成后自检（把结果贴回来，或直接告诉我"生成好了"）

生成完成后检查这几项，任何一项不符请先别往下走：

| # | 检查项 | 期望 |
|---|---|---|
| 1 | 目录 `D:\Embedded\STM32\cubli\Core\`、`Drivers\`、`MDK-ARM\` | 都存在 |
| 2 | `MDK-ARM\` 下的工程文件 | `cubli.uvprojx` |
| 3 | `Core\Src\usart.c` | 有 `huart1` **和** `huart2` |
| 4 | `Core\Src\dma.c` | 有 `hdma_usart1_rx/tx` 与 `hdma_usart2_rx/tx`，5 个通道互不重复 |
| 5 | `Core\Src\tim.c` | `Prescaler=0`、`Period=4249`、`DeadTime=4`、`CounterMode=CENTERALIGNED1` |
| 6 | `Core\Src\adc.c` | ADC2 注入组 3 个 rank，触发源 `ADC_EXTERNALTRIGINJEC_T1_CC4` |
| 7 | `Core\Src\spi.c` | `BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16`、`CLKPhase=SPI_PHASE_2EDGE` |
| 8 | `Core\Src\gpio.c` | `CSN` 引脚初始电平 **SET**、`LED` 引脚存在 |
| 9 | `cubli.ioc` | `ProjectManager.ProjectName=cubli`，含 `USART2` 段落 |
| 10 | `cubli.ioc` | `USART1.BaudRate=921600` |

---

## 5. 已知的、生成后必须补的东西（开发方负责，不是你的活）

| 项 | 原因 |
|---|---|
| `MDK-ARM\cubli.uvprojx` 里加 `User\**\*.c` 分组与 include 路径 | CubeMX 只登记 `Core/`，`User/` 全是手工加的（dsh_foc 手工改了 12 次） |
| `User\`、`test\`、`sim\` 目录 | 不在 CubeMX 管辖范围 |
| `-D` 预定义宏（如 `USE_HAL_DRIVER`、版本宏） | 同上 |
| `Core\Src\main.c` 的 `USER CODE` 段接入 `app_main()` | 避免下次重新生成被覆盖 |
| **`MDK-ARM\cubli.uvprojx` 的 IROM 链接区收窄到 `0x8000000`/`0x1f800`**（`OnChipMemories/OCR_RVCT4`、`OCR_RVCT9`） | CubeMX 生成的工程这两个字段是**空的**，实际 scatter 变成 128 KB 全量；而校准记录区 `CAL_FLASH_ADDR=0x0801F800` 占最后一页 2 KB，`bsp_flash` 擦写会**擦掉代码**。首次生成已踩到，见 [00_开发日志.md](00_开发日志.md) #007 |
| **`MDK-ARM\startup_stm32g431xx.s` 的 `Stack_Size` 改 `0x800`** | CubeMX 默认 `0x400`；日志里的 `printf`/浮点格式化可能要 500 B+ 栈，异常复位优先怀疑栈溢出 |
| 重新生成后再核对 §3 的 4 项参数 | CubeMX GUI 改完保存，生成代码才生效；每次重生成都要复查一遍 |
