# hwtest：基础硬件测试固件

这个工程用来在 PVC 管实验平台上测试三样东西：蓝牙和上位机能不能通信、扬声器能不能受控发声、两只麦克风能不能正常测量。还没有任何 ANC 算法。

- 芯片：STM32F407VET6 黑色最小系统板（原理图在 `资料/数据手册/开发板原理图_stm32f407v.pdf`），8 MHz 晶振，跑 168 MHz
- 外设库：STM32F4xx 标准外设库 V1.9.0（StdPeriph）
- 操作系统：FreeRTOS V10.3.1（heap_4）
- 工具链：STM32CubeCLT 1.22（arm-none-eabi-gcc 14.3、CMake、Ninja、STM32_Programmer_CLI）
- 上位机：`../../host/hwtest.py`（Python + pyserial），通过蓝牙 SPP 虚拟串口连接

## 1. 接线

| 模块 | 模块引脚 | 开发板 | 说明 |
| --- | --- | --- | --- |
| JDY-31 蓝牙 | VCC / GND | J6 的 5V / GND | J6 排针是 5V、GND、PA10、PA9 |
| | TXD | PA10（USART1_RX） | 交叉连接 |
| | RXD | PA9（USART1_TX） | |
| 参考麦 MAX4466 | VCC / GND | 3.3V / GND | **必须 3.3V**，不然输出会超过 ADC 量程 |
| | OUT | PC0（ADC123_IN10） | 建议串 2.2 kΩ，引脚端对地接 22 nF 做抗混叠 |
| 误差麦 MAX4466 | OUT | PC1（ADC123_IN11） | 同上 |
| PAM8403 | VCC / GND | 外部 5V / GND | 不要用 3.3V 去供 |
| | 左声道输入 L | PA4（DAC_OUT1）→ 10 µF 隔直 | 建议再加一级约 1/5 的电阻分压，避免削波 |
| | 左声道输出 L+ / L− | 扬声器两端 | BTL 输出，**L− 不能接地** |

所有模块的 GND 都要和开发板共地。

板上资源：D2（PA6）心跳灯，500 ms 翻转一次；如果变成 80 ms 快闪，说明 8 MHz 外部晶振没起振。D3（PA7）在扬声器有输出时亮。K0（PE4）开关 200 Hz 单音，K1（PE3）静音，K_UP（PA0）开关白噪声。

## 2. 编译与烧录

```powershell
cd D:\Work\Diploma\DuctANC\firmware\hwtest
cmake --preset debug
cmake --build --preset debug
cmake --build --preset debug --target flash   # ST-Link 接 SWD（PA13/PA14）
```

编译产物在 `build/` 下：`hwtest.elf`、`hwtest.bin`、`hwtest.hex`。如果不用 ST-Link，也可以用 CubeProgrammer 图形界面打开 hex 烧录。

两个可调的编译参数：

- `-DBT_BAUD=9600`：USART1 波特率，必须和蓝牙模块一致。JDY-31 出厂是 9600。
- `-DFS_HZ=8000`：ADC 和 DAC 的采样率。

改参数的方法：`cmake --preset debug -DBT_BAUD=115200`，然后重新编译。

## 3. 蓝牙连接

1. 给板子上电，JDY-31 的灯会快闪，表示还没配对。
2. 在 Windows 里打开「设置 → 蓝牙和其他设备 → 添加设备」，选 `JDY-31-SPP`。如果要配对码，一般是 `1234`。
3. 配对后系统会生成两个 COM 口，要用「传出」（outgoing）那个。可以在「更多蓝牙设置 → COM 端口」里看，或者运行：

```powershell
cd D:\Work\Diploma\DuctANC\host
pip install -r requirements.txt
python hwtest.py ports
```

手机上的 SPP 串口助手（任意蓝牙串口 App）也能直接发命令，每条命令以换行结尾。

**波特率的影响**：9600 baud 每秒大约只能传 960 字节。采集 0.5 s 双通道（4000 帧，16 KB）大约要 17 s。如果想快一些，可以把 JDY-31 改成 115200：先断开蓝牙连接，用 USB-TTL 接模块，发送 `AT+BAUD8`（带回车换行；具体以模块手册为准），然后固件用 `-DBT_BAUD=115200` 重新编译，上位机加 `-b 115200`。

## 4. 测试步骤

下面以 `COM7` 为例，换成你自己的端口号。

| 步骤 | 命令 | 合格标准 |
| --- | --- | --- |
| 1 上电 | — | D2 每 500 ms 闪一次；连上蓝牙后收到 `# duct-anc-hwtest-0.1 ...` 横幅 |
| 2 链路 | `python hwtest.py -p COM7 ping -n 50` | 0 丢包。往返时间大约几十 ms，属于蓝牙的正常延迟 |
| 3 状态 | `python hwtest.py -p COM7 info` | `clk_ok=1`，`ovr=0`，`udr=0`，`late=0` |
| 4 扬声器 | `tone 200`、`sweep 100 1000 5`、`noise`、`stop` | 能听到声音，D3 亮；`stop` 后在 20 ms 内淡出，没有爆音 |
| 5 麦克风静态 | `level --watch` | 两路 `dc` 都在 2048 左右（约 1650 mV），`clip=0`，安静时 rms 只有几 mV |
| 6 麦克风动态 | 边 `level --watch` 边拍手 / 开风扇 | rms 明显上升，不出现 `clip` |
| 7 采集 | `capture --frames 4000 --tag fan_on` | 在 `实验记录/` 下生成 wav、npz 和 png（波形、频谱、相干函数） |
| 8 一键自检 | `selftest --check-swap` | 全部 PASS，报告写到 `实验记录/<时间>_selftest/report.md` |

自检做的事情：PING → INFO → 两路偏置 → 采一段静音 → 扬声器放 200 Hz → 再采一段 → 判断两只麦在 200 Hz 处是否都比静音高出 10 dB 以上。加 `--check-swap` 后会提示你对着参考麦吹气，用来确认 PC0 和 PC1 没有接反。

### 麦克风专项测试 `mictest`

```powershell
python hwtest.py -p COM16 mictest
```

按提示操作，全程大约 1.5 分钟（9600 baud 时每段 0.5 s 采集要传约 17 s）：

| 阶段 | 你要做的 | 自动判断 |
| --- | --- | --- |
| 1 本底 | 关风扇，保持安静 | 偏置约 1650 mV；本底 rms < 50 mV；两路独立（排除 ADC 串扰） |
| 2 敲击定位 | 先反复轻敲参考麦旁的管壁，再敲误差麦旁的管壁，各 3 s | 被敲的那只要比另一只高 3 dB 以上，以此确认 PC0/PC1 没接反 |
| 3 扬声器白噪声 | 无 | 两只麦在 100–400 Hz 都比本底高 ≥10 dB 且不削波；GCC-PHAT 测出误差麦比参考麦早约 3 个样本（误差麦离喇叭更近） |
| 4 风扇 | 按提示开风扇 | 不削波、量程占用 5%–90%；100–400 Hz 信噪比 ≥20 dB 为理想，≥12 dB 为最低；两麦相干 γ^2 及对应的理论降噪上限；误差麦比参考麦晚约 10 个样本（428 mm） |

结果分三档：PASS；WARN（可以用，但不理想）；FAIL（要处理）。报告和三段数据（wav/npz/png）保存在 `实验记录/<时间>_mictest/`。

如果你的管子尺寸和架构文档不一样，用 `--d-ref-err`、`--d-spk-ref`、`--d-spk-err`（单位 mm）告诉脚本实际尺寸。喇叭到某只麦的声程按「支管长度 + 三通轴线到该麦的距离」计算。8 kHz 采样时 1 个样本约对应 43 mm，默认允许偏差 ±3 个样本。

**麦克风增益怎么调**：MAX4466 背面的电位器决定增益。先开风扇，同时把扬声器开到你打算用的最大幅度，然后调电位器，让 `max-min` 大约占量程的一半，且 `clip=0`。调好之后不要再动，因为后面辨识出的次级路径和这个增益绑定。

**扬声器幅度**：`amp` 是相对 DAC 满幅的比例，默认 0.1。PAM8403 自带大约 24 dB 的增益，没有分压时 amp 超过 0.2 左右就可能削波，听起来会发破。

## 5. 串口协议（上位机和手机共用）

每条命令占一行，大小写都可以。回复规则：成功以 `OK` 开头，失败以 `ERR` 开头，数据行以 `PONG`/`INFO`/`LVL` 开头，`#` 开头的是注释行，`KEY` 是按键事件。

| 命令 | 说明 |
| --- | --- |
| `HELP` | 列出所有命令 |
| `PING [token]` | 回复 `PONG token tick_ms` |
| `INFO` | 固件版本、时钟、输出状态、`blocks`/`late`/`ovr`/`udr`/`rxovf` 计数、剩余堆 |
| `TONE f [amp]` | 正弦，20 Hz–3600 Hz |
| `NOISE [amp]` | 均匀白噪声 |
| `SWEEP f1 f2 sec [amp]` | 对数扫频，循环播放 |
| `STOP` | 静音，DAC 回到中点 2048 |
| `LEVEL` | 最近约 0.5 s 每路的 dc、rms（码值）、mV、dBFS、min、max、削波点数 |
| `STREAM ms` | 每隔 ms 毫秒自动输出一次 `LEVEL`；`STREAM 0` 关闭 |
| `CAP n` | 采集 n 帧（上限 16000），先回 `#CAP n fs 2`，接着是 n×2×uint16 小端二进制（交织 ref,err），最后 `#END <32 位校验和>` |
| `RESET` | 重启 |

## 6. 软件结构

```
TIM2 (8 kHz TRGO) ─┬─> ADC1 扫描 PC0,PC1 ─> DMA2 S0 环形 ─┐
                   └─> DAC1 PA4 <─ DMA1 S5 环形 <─────────┤ 半满/全满中断 → 任务通知
                                                          ▼
        audio 任务 (优先级 6)：回填 DAC 半缓冲（信号发生 + 20 ms 淡入淡出）
                               统计 ADC 半缓冲（dc/rms/min/max/clip），按需拷贝到 CCM 采集区
        cli 任务   (优先级 3)：蓝牙命令行
        house 任务 (优先级 2)：LED、按键、STREAM 定时输出
USART1 中断 (优先级 6) ⇄ FreeRTOS 流缓冲 ⇄ bt_printf / bt_getline（递归互斥锁保证一行不被打断）
```

| 文件 | 内容 |
| --- | --- |
| `User/board.h` | 引脚、波特率、中断优先级 |
| `User/bsp.c` | 时钟检查、LED、按键消抖 |
| `User/bt_uart.c` | USART1 中断收发、流缓冲 |
| `User/audio.c` | TIM2/ADC/DAC/DMA 配置、信号发生、电平统计、采集 |
| `User/app.c` | 命令行和人机任务 |
| `User/system_stm32f4xx.c` | 改过 `PLL_M = 8`，对应 8 MHz 晶振 |
| `ld/STM32F407VETx_FLASH.ld` | 512 KB Flash、128 KB SRAM、64 KB CCM（采集缓冲放在 CCM） |

这里的块处理（每块 128 帧，延迟约 16 ms）只用于测试。后面的 ANC 固件需要改成逐点中断，在一个采样周期内完成从 ADC 到 DAC 的计算，见 `01_设计评估.md` 里的时延分析。
