# hwtest：基础硬件测试固件

这个工程用来在 PVC 管实验平台上测试蓝牙、扬声器和两只麦克风，并从 0.3 起跑 200 Hz 的逐点 FxNLMS。旧命令（TONE、NOISE、SWEEP、STOP、LEVEL、CAP、INFO）还在。ANC 默认关，开着的时候 TONE / NOISE / SWEEP 会回 `ERR anc on`，不改 DAC。

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

- `-DBT_BAUD=115200`：USART1 波特率（默认 115200）。JDY-31 出厂是 9600，固件开机时会自动把模块改成这个值，见第 3 节。
- `-DFS_HZ=8000`：ADC 和 DAC 的采样率。

改参数的方法：`cmake --preset debug -DBT_BAUD=115200`，然后重新编译。注意 CMake 会缓存旧值，已有 `build/` 时改默认值不会生效，要用 `-D` 显式指定。

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

**波特率与自动配置**：JDY-31 只在没有蓝牙连接时响应 AT 指令（连上以后串口是透传）。固件开机约 0.8 s 后，先按 `BT_BAUD` 发 `AT+BAUD` 查询；不通就按出厂 9600 再查，通了就发 `AT+BAUD8`（115200）和 `AT+RESET`，再按新速率确认。设置保存在模块里，只需成功一次。结果显示在开机提示和 `INFO` 的 `bt=` 字段：

| `bt=` | 含义 |
| --- | --- |
| `ok` | 模块已经是 `BT_BAUD` |
| `set` | 刚从 9600 改成功 |
| `set_fail` | 已发改速率指令但没确认，断电重启再看 |
| `noreply` | 两种速率都没回复：开机时电脑已经连着蓝牙（比如只用 ST-Link 复位了 MCU，模块没断电）。模块已经改过的话照常能用；**第一次改速率时，必须在电脑没连蓝牙的状态下给板子断电重启** |

115200 时采集 4000 帧（16 KB）约 1.5 s，16000 帧约 6 s；9600 时分别要 17 s 和 67 s。上位机 `-b` 默认 115200。要退回 9600：固件用 `-DBT_BAUD=9600` 编译，开机时模块会被自动改回去，上位机加 `-b 9600`。

## 4. 测试步骤

下面以 `COM7` 为例，换成你自己的端口号。

| 步骤 | 命令 | 合格标准 |
| --- | --- | --- |
| 1 上电 | — | D2 每 500 ms 闪一次；连上蓝牙后收到 `# duct-anc-hwtest-0.3 ...` 横幅 |
| 2 链路 | `python hwtest.py -p COM7 ping -n 50` | 0 丢包。往返时间大约几十 ms，属于蓝牙的正常延迟 |
| 3 状态 | `python hwtest.py -p COM7 info` | `clk_ok=1`，`ovr=0`，`udr=0`，`late=0` |
| 4 扬声器 | `tone 200`、`sweep 100 1000 5`、`noise`、`stop` | 能听到声音，D3 亮；`stop` 后在 20 ms 内淡出，没有爆音 |
| 5 麦克风静态 | `level --watch` | 两路 `dc` 都在 2048 左右（约 1650 mV），`clip=0`，安静时 rms 只有几 mV |
| 6 麦克风动态 | 边 `level --watch` 边拍手 / 用手机播放 200 Hz 单音 | rms 明显上升，不出现 `clip` |
| 7 采集 | `capture --frames 4000 --tag source_on` | 在 `实验记录/` 下生成 wav、npz 和 png（波形、频谱、相干函数） |
| 8 采集 + DAC | 先 `noise --amp 0.02`，再 `capture --dac --frames 10666 --tag sid` | npz 里多一个 `dac` 数组；另存 `sid_dac.png`：DAC→两麦的冲激响应粗估和相干，终端打印主峰延迟。先 `stop` 再结束 |
| 8 一键自检 | `selftest --check-swap` | 全部 PASS，报告写到 `实验记录/<时间>_selftest/report.md` |

自检做的事情：PING → INFO → 两路偏置 → 采一段静音 → 扬声器放 200 Hz → 再采一段 → 判断两只麦在 200 Hz 处是否都比静音高出 10 dB 以上。加 `--check-swap` 后会提示你对着参考麦吹气，用来确认 PC0 和 PC1 没有接反。

### 麦克风专项测试 `mictest`

```powershell
python hwtest.py -p COM16 mictest
```

按提示操作，全程大约 1.5 分钟（9600 baud 时每段 0.5 s 采集要传约 17 s）：

| 阶段 | 你要做的 | 自动判断 |
| --- | --- | --- |
| 1 本底 | 关闭外部声源，保持安静 | 偏置约 1650 mV；本底 rms < 50 mV；两路独立（排除 ADC 串扰） |
| 2 敲击定位 | 先反复轻敲参考麦旁的管壁，再敲误差麦旁的管壁，各 3 s | 被敲的那只要比另一只高 3 dB 以上，以此确认 PC0/PC1 没接反 |
| 3 扬声器白噪声 | 无 | 两只麦在 100–400 Hz 都比本底高 ≥10 dB 且不削波；GCC-PHAT 测出误差麦比参考麦早约 3 个样本（误差麦离喇叭更近） |
| 4 外部声源 | 按提示用手机播放 200 Hz 单音 | 不削波、量程占用 5%–90%；100–400 Hz 信噪比 ≥20 dB 为理想，≥12 dB 为最低；两麦相干 γ^2 及对应的理论降噪上限；误差麦比参考麦晚约 10 个样本（428 mm） |

结果分三档：PASS；WARN（可以用，但不理想）；FAIL（要处理）。报告和三段数据（wav/npz/png）保存在 `实验记录/<时间>_mictest/`。

如果你的管子尺寸和架构文档不一样，用 `--d-ref-err`、`--d-spk-ref`、`--d-spk-err`（单位 mm）告诉脚本实际尺寸。喇叭到某只麦的声程按「支管长度 + 三通轴线到该麦的距离」计算。8 kHz 采样时 1 个样本约对应 43 mm，默认允许偏差 ±3 个样本。

**麦克风增益怎么调**：MAX4466 背面的电位器决定增益。先用手机播放 200 Hz 单音，同时把扬声器开到你打算用的最大幅度，然后调电位器，让 `max-min` 大约占量程的一半，且 `clip=0`。调好之后不要再动，因为后面辨识出的次级路径和这个增益绑定。

**扬声器幅度**：`amp` 是相对 DAC 满幅的比例，默认 0.1。PAM8403 自带大约 24 dB 的增益，没有分压时 amp 超过 0.2 左右就可能削波，听起来会发破。

### P0 硬件对照 `p0test`

```powershell
python hwtest.py -p COM16 p0test                # 全部
python hwtest.py -p COM16 p0test --part couple  # 只做参考麦电串扰
python hwtest.py -p COM16 p0test --part source  # 只做外部声源
```

按提示操作。9600 baud 时每段 4000 帧采集要传约 17 s：

| 阶段 | 你要做的 | 自动判断 |
| --- | --- | --- |
| 0 本底 | 关闭外部声源和喇叭，保持安静 | 各频段本底，作为后面所有对比的基准 |
| 喇叭幅度搜索 | 无 | 从 `--amps` 里选第一个不削波、量程 <90% 的幅度 |
| 接喇叭 | 无 | 记录喇叭单音下两麦幅度和相位 |
| 拔喇叭线 | 只拔 PAM8403 输出端，功放保持通电 | err 应回到本底（声学路径断开）；ref 残留 = 电串扰·功放通电 |
| 关功放 | 关闭 PAM8403 电源，DAC 照常输出 | ref 残留 = 电串扰·功放断电；结合上一步定位串扰来自功放侧还是 DAC/板子 |
| 调音量 | 手机固定支架上播放 200 Hz 单音 | 循环显示 LEVEL，把参考麦 rms 调到 `--ref-target` 范围（默认 200–300 mV）；发现削波会要求先调小音量，确认后才继续 |
| 支架 | 无（手机在支架上播放） | ref 电平、削波、谐波失真（区分手机喇叭失真和参考麦近场饱和） |
| 贴管 | 把手机贴在管壁上 | 对比支架段，判断低频振动是不是碰管带进来的 |
| 静音文件 | 改播静音文件或暂停 | 区分低频来自播放内容还是支架/气流 |
| MCU 同步干扰 | 无（自动检查所有段） | 若某段出现每 128 样本（固件处理块，16 ms）一次的跳变则 FAIL：这不是声音。对照 0 段时的功放/喇叭接线和地线；手机拿远、喇叭 STOP 时用 `capture` 复查，再拔喇叭线复查——消失说明喇叭在放 DAC 线上的噪声，仍在则是地线/接线耦合。此时低频来源会标「无法判断」，排除干扰后重测 |

报告和每段数据（wav/npz/png）保存在 `实验记录/<时间>_p0test/`。

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
| `CAP n D` | 同上再加 DAC 通道（上限 10666 帧）：`#CAP n fs 3`，交织 ref,err,dac。dac 是该帧 ADC 触发时刻起 DAC 正在输出的码值（中点 2048），与麦克风逐帧对齐，用于离线辨识次级路径。对齐依赖 DMA 没有重启过，采集前后 `INFO` 里 `ovr`、`udr` 应为 0 |
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
