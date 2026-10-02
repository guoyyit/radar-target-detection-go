# radar-target-detection-go

本地调频连续波（FMCW）雷达信号处理与目标检测服务。输入为内存管道模拟的基带回波帧，
输出每帧检测到的目标（距离、径向速度、幅度、信噪比），并支持跨帧关联与断点续算。

## 处理链路

```
Pipe(内存管道)
  -> ParseFrame      参数校验 / 维度一致性校验，拒绝原因可区分
  -> PreparePulses   快时间直流消除 + 距离向加窗 + 补零
  -> range FFT       每脉冲距离向 FFT
  -> 慢时间直流消除  距离单元级复数均值相消（精确落在零多普勒单元）
  -> Doppler FFT     多普勒向加窗 + FFT，得到二维幅度谱及物理轴
  -> Detect          二维 CA-CFAR（保护/训练单元可配）+ 8 邻域局部极大值
  -> Cluster         相邻峰并查集合并，幅度加权质心
  -> Tracker         贪心最近邻关联 + 连续确认 + 保持期 + 滑窗平滑
  -> Checkpoint      每帧完整算完后原子持久化 LastCompletedFrame
```

### 关键正确性点

- **虚假目标/漏检**：直流偏置在距离 FFT 之后按距离单元做复数均值相消，其能量
  完全集中在零多普勒单元，配合 `SkipDCBins` 排除零近距离单元，直流不会产生虚假目标。
  加窗（Hann/Hamming/Blackman）抑制谱泄漏；`ThresholdDB` 提供绝对信噪比下限。
- **密集目标**：检测要求 8 邻域局部极大值，相邻峰在合并前各自独立检出；
  合并半径（`MaxRangeBins`/`MaxDopplerBins`）小于目标间距时不会被错误合并。
- **可复现**：FFT 使用确定性的基-2 Cooley–Tukey（固定顺序的蝶形运算）；
  仿真噪声使用内置 PCG32，与全局随机源无关；检测、合并、关联、输出均做了
  确定性排序。相同种子与相同输入下输出逐条一致（浮点运算顺序固定，
  通常 `==` 一致；跨平台比较建议使用下文容差）。

## 信号模型与物理轴

单目标复基带信号：

```
x[p,k] = A exp(j 2π (fb·k/fs + fd·(p·PRI + k/fs)))
fb = slope·2R/c,  fd = 2v/λ,  λ = c/fc
```

- 距离轴：`r[k] = k · fs/Nr · c/(2·slope)`，只搜索 0..Nyquist（`0..Nr/2`）。
- 速度轴：FFT bin 顺序，多普勒中心频率 `1/(Nd·PRI)`，
  不模糊速度 `λ/(2·PRI)`，超出部分按 2π 折叠。

## 参数说明

| 配置 | 字段 | 说明 |
| --- | --- | --- |
| `ParseConfig` | `Min/Max*` | 采样率、斜率、脉冲数、采样点数、载频、PRI 的上下限（0 表示不校验该侧） |
| | `ExpectedPulses/ExpectedSamples` | >0 时要求帧严格匹配，不匹配返回 `ParseParameterMismatch` |
| `WindowConfig` | `RangeWindow/DopplerWindow` | `rect`/`hann`/`hamming`/`blackman` |
| | `RemoveDC` | 快时间均值相消 + 距离 FFT 后慢时间零多普勒相消 |
| | `RangePadTo/DopplerPadTo` | 目标 FFT 长度，自动向上取 2 的幂；小于输入长度报错 |
| `CFARConfig` | `GuardRange/Doppler` | 保护单元数（不参与噪声估计） |
| | `TrainRange/Doppler` | 每侧训练单元数 |
| | `Alpha` | 噪声基底乘子，判据 `power > Alpha·meanTrainPower` |
| | `ThresholdDB` | 绝对信噪比下限（dB） |
| | `SkipDCBins` | 从第几个距离单元开始搜索（排除零近直流残余） |
| | `MinTrainingCells` | 边缘训练单元不足时跳过（默认 4） |
| `ClusterConfig` | `MaxRangeBins/MaxDopplerBins` | 相邻峰合并半径（bin，多普勒按环形距离） |
| `TrackerConfig` | `AssocRange/AssocVelocity` | 归一化关联门限（米 / 米每秒） |
| | `ConfirmCount` | 连续命中多少次才输出新目标 |
| | `HoldFrames` | 未匹配保持帧数，超期删除 |
| | `WindowSize` | 输出值滑窗平均长度（内存有界） |

拒绝原因（`ParseErrorKind`）：`ParseEmptyFrame`、`ParseBadSampleRate`、
`ParseBadChirpSlope`、`ParseBadPulseCount`、`ParseBadSamplesPerPulse`、
`ParseBadCarrierFreq`、`ParseBadPRI`、`ParsePulseCountMismatch`、
`ParseSampleCountMismatch`、`ParseParameterMismatch`。

## 检查点与恢复语义

- 检查点内容：`{"last_completed_frame": N}`，JSON 文件。
- 写入方式：同目录临时文件 `fsync` 后原子 `rename`，崩溃不会留下半帧状态。
- **先落盘后返回**：一帧从解析到 tracker 更新全部完成后才写检查点；
  返回结果即表示该帧已提交。
- 重启后：
  - `id <= LastCompletedFrame` 的帧返回/跳过 `ErrFrameStale`（不重复输出）；
  - 期望 `id == LastCompletedFrame+1`，跳号/回退返回 `ErrFrameOutOfOrder`（不静默跳过）；
  - 被拒绝的非法帧同样推进检查点（它已被消费并裁决）。
- 内存：tracker 仅保存存活航迹及定长滑窗，谱缓冲每帧分配并释放，
  连续处理的常驻内存不随已处理帧数增长。

注意：当前检查点只持久化进度，不持久化航迹内部状态；重启后 tracker
从空状态开始，已有航迹需重新经过 `ConfirmCount` 确认。帧级 exactly-once
语义（不重复、不跳帧、无半帧）完整保证。

## 精度边界与容差

- FFT：2 的幂使用 radix-2 Cooley–Tukey；非 2 的幂回退朴素 DFT（较慢）。
  补零长度始终取 2 的幂。
- 距离分辨率约 `fs/Ns · c/(2·slope)`（每 bin），多普勒分辨率 `1/(Np·PRI)`；
  目标参数落在 bin 之间时存在最多半个 bin 的量化误差，合并质心与滑窗平均
  会减小该误差。
- 建议自动化比较容差：距离 ≤ 1.5 个距离 bin，速度 ≤ 1.2 个多普勒 bin，
  SNR 幅度 ≤ 1 dB；严格确定性比较（同机同二进制）可直接 `==`。
- 幅度为谱幅度（非 dB）；`SNRDB = 10·log10(Pcell / mean(Ptrain))`。

## 运行与测试

```bash
go test ./...
go run ./cmd/radar -frames 8 -checkpoint checkpoint.json
```

演示程序从内存管道喂入确定性仿真帧，打印每帧检测与确认目标。
删除/更换检查点文件即重新开始；复用检查点则从上次位置续算。

## 测试覆盖

- 多目标检测、不同信噪比（弱/中/强目标）
- 直流偏置下零近距无虚假目标
- 密集目标可区分（相邻目标不被合并）
- 全部非法参数拒绝原因与期望维度不匹配
- 加窗/补零配置
- 跨帧确认、保持期、超期删除、连续命中重置、关联门限
- 中断重启：不重复、不跳帧、跳号报错、拒绝帧推进检查点
- 相同种子逐条输出一致（重复运行 5 次）
- 长序列处理内存有界
