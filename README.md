# FMCW Radar Target Detection (Go)

本地调频连续波（FMCW）雷达信号处理与目标检测服务。输入为内存管道中逐帧投递的
仿真基带回波，输出每帧确认目标的距离、径向速度、幅度与信噪比，并支持跨帧关联、
检查点续算与确定性复现。

## 处理链路

```
内存 channel ([]byte)
   -> frame.Parse      自定界帧解析 + 帧头严格校验（可区分错误原因）
   -> signal           逐脉冲去直流 -> 加窗 -> 补零 -> 快时间 FFT（距离）
                        慢时间加窗 -> 补零 -> FFT + fftshift（多普勒）
   -> detect.Detect    二维 CA-CFAR（保护/训练单元、噪声基底、已判目标剔除）
   -> detect.Cluster   相邻超阈格 BFS 聚簇 + 抛物面亚 bin 插值 + 物理量换算
   -> track.Update     门限贪心关联、连续确认、保持期 coast、环形滑窗平滑
   -> checkpoint       结果投递成功后原子写入 (seq + tracker 状态)
```

每帧严格按“解析 -> 处理 -> 投递结果 -> 提交检查点”的顺序执行，因此不会留下半帧
结果；崩溃恢复后已提交帧绝不重复输出，未提交帧会重新处理。

## 包结构

- `pkg/frame` —— 线格式编解码与帧头校验
- `pkg/signal` —— 窗函数、直流消除、补零、迭代 radix-2 FFT 与二维谱
- `pkg/detect` —— 2D CA-CFAR 检测与谱峰聚簇
- `pkg/track` —— 跨帧关联、确认/保持/删除与滑窗平滑（含 JSON 状态序列化）
- `pkg/pipeline` —— 内存管道服务、原子文件检查点、序列号连续性校验
- `pkg/sim` —— 确定性 FMCW 仿真器（拍频 + 慢时间多普勒 + 复高斯噪声 + 直流偏置）
- `cmd/radardemo` —— 端到端演示

## 帧线格式（小端，自定界）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 4 | magic `"RDRF"` |
| 4 | 4 | `frameLen`，整帧总字节数（含头与载荷） |
| 8 | 1 | 协议版本（当前 1） |
| 9 | 1 | 标志位（保留，必须为 0） |
| 10 | 2 | `headerLen`（核心头长度，当前 48） |
| 12 | 48 | 核心头：`seq` int64；采样率、调频斜率、载频、PRI 各 float64；脉冲数、每脉冲采样数 uint32 |
| 60 | N | I/Q 载荷，`[pulse][sample]` 交错的 float64，共 `2 × 脉冲数 × 采样数 × 8` 字节 |

解析按 `frameLen` 精确消费，可在同一流上背靠背解析多帧。

### 帧头拒绝原因（`errors.Is` 可区分）

- `ErrBadMagic` / `ErrUnsupportedVersion`
- `ErrTruncatedHeader` / `ErrTruncatedPayload`
- `ErrInvalidSampleRate` / `ErrInvalidChirpSlope` / `ErrInvalidCarrierFreq` / `ErrInvalidPRI`
- `ErrInvalidPulseCount` / `ErrInvalidSamples`
- `ErrPayloadMismatch`（`frameLen` 与载荷维度不符）
- `ErrLimitExceeded`（脉冲数/采样数/帧字节数超过 `ParseConfig` 上限）

## 关键参数

- `signal.Config`：`Window`（none/hann/hamming/blackman）、`RemoveDC`、
  `RangeZeroPad`、`DopplerZeroPad`（0/1 表示不补零，2 表示 FFT 尺寸翻倍，向上取 2 的幂）。
- `detect.CFARConfig`：`GuardRange/Doppler`、`TrainRange/Doppler`、
  `AlphaDB`（相对噪声基底的 dB 门限）、`SkipRangeDC`（屏蔽距离 bin 0）、
  `SkipNearBins`（屏蔽最近 N 个 bin 的发射泄漏）。
- `detect.ClusterConfig`：`MaxRangeGap/DopplerGap`（允许桥接的间隔）、
  `MinCells`（小于该格数的簇丢弃）、`Interpolate`（抛物面亚 bin 插值）。
- `track.Config`：`RangeGate/VelocityGate`（椭圆门）、`ConfirmCount`
  （连续命中确认帧数）、`HoldFrames`（短暂丢失保持帧数）、`WindowLength`
  （尾相随滑窗长度）。
- `pipeline.Config.CheckpointPath`：为空时仅内存检查点；非空时 JSON 文件，
  临时文件 `fsync` + `rename` 原子落盘。

## 物理量换算

- 距离：`R = k · fs · c / (2 · S · N_range)`，其中 `k` 为（插值后）距离 bin，
  `S` 为调频斜率，`N_range` 为快时间 FFT 尺寸。只返回非负的单边距离 bin。
- 速度：多普勒 FFT 经 `fftshift` 后零速居中，
  `v = (d - N_dop/2) / N_dop / PRI · c / (2 · fc)`，正值表示远离。
- SNR：峰格功率相对其 CA-CFAR 训练窗平均功率的 dB 比。

## 精度与能力边界

- 距离分辨率约 `fs / N_range · c / (2S)`；速度分辨率约 `c / (2fc · N_pulses · PRI)`。
  加窗以旁瓣抑制换取主瓣展宽（Hann 主瓣约展宽一倍）。
- 最大无模糊距离受快时间 Nyquist 约束：`R_max = fs/2 · c / (2S)`。
  仿真拍频 `fb = 2SR/c - fd`，超带宽能量会折叠，参数必须使目标落在带内。
- 最大无模糊速度 `± c / (4 fc PRI)`；多普勒 bin 在 Nyquist 边界按周期环绕。
- FFT 为确定性迭代 radix-2，无并发、无 map 遍历依赖；固定种子与相同输入下，
  整条链路逐帧逐条输出一致。浮点比较建议使用相对/绝对容差
  （测试采用 `1e-9` 比较确定性复现；物理量验收使用分辨率量级的容差）。
- 内存只随“当前存活目标数 × 滑窗长度”和单帧谱尺寸增长，不随已处理帧数增长
  （有自动化测试守护）。

## 运行

```bash
go test ./...
go run ./cmd/radardemo -frames 12 -checkpoint ./cp.json
# 中断后用同一 -checkpoint 续跑，已提交帧不会重复输出
```

## 测试覆盖

- 多目标、密集目标可区分、不同信噪比、强直流偏置无虚警且不漏真实目标
- 非法帧头参数的各类可区分拒绝、截断/错误 magic/版本、背靠背流解析
- FFT 冲激/复指数、去直流、补零谱形与确定性复现
- 跟踪确认、丢失保持与删除、保持期内重获保持同一 ID、滑窗平滑
- 检查点原子落盘、崩溃恢复不重复不丢帧、序列号跳变拒绝、重复投递幂等
- 相同种子/输入下两次运行逐条一致；长序列处理内存不随帧数增长
