# PPP-B2b 非侵入式端到端实验工具

本工具只读取现有 RTKLIB 可执行程序、配置和数据，不修改 `rtklib/src/`、原始 RINEX、B2b raw 或历史输出。每次运行都创建一个此前不存在的实验目录；若目录已存在会立即停止，绝不覆盖。

README、实验报告、阻塞报告、总控提示和日志包装文字使用中文。为保持自动化接口稳定，`summary.csv`、`summary.json`、`preflight.json` 和 `mode_result.json` 的字段名，以及 `completed`、`blocked`、`unavailable` 等机器状态值继续使用英文；中文报告会同时给出可读说明。

## 1. 模式

| 模式 | 生成配置 | 当前状态 |
|---|---|---|
| M0 | `pos1-sateph=brdc`，关闭 B2b 文件/格式 | 可运行 |
| M1 | B2b 轨道/钟差开启、码偏差关闭 | 当前主程序没有独立码偏差开关，因此 fail-fast 为 `blocked` |
| M2 | `pos1-sateph=brdc+b2b-apc`，显式 B2b 格式和 raw 路径 | 可运行 |
| M3 | `pos1-sateph=precise`，关闭 B2b，输入 SP3/CLK | 可选；有 SP3/CLK 时可运行 |

M1 不会复用或改名 M2。只有主程序和配置解析器真正提供独立的 code-bias 开关后，才能把 M1 变为可运行模式。

## 2. 快速开始

在工作区根目录的 PowerShell 中运行：

```powershell
$rtklib = (Resolve-Path .\rtklib).Path

& "$rtklib\scripts\pppb2b_validation\Invoke-PppB2bValidation.ps1" `
  -DataDirectory "$rtklib\data\20260523" `
  -ReferenceEcef @(-2160815.1967,4383231.0078,4084983.5530) `
  -StartTime '2026/05/23 00:10:00' `
  -EndTime '2026/05/23 00:40:00' `
  -Modes @('M0','M1','M2')
```

在该数据目录中，工具可唯一发现 MO、MN、Sino raw、SP3 和 CLK；默认配置优先使用 `conf/sino_b2b.conf`，默认可执行程序优先使用 `x64/Release/rnx2rtkp.exe`，其次使用 `x64/Debug/rnx2rtkp.exe`。如果候选不唯一，工具会要求显式路径，而不是猜测。

显式指定全部输入：

```powershell
& "$rtklib\scripts\pppb2b_validation\Invoke-PppB2bValidation.ps1" `
  -ObsPath "$rtklib\data\20260523\URUM00CHN_R_20261430000_24H_30S_MO.rnx" `
  -NavPath "$rtklib\data\20260523\URUM00CHN_R_20261430000_24H_30S_MN.rnx" `
  -B2bPath "$rtklib\data\20260523\URUM00CHN_20260523_B2b_PPP.txt" `
  -B2bFormat sino `
  -ConfigTemplate "$rtklib\conf\sino_b2b.conf" `
  -ExecutablePath "$rtklib\x64\Debug\rnx2rtkp.exe" `
  -ReferenceEcef @(-2160815.1967,4383231.0078,4084983.5530) `
  -StartTime '2026/05/23 00:10:00' `
  -EndTime '2026/05/23 00:40:00' `
  -Modes @('M0','M2')
```

加入精密产品对照组：

```powershell
& "$rtklib\scripts\pppb2b_validation\Invoke-PppB2bValidation.ps1" `
  -DataDirectory "$rtklib\data\20260523" `
  -ReferenceEcef @(-2160815.1967,4383231.0078,4084983.5530) `
  -Modes @('M0','M2','M3')
```

## 3. 更换数据

- 使用 `-DataDirectory` 时，MO、MN、B2b raw、SP3、CLK 必须在该目录内可唯一发现。
- 候选不唯一时分别使用 `-ObsPath`、`-NavPath`、`-B2bPath`、`-Sp3Path`、`-ClkPath`。
- 使用 `-B2bFormat auto|unicore|sino` 指定或检测接收机格式。Sino 二进制 `.txt` 不会按文本处理。
- 配置中的非空 `file-*` 输入也会检查存在性并写入 SHA256 清单；输出型 `file-tracefile`、`file-solstatfile` 和 `file-tempdir` 除外。
- 自动发现只做确定性选择。发现零个或多个同优先级候选时 fail-fast。

## 4. 设置参考坐标

推荐显式给出 ECEF XYZ（米）：

```powershell
-ReferenceEcef @(-2160815.1967,4383231.0078,4084983.5530)
```

若未给出，工具会使用观测 RINEX 头的 `APPROX POSITION XYZ`，并在报告中标为近似坐标，不能据此宣称厘米级精度。参考坐标只用于离线 ENU/误差统计，不改写生成配置或 RTKLIB 解算初值。

## 5. 输出目录

默认根目录为：

```text
rtklib/pppb2b_validation_outputs/<timestamp>_<random>/
```

可以通过 `-OutputRoot` 更换根目录，通过 `-ExperimentId` 指定实验名。实验名已存在时停止，不覆盖、不删除。

```text
<experiment>/
  preflight.json
  inputs_sha256.csv
  summary.csv
  summary.json
  experiment_report.md
  reproduce.ps1
  BLOCKED.md                 # 仅在至少一个模式阻塞时存在
  M0|M1|M2|M3/
    generated.conf           # 仅对可真实表示的模式生成
    command.ps1
    run.stdout.log
    run.stderr.log
    run.log
    solution.pos
    solution.pos.trace       # 当前可执行程序实际产生时保留
    mode_result.json
    BLOCKED.md               # 该模式未运行时存在
```

`inputs_sha256.csv` 包含 MO、MN、B2b raw、配置模板、可执行程序、SP3/CLK，以及配置引用的 ATX/BLQ 等实际输入。每个模式的生成配置 SHA256 写在 `mode_result.json`。

## 6. 指标口径

- 处理历元数：请求时间窗内的 RINEX 观测历元数。
- 有效解历元数：可解析 POS 数据行数；解状态比例按 POS 的 `Q` 字段统计。
- E/N/U RMS：POS ECEF 相对参考 ECEF 转换后的分量 RMS。
- P68、P95、最大误差：三维 ENU 误差范数的百分位和最大值。
- 卫星数：POS `ns` 字段的最小值、平均值和最大值。
- 首次收敛：默认要求三维误差不大于 0.10 m，并连续保持 10 个观测历元；通过 `-ConvergenceThresholdM` 和 `-ConvergenceHoldEpochs` 修改。没有达到时写 `not_reached`。
- B2b 缺失、过期和 IOD 不匹配：只统计 trace/run log 中明确存在的文本标记。没有对应标记时写 `unavailable_no_explicit_log_marker`，不把“未打印”猜成 0。
- B2b 预检中的 Type 1–4 数量是通过外层 CRC 的 raw 帧数，只证明输入包含四类电文，不替代 receiver MASK/IOD 门后的发布计数或 PPP 验收。

## 7. 重复实验

实验根目录中的 `reproduce.ps1` 保存等价总控命令；每个模式的 `command.ps1` 保存实际 `rnx2rtkp` 命令。重复实验会创建新的唯一目录，不复用旧结果。

运行前后可用 `inputs_sha256.csv` 核对数据和可执行程序是否完全一致。更换任一数据、配置模板或 EXE 都应视为新实验。

## 8. 判断失败

- 总控退出码 2：预检或汇总层 fail-fast；查看实验根 `BLOCKED.md`。
- `summary.json.overall_status=BLOCKED`：所有请求模式都无法安全运行。
- `PARTIAL`：至少一个模式完成，同时至少一个模式阻塞或失败；当前默认包含 M1 时会出现此状态。
- `FAILED`：主程序显式报错、退出码非零、POS 不存在或不是可解析 XYZ 格式。
- `completed_no_solution`：主程序完成且没有显式错误，但没有有效解；不能视为定位成功。
- `unavailable`：现有输出格式或日志没有该指标，工具没有猜测。

## 9. 增加模式

新增模式必须同时完成：

1. 在 PowerShell 参数 `ValidateSet` 和 `$ModeDescriptions` 中注册模式；
2. 明确生成配置相对模板只改变哪些键；
3. 明确必需输入和 preflight 阻塞条件；
4. 证明现有主程序能真实区分该模式；
5. 将模式结果继续交给同一 Python `summarize`/`aggregate` 路径；
6. 增加独立 smoke test，不能用已有模式改名代替。

如果新增模式要求改动 `rtklib/src/`，应作为独立、经授权的算法/配置入口任务处理，不应在本工具中暗中实现。
