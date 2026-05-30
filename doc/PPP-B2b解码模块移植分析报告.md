# PPP-B2b 解码模块移植分析报告

生成时间：2026-05-26

本报告基于 `D:\Desktop\rtklib_b2b\RTKLIB-B2b` 参考工程源码分析生成，目标是辅助后续将 PPP-B2b 解码、译码和 SSR 改正数更新能力迁移到另一个 RTKLIB-B2b 主工程中。

本次分析只涉及源码阅读，没有修改、删除或重构任何工程源码。

## 1. 工程总体结论

这个参考工程的 PPP-B2b 能力不是一个完全独立的小模块，而是分成三层：

1. 接收机私有协议解码层
   - 主要文件：`src/rcv/sinan.c`、`src/rcv/unicore.c`
   - 作用：把 SinoGNSS / Unicore 的 PPP-B2b 原始帧解成 B2b SSR 改正数。

2. B2b SSR 数据结构与更新层
   - 主要文件：`include/rtklib.h`、`include/B2b.h`、`src/B2b.c`、`src/rcvraw.c`、`src/postpos.c`、`src/rtksvr.c`
   - 作用：保存 mask、orbit、clock、code bias，并把 `raw->nav.B2bssr[]` 更新到主 `nav.B2bssr[]`。

3. PPP 解算使用层
   - 主要文件：`src/ephemeris.c`、`src/ppp.c`
   - 作用：在 `sateph=5` / `EPHOPT_B2b` 时使用 B2b 轨道、钟差和码偏差。

如果目标是先移植 PPP-B2b 解码/译码能力，不建议直接搬整个 `ppp.c`、`postpos.c`、`rtksvr.c`。更合理的做法是先移植 B2b 数据结构、receiver decoder、raw 分发和 nav 更新函数，再逐步接入 PPP 解算。

## 2. PPP-B2b 相关文件清单

| 文件路径 | 作用 | 是否必须移植 | 原因 |
|---|---|---:|---|
| `include/rtklib.h` | 增加 `EPHOPT_B2b`、`STRFMT_SINO`、`STRFMT_UNICORE`、`B2bmask_t`、`B2bssr_t`、`B2b_t`、`nav_t.B2bssr[]`、`prcopt_t.B2b_format`、`raw_t` B2b 计数器 | 是，按字段移植 | 所有 B2b 解码结果和后续 PPP 使用都依赖这些结构和宏 |
| `include/B2b.h` | B2b 辅助函数声明：`slot2satno()`、`mask2satno()`、`B2btod2time()`、`checkout_B2b*()`、`output_B2bInfo*()` | 是 | decoder、postpos、rtksvr、trace 输出都依赖 |
| `src/B2b.c` | 时间转换、B2b slot 到 RTKLIB satno 映射、mask 生成、改正数一致性检查、B2b trace 输出 | 是 | 是 PPP-B2b 解码后的通用支撑层 |
| `src/rcv/sinan.c` | SinoGNSS/司南 PPP-B2b 私有帧解码 | 视接收机而定 | 需要支持司南接收机时必须移植 |
| `src/rcv/unicore.c` | Unicore/和芯星通 PPP-B2b 私有帧解码 | 视接收机而定 | 需要支持 Unicore 接收机时必须移植 |
| `src/rcvraw.c` | 在 `input_raw()` / `input_rawf()` 中分发 `STRFMT_SINO`、`STRFMT_UNICORE` | 是，按分发点移植 | 没有它，原始字节进不了 B2b decoder |
| `src/postpos.c` | 事后 PPP 中读取 `.b2b/.B2b` 文件，并更新 `navs.B2bssr[]` | 部分移植 | 需要事后处理时移植 `update_B2b_ssr()` 思路，不建议整文件复制 |
| `src/rtksvr.c` | 实时服务中把 `raw->nav.B2bssr[]` 更新到 `svr->nav.B2bssr[]` | 部分移植 | 实时接入必须有同类更新逻辑，但不建议整文件复制 |
| `src/ephemeris.c` | `satpos_B2b()` / `satpos_B2b_otp()`，使用 B2b orbit/clock 改正卫星位置钟差 | 后续接 PPP 时必须 | 解算层耦合深，建议按函数/分支移植 |
| `src/ppp.c` | `corr_meas()` 中使用 `nav->B2bssr[obs->sat].cbias[]` 修正伪距 | 后续接 PPP 时必须 | 只需要移植 B2b 分支，不建议整文件复制 |
| `src/options.c` | 读取 `prcopt.B2b_format`、`prcopt.sateph` 等配置 | 视目标工程而定 | 目标工程配置系统不同的话，应写适配 |
| `src/trace.c` | B2b SSR 独立 trace 输出 | 可选 | 验证解码结果很有用，但不是核心解码必需 |
| `app/postdecoder/postdecoder.c` | 事后 B2b 文件解码 demo | 不建议直接移植 | 可作为最小测试程序参考 |
| `app/rtdecoder/rtdecoder.c` | 实时 B2b 解码 demo/server | 不建议直接移植 | 含大量 app/server 逻辑，可参考其 `update_B2bssr()` |
| `app/postppp/postppp.c` | 事后 PPP app 入口，设置 `PPP_Glo.b2b_flag` | 不建议直接移植 | app 包装层，和目标工程入口耦合 |
| `app/rtppp/rtapp.c` | 实时 PPP app 入口 | 不建议直接移植 | 只是调用 `app_rtkrcv()` |
| `src/rtcm.c` / `src/rtcm3.c` | 标准 RTCM/SSR 解码 | 不建议作为 B2b 移植项 | PPP-B2b 私有帧解码不走 RTCM，除非目标工程还要保留常规 SSR |
| `src/stream.c` | 串口/socket/file 流接口 | 不建议直接移植 | 输入层通用，目标工程通常已有自己的 stream 接口 |
| `src/solution.c` | 解算结果输出 | 暂不需要 | 和 B2b 解码无直接关系 |

## 3. 原始数据输入链路

### 3.1 事后独立解码：`postdecoder`

```text
app/postdecoder/postdecoder.c::main()
  -> 解析 -U / -S / -in / -out
  -> init_raw(&raw0, STRFMT_SINO 或 STRFMT_UNICORE)
  -> fopen(B2bfilepath, "rb")
  -> input_rawf(&raw0, format, B2bfp)
  -> src/rcvraw.c::input_rawf()
  -> input_sinof() 或 input_unicoref()
  -> input_sino() 或 input_unicore()
  -> decode_sino() 或 decode_unicore()
  -> decode_B2b() 或 decode_PPPPB2BINFO1/2/3/4()
  -> raw0.nav.B2bssr[satno].*
  -> navs.B2bssr[i] = raw0.nav.B2bssr[i]
```

关键位置：

- `app/postdecoder/postdecoder.c::main()`：约第 26 行
- `src/rcvraw.c::input_rawf()`：约第 1449 行
- `src/rcv/sinan.c::input_sinof()`：约第 627 行
- `src/rcv/unicore.c::input_unicoref()`：约第 709 行

### 3.2 事后 PPP：`postppp`

```text
app/postppp/postppp.c::main()
  -> load_config()
  -> prcopt.sateph == EPHOPT_B2b 时 PPP_Glo.b2b_flag = 1
  -> postpos()
  -> src/postpos.c 识别 .b2b / .B2b 输入文件
  -> init_raw(&B2braw, prcopt->B2b_format)
  -> 每个观测历元调用 update_B2b_ssr(obs[0].time, format)
  -> input_rawf(&B2braw, format, fp_B2b)
  -> receiver decoder
  -> navs.B2bssr[i] = B2braw.nav.B2bssr[i]
  -> satpos_B2b() / corr_meas() 使用 navs.B2bssr[]
```

关键位置：

- `app/postppp/postppp.c`：`prcopt.sateph == 5` 时设置 `PPP_Glo.b2b_flag`
- `src/postpos.c`：识别 `.b2b` / `.B2b` 输入文件
- `src/postpos.c::update_B2b_ssr()`：事后模式 B2b SSR 更新入口

### 3.3 实时 PPP：`rtppp`

```text
app/rtppp/rtapp.c::main()
  -> app_rtkrcv()
  -> rtksvr stream read
  -> src/rtksvr.c::decoderaw()
  -> input_raw(svr->raw + index, svr->format[index], byte)
  -> input_sino() 或 input_unicore()
  -> decode_sino() 或 decode_unicore()
  -> ret == 20
  -> update_svr()
  -> update_B2bssr(svr, index)
  -> svr->nav.B2bssr[i] = raw->nav.B2bssr[i]
  -> 实时 PPP 解算使用 svr->nav.B2bssr[]
```

关键位置：

- `app/rtppp/rtapp.c::main()`
- `src/rtksvr.c::decoderaw()`
- `src/rtksvr.c::update_B2bssr()`

## 4. B2b message 解码链路

### 4.1 SinoGNSS / 司南

入口：

- `src/rcv/sinan.c::input_sino()`
- `src/rcv/sinan.c::decode_sino()`
- `src/rcv/sinan.c::decode_B2b()`

`decode_B2b()` 中读取 B2b message type：

```text
decode_B2b()
  -> 读取 prn_32 / prn_6 / mes_type
  -> switch (mes_type)
     1 -> process_message_type_1()
     2 -> process_message_type_2()
     3 -> process_message_type_3()
     4 -> process_message_type_4()
```

| B2b message | 内容 | SinoGNSS 解码函数 |
|---|---|---|
| Type 1 | MASK | `process_message_type_1()` |
| Type 2 | Orbit + URA | `process_message_type_2()` |
| Type 3 | Code Bias | `process_message_type_3()` |
| Type 4 | Clock | `process_message_type_4()` |

### 4.2 Unicore / 和芯星通

入口：

- `src/rcv/unicore.c::input_unicore()`
- `src/rcv/unicore.c::decode_unicore()`

`decode_unicore()` 根据消息 ID 分发：

| Unicore message ID | 内容 | 解码函数 |
|---|---|---|
| `2302` | PPPB2BINFO1 / MASK | `decode_PPPPB2BINFO1()` |
| `2304` | PPPB2BINFO2 / Orbit + URA | `decode_PPPPB2BINFO2()` |
| `2306` | PPPB2BINFO3 / Code Bias | `decode_PPPPB2BINFO3()` |
| `2308` | PPPB2BINFO4 / Clock | `decode_PPPPB2BINFO4()` |

### 4.3 改正类型汇总

| 改正类型 | 解码函数 | 输入 | 输出结构体/变量 | 后续使用 |
|---|---|---|---|---|
| MASK | `process_message_type_1()` / `decode_PPPPB2BINFO1()` | receiver raw payload | `sinan_mask` / `unicore_mask`，类型 `B2bmask_t` | 后续 orbit/code/clock 根据 mask 和 slot 找 satno |
| 轨道改正 | `process_message_type_2()` / `decode_PPPPB2BINFO2()` | B2b message type 2 | `raw->nav.B2bssr[satno].deph[]`、`iodn`、`iodcorr[0]`、`ura`、`t0[0]` | `update_B2bssr()` 后进入 `nav.B2bssr[]`，由 `satpos_B2b()` 使用 |
| 钟差改正 | `process_message_type_4()` / `decode_PPPPB2BINFO4()` | B2b message type 4 | `raw->nav.B2bssr[satno].dclk[0]`、`iodcorr[1]`、`t0[2]` | `satpos_B2b()` 改正卫星钟差 |
| 码偏差 | `process_message_type_3()` / `decode_PPPPB2BINFO3()` | B2b message type 3 | `raw->nav.B2bssr[satno].cbias[code]`、`t0[1]` | `ppp.c::corr_meas()` 修正伪距 |
| 相位偏差 | 未发现 PPP-B2b 解码实现 | 无 | `B2bssr_t` 中没有 `pbias` 字段 | 当前 B2b 分支不使用 phase bias |
| URA | Type 2 | B2b orbit message | `B2bssr_t.ura` | `var_uraB2b()` / `satpos_B2b()` |
| IOD SSR | Type 1/2/3/4 | B2b message header | `B2bmask_t.IOD_SSR`、`B2bssr_t.iodssr[]` | update 阶段一致性检查 |
| IOD Corr | Type 2/4 | orbit/clock message | `B2bssr_t.iodcorr[0/1]` | orbit/clock 配套检查 |
| Update interval | update 阶段推算 | 相邻 `t0[]` 时间差 | `B2bssr_t.udi[]` | `satpos_B2b()` 判断 age |

## 5. 解码结果数据结构

### 5.1 `B2bmask_t`

定义位置：`include/rtklib.h`

关键字段：

| 字段 | 含义 |
|---|---|
| `time` | 当前 mask 对应时间 |
| `MASK_BD[63]` | BDS mask |
| `MASK_GPS[37]` | GPS mask |
| `MASK_GALILEO[37]` | Galileo mask |
| `MASK_GLONASS[37]` | GLONASS mask |
| `IOD_SSR` | mask 消息的 IOD SSR |
| `IODP` | mask IODP |
| `satnum` | 当前 mask 有效卫星数 |
| `satno[B2B_MAXSAT]` | B2b slot 映射到 RTKLIB satno 后的结果 |

移植建议：必须保留，但可封装为 decoder context，避免继续使用 `sinan_mask` / `unicore_mask` 这种文件级全局变量。

### 5.2 `B2bssr_t`

定义位置：`include/rtklib.h`

关键字段：

| 字段 | 含义 |
|---|---|
| `sow` | B2b 消息中的秒内时间 |
| `verify_sow` | 用于检查/输出的 SOW |
| `t0[6]` | 各类改正数参考时间：orbit、cbias、clock、ura 等 |
| `udi[6]` | 各类改正数 update interval |
| `iodssr[6]` | 各类消息的 IOD SSR |
| `iodp[2]` | mask IODP |
| `iodn` | 广播星历 IODN / IODC 关联字段 |
| `iodcorr[4]` | orbit/clock 改正数匹配用 IOD Corr |
| `deph[3]` | 轨道径向/切向/法向改正，单位 m |
| `ddeph[3]` | 轨道改正变化率，目前 B2b 主流程基本未用 |
| `ura` | B2b URA index |
| `cbias[MAXCODE]` | 不同信号码偏差，单位 m |
| `dclk[3]` | 钟差改正，目前主要用 `dclk[0]` |
| `update` | decoder 写入新数据后的更新标志 |

移植建议：这是核心结构体，必须同步。但要特别注意这个工程中经常使用 `nav->B2bssr[sat]`，不是标准 RTKLIB 常见的 `sat-1`。

### 5.3 `B2b_t`

定义位置：`include/rtklib.h`

保存 B2b decoder 状态、消息计数和 `B2bssr[MAXSAT]`。当前主路径更多使用 `raw_t.nav.B2bssr[]`，`B2b_t` 更像早期/独立控制结构。移植时可作为参考，不一定要直接暴露到目标主工程 API。

### 5.4 `nav_t`

定义位置：`include/rtklib.h`

新增关键字段：

```c
B2bssr_t B2bssr[MAXSAT];
```

作用：主导航数据中保存 PPP-B2b SSR 改正数。后续 `ephemeris.c` 和 `ppp.c` 都直接读取它。

### 5.5 `raw_t`

定义位置：`include/rtklib.h`

B2b 新增字段：

| 字段 | 含义 |
|---|---|
| `geoprn` | 当前 B2b GEO PRN |
| `num_PPPB2BINF01` | mask 消息计数 |
| `num_PPPB2BINF02` | orbit 消息计数 |
| `num_PPPB2BINF03` | code bias 消息计数 |
| `num_PPPB2BINF04` | clock 消息计数 |
| `raw_nmsg[16]` | raw 消息统计 |

初始化位置：`src/rcvraw.c::init_raw()`

### 5.6 `prcopt_t`

定义位置：`include/rtklib.h`

新增字段：

```c
int B2b_format;
```

作用：指定 B2b 原始数据格式，`20` 为 SinoGNSS，`21` 为 Unicore。

## 6. SSR/B2b 改正数更新链路

### 6.1 事后更新

核心函数：`src/postpos.c::update_B2b_ssr()`

```text
update_B2b_ssr(time, format)
  -> 打开/切换 B2b 文件
  -> input_rawf(&B2braw, format, fp_B2b)
  -> decoder 写 raw->nav.B2bssr[]
  -> 检查 duplicate / IOD / update interval
  -> navs.B2bssr[i] = B2braw.nav.B2bssr[i]
  -> 清 raw update flag 和消息计数
```

核心写入：

```c
navs.B2bssr[i] = B2braw.nav.B2bssr[i];
```

### 6.2 实时更新

核心函数：`src/rtksvr.c::update_B2bssr()`

```text
update_svr(... ret == 20 ...)
  -> update_B2bssr(svr, index)
  -> 检查 orbit/code/clock 的时间、IOD、IODN
  -> svr->nav.B2bssr[i] = raw->nav.B2bssr[i]
  -> raw->nav.B2bssr[i].update = 0
```

核心写入：

```c
nav->B2bssr[i] = raw->nav.B2bssr[i];
```

`app/rtdecoder/rtdecoder.c` 中也有一份类似逻辑。移植时不建议重复保留两份，应在目标工程中抽出一个统一的 B2b SSR update helper。

## 7. 和 PPP 解算的耦合点

### 7.1 `sateph=5`

新增枚举：

```c
#define EPHOPT_B2b 5
```

含义：广播星历 + B2b SSR APC。

相关配置：

- `prcopt.sateph = 5`
- `prcopt.B2b_format = 20`：SinoGNSS
- `prcopt.B2b_format = 21`：Unicore

### 7.2 轨道钟差

核心函数：

- `src/ephemeris.c::satpos_B2b()`
- `src/ephemeris.c::satpos_B2b_otp()`

调用点：

- `src/ephemeris.c::satpos()` 中 `EPHOPT_B2b` 分支
- `src/ephemeris.c::satpos_otp()` 中 `EPHOPT_B2b` 分支

注意点：

- 函数内部使用 `nav->B2bssr + sat`
- 这里不是 `sat-1`
- 目标工程移植时必须统一索引约定

### 7.3 码偏差

核心位置：`src/ppp.c::corr_meas()`

B2b 分支使用：

```c
P[i] -= nav->B2bssr[obs->sat].cbias[ix];
```

当前没有看到 PPP-B2b phase bias 的解码或使用。`ssr_t` 里有 `pbias`，但 B2b 的 `B2bssr_t` 没有 `pbias` 字段，B2b PPP 分支主要用 orbit、clock、code bias。

## 8. 最小可移植模块清单

### 8.1 A 类：必须直接移植或等价实现

| 模块 | 建议 |
|---|---|
| B2b 宏和结构体 | 从 `include/rtklib.h` 中移植 `EPHOPT_B2b`、`STRFMT_SINO`、`STRFMT_UNICORE`、`B2bmask_t`、`B2bssr_t`、`nav_t.B2bssr[]`、`raw_t` B2b 字段、`prcopt_t.B2b_format` |
| B2b helper API | 移植 `include/B2b.h` |
| B2b helper 实现 | 移植 `src/B2b.c` 中时间转换、`slot2satno()`、`mask2satno()`、`checkout_B2b*()` |
| receiver decoder | 按接收机选择移植 `src/rcv/sinan.c` 和/或 `src/rcv/unicore.c` |
| raw 分发 | 在目标 `rcvraw.c` 中增加 `STRFMT_SINO/UNICORE` 的 `input_raw()` / `input_rawf()` 分发 |
| raw 初始化 | 在 `init_raw()` 中初始化 `geoprn` 和 `num_PPPB2BINF01..04` |
| B2b SSR update | 从 `postpos.c::update_B2b_ssr()` 和 `rtksvr.c::update_B2bssr()` 提炼统一更新逻辑 |
| 最小测试程序 | 参考 `app/postdecoder/postdecoder.c` 做一个只读 B2b 文件并输出 SSR 的小程序 |

### 8.2 B 类：不建议直接移植，应该写适配层

| 部分 | 原因 |
|---|---|
| `nav_t` / `raw_t` 整体定义 | 目标工程可能已有差异，建议只增量合并字段 |
| `trace.c` B2b trace 系统 | 可用回调或日志接口替代，避免侵入目标工程 trace |
| `postpos.c` 整文件 | 和 RTKLIB 事后处理流程耦合深，只提取 B2b 文件读取和 nav 更新逻辑 |
| `rtksvr.c` 整文件 | 实时 server、stream、线程状态耦合深，只提取 `update_B2bssr()` 逻辑 |
| `options.c` 整文件 | 配置系统可能不同，只保证目标工程能设置 `sateph=5` 和 `B2b_format` |
| `stream.c` | 目标工程应复用自己的串口/socket/file 输入 |
| app 入口 | `postdecoder` 可做测试参考，`rtdecoder/postppp/rtppp` 不应直接作为目标工程入口 |

### 8.3 C 类：暂时不要动

| 部分 | 原因 |
|---|---|
| PPP 主滤波 / Kalman | 解码移植阶段不需要改 |
| 残差整体模型 | 先只接 `cbias`，不要扩大修改范围 |
| `solution.c` | 和 B2b 解码无直接关系 |
| GUI / demo app | 非核心 |
| RTCM 标准 SSR 解码 | PPP-B2b 私有帧不是 RTCM，先不要混在一起改 |

## 9. 不建议直接移植的高风险部分

1. `ppp.c` 整文件
   - `ppp.c` 是解算核心。
   - 当前 B2b 修改点主要集中在 `corr_meas()` 的 B2b code bias 分支。
   - 不建议整文件覆盖目标工程。

2. `ephemeris.c` 整文件
   - B2b 主要新增 `satpos_B2b()`、`satpos_B2b_otp()` 和 `EPHOPT_B2b` 分支。
   - 目标工程若已有自己的星历/SSR 逻辑，应按函数合并。

3. `postpos.c` / `rtksvr.c` 整文件
   - 二者只应提取 B2b SSR update 链路。
   - 整文件复制容易破坏目标工程已有后处理/实时框架。

4. B2b SSR 索引约定
   - 当前工程多处使用 `nav->B2bssr[sat]`、`raw->nav.B2bssr[satno]`。
   - 这不是标准 RTKLIB 常见的 `sat-1`。
   - 这是移植时最容易引入隐性错误的位置。

5. Unicore code bias 内存处理
   - `decode_PPPPB2BINFO3()` 中有动态分配 `StCodeBias_t` 的逻辑。
   - 移植时建议改成目标工程认可的缓冲区管理方式，避免泄漏或越界。

## 10. 给目标主工程的移植步骤建议

1. 先移植数据结构
   - 增加 `B2bssr_t`
   - 增加 `B2bmask_t`
   - 增加 `nav_t.B2bssr[]`
   - 增加 `raw_t` B2b 消息计数器
   - 增加 `prcopt_t.B2b_format`
   - 增加 `EPHOPT_B2b`
   - 增加 `STRFMT_SINO` / `STRFMT_UNICORE`

2. 增加 B2b helper
   - 新增 `B2b.h` / `B2b.c` 或放入目标工程已有 GNSS helper 模块。
   - 优先保证以下函数可用：
     - `slot2satno()`
     - `mask2satno()`
     - `B2btod2time()`
     - `checkout_B2beph()`
     - `checkout_B2bcbia()`
     - `checkout_B2bclk()`

3. 接入 receiver decoder
   - 只需要和芯星通：移植 `src/rcv/unicore.c`
   - 只需要司南：移植 `src/rcv/sinan.c`
   - 两种都需要：二者都移植，但建议共享统一的 B2b update 层

4. 接入 `input_raw()` 分发
   - 在目标 `rcvraw.c` 中加入：

```text
STRFMT_UNICORE -> input_unicore() / input_unicoref()
STRFMT_SINO    -> input_sino() / input_sinof()
```

5. 先做最小 postdecoder 测试
   - 目标不是 PPP 解算，而是验证：
     - mask 是否正确输出
     - orbit `deph[]` 是否更新
     - clock `dclk[0]` 是否更新
     - code bias `cbias[]` 是否更新
     - `iodssr` / `iodcorr` / `iodn` 是否合理

6. 再接 nav 更新层
   - 从 `update_B2b_ssr()` / `update_B2bssr()` 提炼一个目标工程函数，例如：

```text
b2b_update_nav_from_raw(nav_t *nav, raw_t *raw, gtime_t curtime)
```

7. 最后接 PPP
   - 先在 `ephemeris.c` 接 `EPHOPT_B2b -> satpos_B2b()`
   - 再在 `ppp.c::corr_meas()` 接 `nav->B2bssr[obs->sat].cbias[]`
   - 最后再处理 `postppp` / `rtppp` 配置和 app 入口

8. 构建系统
   - 目标 `CMakeLists.txt` / Makefile 至少新增：
     - `src/B2b.c`
     - `src/rcv/unicore.c` 或 `src/rcv/sinan.c`
     - 对应 include 路径
   - 若保留 B2b trace，还要确认 `trace.c` 中 B2b trace API 存在。

## 11. 后续需要人工确认的问题

1. 目标主工程中的 `nav_t.ssr[]`、`nav_t.B2bssr[]` 是否已有类似字段，避免重复定义。
2. 目标工程卫星编号是否和本工程 `satno()` 一致，尤其是 BDS/GPS/GAL/GLO 的范围。
3. 是否继续沿用 `nav->B2bssr[sat]` 索引，还是改成 `sat-1`；必须全工程统一。
4. 目标接收机是否只需要 Unicore/和芯星通，还是还要 SinoGNSS/司南。
5. B2b 原始数据来自独立文件、串口、NTRIP，还是和观测数据同一个 receiver raw stream。
6. 目标工程是否已有实时 SSR 更新机制；如果有，应把 B2b update 接成同类接口，不要再复制一套 server 逻辑。
7. 接入 PPP 前必须确认以下字段已经按历元正确写入 `nav.B2bssr[sat]`：
   - `deph[]`
   - `dclk[0]`
   - `cbias[]`
   - `iodn`
   - `iodcorr[]`
   - `iodssr[]`
   - `t0[]`
   - `udi[]`

