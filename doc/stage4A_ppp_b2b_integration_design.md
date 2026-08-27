# Stage 4A：PPP-B2b 改正接入设计

日期：2026-06-13

## 1. 本阶段范围

本文档定义 Stage 4 中 PPP-B2b 产品接入事后和实时 PPP 的设计方案。

Stage 4A 只做设计，不实现以下内容：

- `EPHOPT_B2b`；
- 事后 B2bBin 回放；
- `satpos_B2b()`；
- B2b 轨道或钟差改正；
- `corr_meas()` 中的 B2b 码偏差改正；
- 配置入口或 GUI 格式注册。

Stage 4A 唯一新增或修改的文件就是本文档。

## 2. 当前基线

检查时目标仓库的状态为：

```text
HEAD 9df5878 调试设备中断点可以执行后处理，raw → 主 nav 桥接已实现
工作树：新增本文档前为 clean
```

当前实现已经包含 Stage 3D 桥接：

```text
b2b_update_nav_from_raw(nav, raw)
  -> raw->nav.B2bssr[sat].update == 1 时复制该卫星产品
  -> 保留 nav->B2bssr[sat].update = 1
  -> 清除 raw->nav.B2bssr[sat].update
```

实时 `rtksvr` 已在以下条件下调用该 helper：

```text
ret == 20 && format == STRFMT_UNICORE
```

Stage 4 必须复用这个桥接函数，不能再为事后处理编写另一套复制、检查和
`update` 清理策略。

目标工程中不存在 `rtklib/app/postppp/` 和 `rtklib/app/rtppp/`。对应入口为：

- 事后处理程序：`rtklib/src/rnx2rtkp.c`；
- 事后处理引擎：`rtklib/src/postpos.c`；
- 实时服务路径：`rtklib/src/rtksvr.c`。

`rtklib/src/rnx2rtkp.c` 当前在命令行解析后仍存在硬编码输入、输出路径赋值，
会覆盖正常的 CLI 参数。Stage 4F 端到端验收前，必须删除这些硬编码赋值，
或将其放入显式的仅调试开关中。

## 3. 已检查的参考和目标代码

只读检查的参考工程：

- `RTKLIB-B2b/src/postpos.c`
  - `update_B2b_ssr()`
  - B2b 文件发现逻辑
  - 每个观测历元的调用位置
- `RTKLIB-B2b/src/rtksvr.c`
  - `update_B2bssr()`
  - `update_svr()`
- `RTKLIB-B2b/src/ephemeris.c`
  - `var_uraB2b()`
  - `satpos_B2b()`
  - `satpos()`
  - 修改过的 `seleph()`
- `RTKLIB-B2b/src/ppp.c`
  - `corr_meas()`
- `RTKLIB-B2b/include/rtklib.h`
  - `EPHOPT_B2b`
  - `B2bmask_t`
  - `B2bssr_t`
  - `nav_t.B2bssr[]`
  - `prcopt_t.B2b_format`
- `RTKLIB-B2b/include/B2b.h`
- `RTKLIB-B2b/src/B2b.c`
  - `checkout_B2beph()`
  - `checkout_B2bcbia()`
  - `checkout_B2bclk()`
  - `B2btod2time()`

检查的目标工程：

- `rtklib/src/b2b.c`
  - `b2b_slot2satno()`
  - `b2b_mask2satno()`
  - `b2b_tod2time()`
  - `b2b_update_nav_from_raw()`
- `rtklib/src/rtklib.h`
  - 当前的 `B2bmask_t`、`B2bssr_t`
  - `nav_t.B2bssr[MAXSAT+1]`
  - 当前星历和数据流格式常量
- `rtklib/src/rcv/unicore.c`
  - `decode_PPPPB2BINFO1/2/3/4()`
  - 当前字段比例、时间、IOD 和 `update` 行为
- `rtklib/src/rtksvr.c`
  - `update_b2b_ssr()`
  - `update_svr()`
  - `rtksvrthread()` 中的定位循环
- `rtklib/src/postpos.c`
  - `inputobs()`
  - `procpos()`
  - `readobsnav()`
  - `readpreceph()` / `freepreceph()`
- `rtklib/src/ephemeris.c`
  - `seleph()`
  - `ephpos()`
  - `satpos_ssr()`
  - `satpos()`
- `rtklib/src/ppp.c`
  - `corr_meas()`
  - `pppos()`
- `rtklib/src/options.c`
  - `EPHOPT`
  - `pos1-sateph`
- `rtklib/src/rnx2rtkp.c`
- `rtklib/rtklib.vcxproj`
- `rtklib/app/test_b2b_decoder/test_unicore_receiver.c`

## 4. 当前完整数据流

当前接收机解码和实时路径：

```text
Unicore B2bBin/实时流字节
  -> input_raw()/input_rawf(STRFMT_UNICORE)
  -> input_unicore()/input_unicoref()
  -> decode_PPPPB2BINFO1/2/3/4()
  -> raw->rcv_data 中接收机实例独立的 MASK
  -> raw->nav.B2bssr[sat]，update = 1
  -> rtksvr.c::update_b2b_ssr()
  -> b2b_update_nav_from_raw(&svr->nav, raw)
  -> svr->nav.B2bssr[sat]，update = 1
```

Stage 4 尚缺少的路径：

```text
事后处理观测历元
  -> 只回放截至当前历元已可用的 B2bBin 数据
  -> b2b_update_nav_from_raw(&navs, &b2b_raw)
  -> navs.B2bssr[sat]
  -> EPHOPT_B2b 分发
  -> B2b 轨道/钟差改正
  -> B2b 码偏差改正
  -> PPP 残差和滤波
```

所有 B2b 路径必须继续使用：

```text
B2bssr[sat]，sat = 1..MAXSAT
```

下标 0 不使用。不能把标准 RTKLIB 的 `nav->ssr[sat-1]` 规则套到 B2b 存储上。

## 5. 事后处理 B2bBin 回放

### 5.1 文件所有权和初始化

Stage 4B 应在 `postpos.c` 中增加一个事后 B2b 回放上下文，至少包含：

```text
raw_t raw
FILE *fp
char path[MAXSTRPATH]
int format
int eof
int pending
gtime_t pending_rx_time
```

对于当前已支持的接收机，扩展名为 `.B2bBin`、`.b2b` 或 `.B2b` 的输入文件
与 `STRFMT_UNICORE` 关联。以后存在多种格式时，必须通过显式配置指定格式，
不能靠猜测二进制内容来决定。

必须单独识别 B2b 输入文件，并让 `readobsnav()` 跳过它。否则
`readobsnav()` 会把每个输入路径都交给 `readrnxt()`，从而把 B2bBin
错误地当成 RINEX 文件读取。

初始化和清理流程：

```text
会话开始
  -> 查找一个 B2b raw 文件
  -> init_raw(&ctx.raw, STRFMT_UNICORE)
  -> fopen(path, "rb")

会话结束或出错
  -> fclose(ctx.fp)
  -> free_raw(&ctx.raw)
```

除非现有 `postpos.c` 的全局会话设计确实要求，否则不能让多个并发会话共享
同一个 B2b 回放状态。

### 5.2 按观测历元调度

对每个正向观测历元 `Tobs`，Stage 4B 应执行：

1. 如果之前已解码并暂存了一帧，只有当
   `pending_rx_time <= Tobs + DTTOL` 时才能发布。
2. 持续调用 `input_rawf()`，直到：
   - 到达 EOF 或发生错误；或者
   - 下一完整 B2b 帧的接收机时间晚于 `Tobs`。
3. 对所有在 `Tobs` 之前或等于 `Tobs` 可用的帧，只调用现有桥接：

```c
b2b_update_nav_from_raw(&navs, &ctx.raw);
```

4. 第一帧未来数据保留在 `ctx.raw` 中并标记为 pending。在该帧对后续观测
   历元变为可用之前，不能继续解码下一帧。
5. PPP 使用已经发布到 `navs` 中的最新有效产品。

调度门限使用 Unicore 帧头时间 `raw.time`，因为它代表改正产品实际可用的
接收时刻。产品 age 检查使用 `t0[0..2]`，不能使用 `raw.time`。

单帧 look-ahead 可以防止在当前观测历元之后才收到的改正数被提前用于当前
历元。不能直接复制参考工程的 `update_B2b_ssr()` 循环，因为参考实现没有
清晰分离“未来帧暂存”和“主 nav 发布”。

### 5.3 正向、反向和组合处理

Stage 4B 首先只支持正向处理。

反向处理不能直接消费只支持向前读取的 B2b 字节流，否则必须建立带时间索引
的改正产品时间线，或者反复 rewind 后重新回放。Stage 4F 应明确选择：

- 预解码全部 B2b 产品，建立不可变的按时间索引时间线，各观测历元按时间查询；
  或者
- 对 B2b 模式明确拒绝反向和组合处理并给出错误。

禁止在反向处理中静默复用正向处理已经到达 EOF 的状态。

## 6. 实时路径复用

实时路径已经采用所需的发布流程：

```text
decoderaw()
  -> update_svr(ret == 20, STRFMT_UNICORE)
  -> update_b2b_ssr()
  -> b2b_update_nav_from_raw()
```

Stage 4 必须保留此路径。事后处理也必须调用同一个公共桥接，不能重新实现
参考工程中重复的 IOD、UDI、复制和 `update` 清理逻辑。

产品可用性检查应放在共享 B2b 校验 helper 以及
`satpos_B2b()`/`corr_meas()` 中，保证以下两种路径行为一致：

- 事后处理中的 `navs.B2bssr[sat]`；
- 实时处理中的 `svr->nav.B2bssr[sat]`。

实时流格式注册目前仍不完整，因为 `STRFMT_UNICORE == 18`，但
`MAXRCVFMT == 12`。Stage 4F 必须一起审查格式编号、名称表、解析器和
UI/配置链路。不能只修改 `MAXRCVFMT`。

## 7. `EPHOPT_B2b` 和配置

Stage 4D 应定义：

```c
#define EPHOPT_B2b 5 /* 广播星历 + PPP-B2b SSR APC */
```

该值延续当前编号顺序：

```text
0 brdc
1 precise
2 brdc+sbas
3 brdc+ssrapc
4 brdc+ssrcom
5 brdc+b2b-apc
```

Stage 4F 应在 `options.c` 中追加：

```text
pos1-sateph = brdc+b2b-apc
```

配置职责必须分开：

- `sateph` 决定 PPP 是否使用 B2b 改正；
- 事后输入文件列表或专用 B2b 文件选项用于指定 B2bBin；
- 实时数据流配置用于指定 `STRFMT_UNICORE`；
- 设置输入格式不能隐式启用 B2b PPP；
- 启用 `EPHOPT_B2b` 但没有 B2b 输入时，必须明确失败或不生成 B2b 解，
  不能静默回退到精密星历。

参考工程的 `prcopt_t.B2b_format` 可以作为兼容思路，但只有当同一个字段确实
能够正确服务于事后文件回放和实时流配置时，目标工程才应增加它。实时格式的
现有权威来源已经是 `rtksvr.format[]`。

## 8. 使用 `iodn` 选择广播星历

### 8.1 专用选择器

Stage 4D 应在 `ephemeris.c` 中增加专用于 B2b 的广播星历选择器。不能为了
B2b 修改通用 `seleph()` 的行为，否则会影响所有原有 RTKLIB 路径。

选择器必须：

1. 选择指定 `sat`；
2. 要求广播星历健康；
3. 满足 RTKLIB 原有 toe age 限制；
4. 根据对应星座和广播消息类型，用正确字段匹配 B2b `iodn`；
5. 返回一个确定的星历对象，并让位置和钟差计算使用同一个对象。

需要用真实导航数据确认的初始映射：

| 系统 | B2b IODN 比较规则 |
|---|---|
| BDS-3 B-CNAV1 | 与保留下来的 CNAV1 IODC/IODN 比较，目标字段预计为 `eph.iodc` |
| GPS LNAV | 与等价 IODE 值比较，目标字段预计为 `eph.iode` |
| Galileo | 在确认 B2b IODN 对 I/NAV 或 F/NAV 的规则前，不启用 |
| GLONASS | 在确认 B2b IODN 与 `geph.iode` 的规则前，不启用 |

参考工程通过全局标志让通用 `seleph()` 在 `iode` 和 `iodc` 之间切换。这种
方法影响范围过大，可能导致非 B2b 路径选错星历，目标工程不能照搬。

### 8.2 找不到星历

如果不存在与 IODN 精确匹配的广播星历：

- `satpos_B2b()` 返回 0；
- `*svh` 设为不可用；
- 禁止回退到 IOD 最接近的星历；
- 限频 trace 记录卫星、IODN 和改正历元。

使用 IOD 不匹配的最近广播星历，会把改正数施加到错误的轨道和钟差上，
这是不可接受的。

## 9. 轨道和钟差规则

### 9.1 存储单位

当前 Unicore decoder 的存储规则：

| 字段 | 比例和单位 |
|---|---|
| `deph[0]` 径向 | 原始值乘 0.0016 m |
| `deph[1]`切向 | 原始值乘 0.0064 m |
| `deph[2]` 法向 | 原始值乘 0.0064 m |
| `ddeph[]` | m/s；INFO2 当前未解码变化率，因此为 0 |
| `dclk[0]` | 原始值乘 0.0016 m |
| `dclk[1]` | m/s；当前为 0 |
| `dclk[2]` | m/s²；当前为 0 |

### 9.2 RAC 到 ECEF

使用广播星历位置 `r` 和速度 `v`：

```text
ea = normalize(v)
ec = normalize(r x v)
er = ea x ec

delta_ecef =
    er * deph_radial +
    ea * deph_along +
    ec * deph_cross
```

PPP-B2b 改正后的卫星位置为：

```text
r_corrected = r_broadcast - delta_ecef
```

该符号遵循 PPP-B2b 用户算法，也与参考实现中 RAC 组合前的负号一致。

RAC 基向量必须由精确匹配 IODN 的广播星历计算。如果速度向量或叉积范数退化，
该改正不可用。

### 9.3 钟差符号

`dclk[]` 是以米为单位的距离改正。改正后的卫星钟差单位为秒：

```text
dts_corrected = dts_broadcast - dclk / CLIGHT
```

PPP-B2b 必须使用负号。目标工程标准 `satpos_ssr()` 当前对标准 SSR 钟差项
使用加号，不能机械复制到 `satpos_B2b()`。

### 9.4 APC 约定

`EPHOPT_B2b` 设计为 B2b APC 产品。除非真实测试数据证明解码产品以卫星质心
COM 为参考，否则 Stage 4D 不能在 `satpos_B2b()` 内再次执行 COM 到 APC
的卫星天线偏移改正。

参考工程虽然将选项命名为 APC，但仍调用自定义天线偏移路径。该行为需要独立
验证，默认不能移植。

## 10. 产品一致性和有效性

### 10.1 轨道和钟差必需条件

单星轨道/钟差改正只有在以下条件全部满足时才可用：

```text
t0[0] 存在
t0[2] 存在
iodssr[0] == iodssr[2]
iodcorr[0] == iodcorr[1]
clock IODP 已经与接收机实例当前 MASK 匹配并被 decoder 接受
存在精确匹配 iodn 的广播星历
轨道和钟差 age 有效
URAI 可用
所有改正值有限且未超过幅度限制
```

码偏差还要求：

```text
t0[1] 存在
iodssr[1] == iodssr[0] == iodssr[2]
码偏差 age 有效
对应 CODE_* 项具有显式有效标志
```

decoder 已经在 INFO2/INFO3 的 IOD SSR 与当前 MASK 不同时拒绝消息，也会在
INFO4 的 IOD SSR 或 IODP 不匹配时拒绝消息。但主 nav 中的产品来自不同消息，
可能属于不同代，因此 Stage 4 使用产品时仍必须再次进行跨产品一致性检查。

### 10.2 标称有效期

使用 PPP-B2b ICD 的标称有效期：

| 产品 | 标称有效期 |
|---|---:|
| MASK | 48 s |
| 轨道 | 96 s |
| 码偏差 | 86400 s |
| 钟差 | 12 s |

在信号发射时刻 `t`：

```text
age = timediff(t, product_t0)
```

产品有效条件：

```text
-DTTOL <= age <= nominal_validity + DTTOL
```

很小的负容差用于覆盖发射时刻迭代和时间戳舍入。产品时间明显晚于请求的发射
时刻时不可使用。

不能复制参考实现额外增加 30 秒宽限的做法。该宽限超过 12 秒钟差标称有效期，
会掩盖回放同步或实时延迟错误。

### 10.3 UDI

当前 Unicore INFO1-4 decoder 没有收到标准 SSR UDI 索引。由相邻产品历元推导
出来的 `udi[]` 只能表示诊断用播发周期，不能作为权威有效期。

规则：

- 使用 ICD 固定标称有效期判断产品是否可用；
- 推导 UDI 必须为正且在合理上限内才能保存；
- UDI 不能延长产品有效期；
- 当前 `ddeph[]`、`dclk[1]`、`dclk[2]` 都是 0，不需要用 UDI 外推；
- 除非后续 ICD 验证证明当前解码历元表示区间边界而不是改正参考历元，否则
  不能从 age 中减去 `udi/2`。

### 10.4 改正幅度保护

Stage 4C 应保留与 RTKLIB SSR 等价的保护：

```text
norm(deph) <= MAXECORSSR
abs(dclk) <= MAXCCORSSR
所有数值必须为有限值
```

检查失败时，该卫星的 B2b 改正不可用，不能只修改广播位置或钟差中的一部分。

## 11. URAI 转卫星位置方差

参考工程的 `var_uraB2b()` 恒返回 0，只是占位实现，不能移植。

对于 6 bit URAI：

```text
class = (urai >> 3) & 7
value = urai & 7
```

转换规则：

```text
urai == 0:
    sigma = 0.0005 m

1 <= urai <= 62:
    sigma = (3^class * (1 + value/4) - 1) * 1E-3 m

urai == 63 或超出 0..63:
    改正不可用

variance = sigma^2
```

Stage 4C 应实现一个返回成功/失败状态和方差的 helper。单元测试必须覆盖
URAI 0、1、各 class 代表性边界、62 和 63。

## 12. 在 `corr_meas()` 中使用码偏差

### 12.1 应用层次

B2b 码偏差应在 `corr_meas()` 中使用：原始伪距通过频率和 SNR 检查并完成
天线改正后、形成无电离层或其他线性组合之前应用。

必须使用实际观测码：

```text
code = obs->code[i]
P[i] -= nav->B2bssr[obs->sat].cbias[code]
```

不能只根据频点下标选择硬编码的标称码。decoder 按 RTKLIB `CODE_*` 保存
码偏差，观测码才是准确的查询键。

在 `EPHOPT_B2b` 下，B2b 码偏差应替代目标工程原有的该信号 DCB 改正。
不能对同一伪距重复应用两种码偏差。

### 12.2 缺失值处理

当前结构无法区分：

- 有效码偏差恰好为 0.0 m；
- B2b 消息没有播发该码。

Stage 4E 必须增加显式的逐码有效性，例如：

```c
uint8_t cbias_valid[MAXCODE+1];
```

decoder 规则：

- 某卫星开始新的码偏差历元或 IOD 代时，清空该卫星的 code-valid 表；
- 只有成功解码的条目才能设置 `cbias_valid[code] = 1`；
- 下标 0 不使用；
- 不能使用 NaN 或数值 0 充当可用性标志。

PPP 规则：

- 要求码偏差产品有效且未过期；
- 要求 `cbias_valid[obs->code[i]]`；
- 缺失时将该改正伪距设为不可用，并输出 trace；
- 不能静默按 0.0 m 处理；
- 如果选定双频组合中的任何一个码偏差缺失，就不能为该卫星形成 B2b 改正后的
  码组合。

增加有效性元数据后，Stage 1/3 decoder 的输出数量必须保持不变。

## 13. `update` 的所有者和清零阶段

存在两个不同层次的事件标志。

### 13.1 raw 侧标志

所有者和消费者：

```text
decoder 设置 raw->nav.B2bssr[sat].update = 1
b2b_update_nav_from_raw() 消费并清零
```

桥接之前，任何其他阶段都不能清除 raw 侧 `update`。

### 13.2 主 nav 标志

含义：

```text
nav->B2bssr[sat].update == 1
```

表示自上一个定位历元以来发布了新的产品集合。它不是产品有效性标志。

清零规则：

- `satpos_B2b()` 和 `corr_meas()` 不能修改 `nav_t`；
- 一个观测历元完成 `rtkpos()` 尝试后，由历元协调层调用一个小型共享 helper
  清除主 nav 的 update 事件；
- 事后处理在每个历元结束后调用该 helper；
- 实时处理在本批 rover 观测处理结束后调用同一 helper；
- 清除 `update` 不能清除 `t0`、IOD、轨道、钟差、URAI 或码偏差。

在 Stage 4F 增加历元级清零调用前，主 nav 的 `update` 可以一直保留为 1。
任何有效性判断都不能依赖该标志。

## 14. 当前样例数据是否足够

目标工程现有样例：

```text
rtklib/data/Unicore_2024360.B2bBin
大小：15,905,080 字节
解码时间范围：约 2024-12-25 00:00:12
               至 2024-12-26 00:00:29 GPST
```

已验证解码数量：

```text
MASK:           3602
ORBIT_URAI:     13420
DIFF_CODE_BIAS: 13294
CLOCK:          86410
CRC_ERROR:      0
FRAME_ERROR:    0
UNKNOWN:        0
CONTEXT_ISOLATION: 1
```

该样例足以测试 decoder、桥接、回放调度和字段规则，但不足以进行 PPP 端到端
验证。

还需要同时间段的以下数据：

1. 覆盖 2024-12-25 至 2024-12-26 的 rover RINEX 观测文件。
2. 覆盖所有被改正 GPS 和 BDS-3 卫星的广播导航数据。
3. 对 BDS-3，导航数据必须保留匹配 B2b `iodn` 所需的 B-CNAV1
   IODC/IODN；只有传统 D1/D2 的导航文件可能不够。
4. 对 GPS，需要包含 B2b 轨道消息所引用 IODE/IODC 代的广播星历。
5. 测站坐标，或者可靠的 RINEX approximate position。
6. 如果选定 PPP 模型需要，提供接收机和卫星天线校准数据。
7. 正式验收建议提供 ERP、海潮负荷等常规 PPP 辅助文件。
8. 用于定量比较的参考解或已知坐标。

检查到的参考工程示例目录包含配置、天线文件和压缩的示例输出，但工作区内
没有找到与当前 B2bBin 同期的原始 RINEX 观测和导航数据。

## 15. Stage 4 实施拆分

### Stage 4B：事后 B2bBin 到主 nav

目的：

- 在事后处理中识别并打开一个 Unicore B2bBin；
- 使用单帧 look-ahead 按观测历元回放；
- 只通过 `b2b_update_nav_from_raw()` 发布；
- 首先只支持正向处理。

预计修改文件：

- `rtklib/src/postpos.c`
- `rtklib/src/rnx2rtkp.c`，仅用于停止硬编码输入覆盖并正常传入 B2b 文件
- `rtklib/app/test_b2b_decoder/` 下的聚焦回放/桥接测试

禁止修改：

- `ephemeris.c`
- `ppp.c`
- `options.c`

验证：

- Debug x64 构建；
- 使用合成观测历元时间回放现有 B2bBin；
- 证明接收时间晚于 `Tobs` 的产品不会进入主 nav；
- 最终主 nav 产品与 Stage 3D 全文件回放结果一致；
- 验证 raw `update` 只由桥接清除。

回退：

- 仅 revert Stage 4B 的独立提交，或对上述文件应用审核过的逆向补丁；
- 禁止 reset、clean、大范围 checkout 或删除文件。

### Stage 4C：共享轨道/钟差和有效性 helper

目的：

- 增加 RAC 到 ECEF helper；
- 增加 URAI 到方差 helper；
- 增加 age、IOD 和产品就绪状态 helper；
- 增加主 nav `update` 事件清除 helper；
- helper 暂不接入 `satpos()` 分发。

预计修改文件：

- `rtklib/src/b2b.c`
- `rtklib/src/rtklib.h`
- `rtklib/app/test_b2b_decoder/` 下的聚焦 helper 测试

验证：

- 用确定性单位向量验证 RAC 符号和基向量；
- 验证以米为单位的钟差按正确符号转成秒；
- URAI 边界测试；
- 96/12/86400 秒标称有效期边界测试；
- IOD SSR 和 IOD Corr 不匹配拒绝测试；
- `[sat]` 下标 1 和 `MAXSAT` 边界测试。

回退：

- 仅 revert Stage 4C 独立提交，或逆向移除该阶段隔离的 helper。

### Stage 4D：`satpos_B2b()` 和 `satpos()` 分发

目的：

- 定义 `EPHOPT_B2b`；
- 增加 B2b 专用且识别 IODN 的广播星历选择器；
- 实现 `satpos_B2b()`；
- 只增加 `satpos()` 分发分支；
- 暂不暴露配置入口。

预计修改文件：

- `rtklib/src/rtklib.h`
- `rtklib/src/ephemeris.c`
- 聚焦卫星位置测试

验证：

- 精确 IODN 匹配能够选择预期广播星历；
- IODN 不匹配时返回不可用；
- 零改正时复现选定广播星历结果；
- 合成径向、切向、法向改正产生预期 ECEF 符号；
- 合成正 `dclk` 使 `dts` 减少 `dclk/CLIGHT`；
- 过期轨道、过期钟差和 URAI 63 被拒绝；
- 原有 BRDC、PRECISE、SBAS 和 SSR 测试行为不变。

回退：

- 仅 revert Stage 4D 独立提交。该分支是增量增加，原有星历模式不受影响。

### Stage 4E：B2b 码偏差

目的：

- 增加显式 `cbias_valid[]`；
- 在 Unicore decoder 中维护该标志；
- 在 `corr_meas()` 形成组合前应用 B2b 码偏差；
- 拒绝缺失或过期的信号专用偏差。

预计修改文件：

- `rtklib/src/rtklib.h`
- `rtklib/src/rcv/unicore.c`
- `rtklib/src/ppp.c`
- decoder 和观测改正测试

验证：

- 有效偏差使 `P[i]` 精确改变 `-cbias`；
- 真实零偏差仍然可用；
- 缺失偏差不会被当成零；
- 错误观测码、过期偏差或 IOD SSR 不匹配被拒绝；
- 非 B2b PPP 保持原有 DCB 行为；
- Stage 1/3 的 section 数量保持既有基线。

回退：

- 仅 revert Stage 4E 独立提交，不删除标准 SSR 或原有 DCB 字段。

### Stage 4F：配置和端到端验收

目的：

- 在文本配置中暴露 `EPHOPT_B2b`；
- 完成事后文件和格式选择；
- 将实时接收机格式注册作为一个完整变更统一审查；
- 事后和实时在定位结束后清除主 nav `update` 事件；
- 使用匹配数据完成端到端 PPP 验收。

预计修改文件：

- `rtklib/src/options.c`
- `rtklib/src/rnx2rtkp.c`
- `rtklib/src/postpos.c`
- `rtklib/src/rtksvr.c`
- 只有完整格式表审查确有需要时才修改 `rtklib/src/rtkcmn.c` 和
  `rtklib/src/rtklib.h`
- 经 Stage 4F 明确授权的工程或应用配置文件

验证：

- `pos1-sateph=brdc+b2b-apc` 可以正确保存并重新加载；
- 禁用 B2b 时复现当前基线；
- 启用 B2b 时报告改正可用或拒绝的具体原因；
- 事后和实时使用同一套主 nav 产品有效性判断；
- 使用同期观测和导航数据确认位置、钟差和码偏差已实际应用；
- 与已知坐标或参考解比较，并在可能时与参考工程 PPP-B2b 结果比较。

回退：

- 将 `pos1-sateph` 切回已有星历模式即可禁用 B2b；
- 配置集成失败时只 revert Stage 4F 独立提交；
- 除非 Stage 4B 至 4E 自身测试失败，否则保留其已通过单测的 helper。

## 16. 通用构建和回归命令

每个实施阶段完成后的主工程构建：

```powershell
MSBuild.exe .\rtklib\rtklib.vcxproj /t:Build /p:Configuration=Debug /p:Platform=x64 /m /v:minimal
```

decoder 基线：

```powershell
cd rtklib\app\test_b2b_decoder
.\test_b2b_decoder.exe ..\..\data\Unicore_2024360.B2bBin out.txt
rg -c "^> MASK" out.txt
rg -c "^> ORBIT_URAI" out.txt
rg -c "^> DIFF_CODE_BIAS" out.txt
rg -c "^> CLOCK" out.txt
```

每个阶段开始前：

```powershell
git -C rtklib status --short --untracked-files=all
```

每个阶段都应检查自己的 diff。用户允许提交时，应保持每个阶段为隔离提交。
回退应使用 `git revert` 回退隔离提交，或应用审核过的逆向补丁；禁止使用
`git reset --hard`、`git clean` 或批量删除文件。

## 17. 剩余风险

- 尚未证明目标 RINEX reader 能完整保留 BDS-3 B-CNAV1 IODN。
- Galileo 和 GLONASS 的 B2b IODN 映射尚未确认，初期应保持禁用。
- 当前样例没有同期观测和导航文件，无法完成 PPP 验收。
- 目标 `rnx2rtkp.c` 中现有硬编码路径会妨碍可信的命令行端到端测试。
- 反向和组合事后处理需要明确的改正产品时间线设计。
- 当前码偏差存储缺少逐码有效性标志。
- 主 nav 的 `update` 当前不会在定位历元结束后清零。
- 除直接 raw 分发外，接收机格式注册仍不完整。

## 18. 外部技术依据

- 北斗卫星导航系统空间信号接口控制文件，精密单点定位服务信号 PPP-B2b
  英文版：
  `https://en.beidou.gov.cn/SYSTEMS/ICD/202008/P020231201538195573144.pdf`
- 中文官方版本：
  `https://www.beidou.gov.cn/xt/gfxz/202008/P020200803362062482940.pdf`

本文采用的标称产品有效期、URAI 解释、RAC 轨道改正符号和钟差改正符号均以
该 ICD 为技术依据。

## 19. Stage 4A 退出声明

Stage 4A 只定义接入和验证方案。本阶段没有把任何 PPP-B2b 改正接入
`satpos()`、`satpos_B2b()`、`corr_meas()`、配置、事后 PPP 或实时 PPP。
