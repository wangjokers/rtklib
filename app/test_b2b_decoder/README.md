# test_b2b_decoder 阶段 1 说明

## 阶段 1 目标

`test_b2b_decoder` 是 rtklib 主工程里的独立验证程序，只读取
Unicore/UM980 输出的 B2bBin raw 文件，解析并打印 B2b 消息：

- `MASK`
- `ORBIT_URAI`
- `DIFF_CODE_BIAS`
- `CLOCK`

本阶段目标是验证 B2b raw 解码结果可以和 RTKLIB-B2b 参考工程
`postdecoder -U` 输出做文本对比。它不接入 PPP 主流程。

## 当前文件结构

```text
rtklib/app/test_b2b_decoder/
  test_b2b_decoder.c  CLI 入口，处理参数和输出文件
  b2b_decoder.h       阶段 1 唯一对外接口声明
  b2b_decoder.c       Unicore B2bBin 同步、CRC、消息分发和四类消息解码
  Makefile            最小 gcc/MinGW 编译方式
  README.md           本说明文档
```

`b2b_decoder.c` 内部函数默认都保持 `static`，只暴露：

```c
int b2b_decode_unicore_file(const char *path, FILE *out);
```

这样做是为了保持阶段 1 独立，不把临时验证函数扩散成 rtklib 主工程 API。

## 编译命令

在 `rtklib/app/test_b2b_decoder/` 目录下：

```sh
make
```

如果当前环境没有 `make`，可以直接用 gcc/MinGW：

```sh
gcc -std=c99 -O2 -Wall -Wextra -DENAGLO -DENAGAL -DENAQZS -DENACMP -DWIN32 -I../../src -o test_b2b_decoder.exe test_b2b_decoder.c b2b_decoder.c -lm -lwinmm
```

默认构建使用 `b2b_decoder.c` 内部的最小 RTKLIB 兼容时间、CRC、卫星号函数。
如果后续确认 `../../src/rtkcmn.c` 在本工程中可直接干净编译，也可以尝试：

```sh
make USE_RTKCMN=1
```

## 运行命令

输出到 stdout：

```sh
./test_b2b_decoder ../../data/Unicore_2024360.B2bBin
```

输出到文本文件：

```sh
./test_b2b_decoder ../../data/Unicore_2024360.B2bBin out.txt
```

Windows PowerShell 下通常是：

```powershell
.\test_b2b_decoder.exe ..\..\data\Unicore_2024360.B2bBin out.txt
```

输入文件由命令行第一个参数决定，代码内部没有硬编码 B2bBin 路径。

## 解码流程

```text
open file
  -> sync AA 44 B5
  -> read header
  -> get payload length
  -> wait header + payload + CRC
  -> CRC check
  -> message id dispatch
  -> decode 2302/2304/2306/2308
  -> print section
  -> summary
```

关键点：

- `AA 44 B5` 是 Unicore 二进制帧同步头。
- Unicore 帧头固定 24 字节。
- header bytes `6..7` 是 payload length，小端 `uint16`。
- 完整帧长度是 `24 + payload length + 4`，最后 4 字节是 CRC。
- `rtk_crc32()` 校验 `sync + header + payload`，不包含 CRC 本身。
- message id `2302/2304/2306/2308` 分别对应
  `MASK/ORBIT_URAI/DIFF_CODE_BIAS/CLOCK`。

## 如何对比 RTKLIB-B2b postdecoder -U

先用阶段 1 程序生成输出：

```powershell
.\test_b2b_decoder.exe ..\..\data\Unicore_2024360.B2bBin out.txt
```

再统计四类 section 数量：

```powershell
rg -c "^> MASK" out.txt
rg -c "^> ORBIT_URAI" out.txt
rg -c "^> DIFF_CODE_BIAS" out.txt
rg -c "^> CLOCK" out.txt
```

这些命令不是分别重新解码四次，而是在同一个 `out.txt` 里统计以对应标题开头的
section 行数。`^` 表示行首，`-c` 表示只输出匹配行数量。

参考工程侧使用 RTKLIB-B2b 的 `postdecoder -U` 对同一个 B2bBin 生成参考输出，
然后用同样的 `rg -c` 方式统计参考 txt 中四类 section 数量。阶段 1 首先看
summary 和四类 section 数量是否一致，再抽样对比具体 `>` 标题行和卫星改正行。

## 当前阶段还没有做什么

- 没有修改 `postppp.c`、`rtppp.c`、`ppp.c`、`ephemeris.c`。
- 没有修改 `rtklib/src/` 现有文件。
- 没有把解码结果写入 `raw_t`、`nav_t` 或 `nav->B2bssr`。
- 没有让 B2b 改正参与 `ppp.c` 或 `ephemeris.c` 的轨道、钟差、码偏差改正。
- 没有支持除 Unicore/UM980 B2bBin 之外的其他接收机 raw 格式。

阶段 1 的边界是独立解码验证。接入 PPP 主流程应放在后续阶段，在解码结果和
参考工程输出稳定可比后再做。
