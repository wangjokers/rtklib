#ifndef TEST_B2B_DECODER_H
#define TEST_B2B_DECODER_H

#include <stdio.h>

/*
 * Stage 1 唯一对外接口。
 *
 * path: Unicore/UM980 输出的 B2bBin 原始二进制文件。
 * out : 已打开的文本输出流，可以是 stdout，也可以是 main() 打开的 out.txt。
 *
 * 设计约束：
 * - 只暴露一个文件级解码入口，其他 Unicore 同步、CRC、消息解码函数都保留
 *   static，避免阶段 1 的临时代码扩散成 rtklib 主工程 API。
 * - 不依赖 rtklib.h/raw_t/nav_t。阶段 1 的目标是“独立验证 B2b raw 解码是否
 *   和参考工程 postdecoder -U 可对比”，还不是接入 PPP 解算。
 * - 输出格式由 b2b_decoder.c 中的 print_b2b_info1/2/3/4 和 print_summary()
 *   固定，便于和 RTKLIB-B2b 参考输出做文本对比。
 */
int b2b_decode_unicore_file(const char *path, FILE *out);

#endif
