#include <stdio.h>

#include "b2b_decoder.h"

/*
 * Stage 1 CLI wrapper.
 *
 * 这个文件只负责命令行入口，不参与任何 B2b 业务字段解析：
 *
 *   argv[1] -> Unicore/UM980 B2bBin 原始文件
 *   argv[2] -> 可选输出 txt；没有传 argv[2] 时直接写 stdout
 *
 * 真正的帧同步、CRC、消息号分发和 MASK/ORBIT/CODE_BIAS/CLOCK 解码
 * 都在 b2b_decoder.c 的 b2b_decode_unicore_file() 里完成。这样 main()
 * 保持很薄，后续如果要把同一个解码器放到别的独立测试程序中，也只需要
 * 复用 b2b_decoder.h 暴露的一个函数。
 */
static void print_usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <Unicore_B2bBin> [out.txt]\n", prog);
}

/*
 * main() 执行流程学习笔记：
 *
 * 1. 检查参数数量。阶段 1 只支持一个输入文件，输出文件是可选的。
 * 2. 如果用户传了 out.txt，就以文本写入方式打开；否则使用 stdout。
 * 3. 调用 b2b_decode_unicore_file()，把“读哪个 B2bBin 文件”和“写到哪里”
 *    交给解码模块。
 * 4. 关闭可选输出文件，并把内部返回值转换成进程退出码。
 *
 * NOTE: main() 不知道 PPP、raw_t、nav_t，也不接触 rtklib/src 的主流程。
 * 它的作用只是让阶段 1 可以单独编译、单独运行、单独对比 postdecoder -U。
 */
int main(int argc, char **argv)
{
    FILE *out = stdout;
    int ret;

    if (argc != 2 && argc != 3) {
        print_usage(argv[0]);
        return 1;
    }
    if (argc == 3) {
        /* argv[2] 是可选输出路径，例如 out.txt；输出格式由解码器保持稳定。 */
        out = fopen(argv[2], "w");
        if (!out) {
            fprintf(stderr, "test_b2b_decoder: failed to open output file: %s\n", argv[2]);
            return 1;
        }
    }

    /* argv[1] 决定本次解码哪个 B2bBin 文件。 */
    ret = b2b_decode_unicore_file(argv[1], out);

    if (out != stdout) {
        fclose(out);
    }
    return ret == 0 ? 0 : 1;
}
