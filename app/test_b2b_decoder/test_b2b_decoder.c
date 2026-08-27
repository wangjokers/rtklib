#include <stdio.h>
#include <string.h>

#include "b2b_decoder.h"

/*
 * Stage 1 CLI wrapper.
 *
 * 这个文件只负责命令行入口，不参与任何 B2b 业务字段解析：
 *
 *   -U input [out] -> Unicore/UM980 B2bBin（默认格式）
 *   -S input [out] -> SinoGNSS/司南 AA 44 12 B2b raw
 *
 * 真正的帧同步、CRC、消息号分发和 MASK/ORBIT/CODE_BIAS/CLOCK 解码
 * 都在 b2b_decoder.c 的 b2b_decode_unicore_file() 里完成。这样 main()
 * 保持很薄，后续如果要把同一个解码器放到别的独立测试程序中，也只需要
 * 复用 b2b_decoder.h 暴露的一个函数。
 */
static void print_usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [-U|-S] <B2b_raw_file> [out.txt]\n", prog);
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
    const char *input_path;
    const char *output_path = NULL;
    int format_sino = 0;
    int argi = 1;
    int ret;

    if (argc > 1 && (strcmp(argv[1], "-U") == 0 || strcmp(argv[1], "-S") == 0)) {
        format_sino = strcmp(argv[1], "-S") == 0;
        argi++;
    }
    if (argc - argi != 1 && argc - argi != 2) {
        print_usage(argv[0]);
        return 1;
    }
    input_path = argv[argi];
    if (argc - argi == 2) output_path = argv[argi + 1];

    if (output_path) {
        /* 可选输出路径例如 out.txt；输出格式由解码器保持稳定。 */
        out = fopen(output_path, "w");
        if (!out) {
            fprintf(stderr, "test_b2b_decoder: failed to open output file: %s\n",
                    output_path);
            return 1;
        }
    }

    /* 无格式选项时保持原有 Unicore 行为；-S 显式选择司南格式。 */
    ret = format_sino ? b2b_decode_sino_file(input_path, out)
                      : b2b_decode_unicore_file(input_path, out);

    if (out != stdout) {
        fclose(out);
    }
    return ret == 0 ? 0 : 1;
}
