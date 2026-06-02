#include "b2b_decoder.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * Stage 1 B2bBin decoder learning notes.
 *
 * 这个文件是 app-local 的独立解码器，目标是把 Unicore/UM980 的 B2bBin
 * 原始帧解成可读 txt，方便和 RTKLIB-B2b 参考工程的 postdecoder -U 输出对比。
 *
 * 为什么阶段 1 不直接改 rtklib.h/raw_t/nav_t？
 * - raw_t/nav_t 是 RTKLIB 主流程的数据合同，一旦改动就会影响 postppp/rtppp/PPP
 *   的编译和行为；阶段 1 只验证“B2b raw 能不能被正确解出来”。
 * - 这里使用轻量结构体保存 MASK 和四类 SSR 结果，字段只覆盖打印和对比所需内容。
 * - 后续阶段如果要接入 PPP，再把已验证字段映射到 nav->B2bssr 等主流程结构。
 *
 * 为什么大部分函数保留 static？
 * - 这些函数都是阶段 1 内部实现细节，比如 Unicore 同步、payload 偏移解析、
 *   临时打印格式。保留 static 可以限制符号可见性，避免变成公共 API。
 * - 当前只暴露 b2b_decode_unicore_file()，后续如果确实需要复用，再有针对性地
 *   拆接口，而不是提前暴露一堆还未稳定的函数。
 *
 * WARNING: 本文件输出格式要贴近 postdecoder -U，注释增强不能改变 fprintf()
 *          字符串、summary 计数口径或解码分支行为。
 */

/*
 * 轻量版 gtime_t。
 *
 * 默认构建不直接链接 rtklib/src/rtkcmn.c，是为了保证阶段 1 完全独立。
 * 如果 Makefile 使用 USE_RTKCMN=1，则下面的 extern 会复用 RTKLIB 原函数；
 * 否则本文件后半部分提供最小兼容实现。
 */
typedef struct {
    time_t time;
    double sec;
} gtime_t;

#ifdef B2B_USE_RTKCMN
extern uint32_t rtk_crc32(const uint8_t *buff, int len);
extern gtime_t gpst2time(int week, double sec);
extern gtime_t gpst2bdt(gtime_t t);
extern void time2epoch(gtime_t t, double *ep);
extern double time2doy(gtime_t t);
extern double timediff(gtime_t t1, gtime_t t2);
extern int satno(int sys, int prn);
extern int satsys(int sat, int *prn);
extern void satno2id(int sat, char *id);
#endif

#define SYS_NONE 0x00
#define SYS_GPS  0x01
#define SYS_SBS  0x02
#define SYS_GLO  0x04
#define SYS_GAL  0x08
#define SYS_QZS  0x10
#define SYS_CMP  0x20
#define SYS_IRN  0x40
#define SYS_LEO  0x80

#define CODE_NONE 0
#define CODE_L1C  1
#define CODE_L1P  2
#define CODE_L1L  8
#define CODE_L1B 11
#define CODE_L1X 12
#define CODE_L2C 14
#define CODE_L2L 17
#define CODE_L2X 18
#define CODE_L5I 24
#define CODE_L5Q 25
#define CODE_L5X 26
#define CODE_L7I 27
#define CODE_L7Q 28
#define CODE_L6C 32
#define CODE_L2I 40
#define CODE_L6I 42
#define CODE_L1D 56
#define CODE_L5D 57
#define CODE_L5P 58
#define MAXCODE  68

/* Unicore OEM/UM980 二进制帧同步头。B2bBin 是一串字节流，需要先找 AA 44 B5。 */
#define UNICORE_SYNC1 0xAA
#define UNICORE_SYNC2 0x44
#define UNICORE_SYNC3 0xB5

/*
 * Unicore 二进制帧头固定 24 字节：
 *   bytes 0..2  : sync AA 44 B5
 *   byte  3     : header length，一般为 24
 *   bytes 4..5  : message id，例如 2302/2304/2306/2308
 *   bytes 6..7  : payload length，小端 uint16
 *   bytes 10..11: GPS week
 *   bytes 12..15: GPS milliseconds of week
 * 帧总长 = header(24) + payload length + CRC(4)。
 */
#define UNICORE_HEADER_LEN 24
#define UNICORE_MAX_LEN 16384

/*
 * Unicore PPPPB2BINFO 消息 ID：
 * 2302 -> MASK，播发本批 B2b 改正覆盖哪些卫星以及 IOD 信息。
 * 2304 -> ORBIT_URAI，轨道改正和 URAI 精度指标。
 * 2306 -> DIFF_CODE_BIAS，差分码偏差 DCB。
 * 2308 -> CLOCK，钟差改正。
 */
#define PPPPB2BINFO1 2302
#define PPPPB2BINFO2 2304
#define PPPPB2BINFO3 2306
#define PPPPB2BINFO4 2308

#define B2B_BDS_MINSAT 1
#define B2B_BDS_MAXSAT 63
#define B2B_GPS_MINSAT 64
#define B2B_GPS_MAXSAT 100
#define B2B_GAL_MINSAT 101
#define B2B_GAL_MAXSAT 137
#define B2B_GLO_MINSAT 138
#define B2B_GLO_MAXSAT 174
#define B2B_MAXSAT 174
#define B2B_CODE_BIAS_MODE_NUM 15
#define B2B_DECODER_MAXSAT 256

/* MJD+SOD 是 b2btod2time() 内部用于跨日修正的临时表示。 */
typedef struct {
    int mjd;
    double sod;
} mjd_gtime_t;

/*
 * MASK 消息缓存。
 *
 * raw_time : Unicore 帧头里的接收/记录时间，来自 GPS week + tow。
 * recv_time: 当前 MASK 帧的接收时间快照。
 * ref_time : B2b payload 里的参考时刻，payload 只有 BDT day-of-second，
 *            需要结合 raw_time 推回完整日期。
 * iod_ssr  : SSR 数据期号，用于让 ORBIT/CODE/CLOCK 和当前 MASK 对齐。
 * iodp     : MASK 期号，CLOCK 需要同时匹配 iod_ssr 和 iodp。
 * satno[]  : 把 B2b slot/mask 映射成 RTKLIB 风格 sat number 后的列表。
 */
typedef struct {
    gtime_t recv_time;
    gtime_t ref_time;
    int mask_bd[63];
    int mask_gps[37];
    int mask_gal[37];
    int mask_glo[37];
    int iod_ssr;
    int iodp;
    int satnum;
    int satno[B2B_MAXSAT];
} b2b_mask_t;

/*
 * 单颗卫星的轻量 SSR 缓存。
 *
 * sow       : payload 里的 B2b 秒内时刻，原始单位是秒。
 * verify_sow: ref_time 转回时分秒后的秒内值，用于调试跨日转换是否合理。
 * t0[0..2] : 三类改正的参考时刻，0=orbit, 1=code bias, 2=clock。
 * udi[0..2]: 与上一次同类更新的时间间隔。打印头部里的 udi 来自这里。
 * iodssr[] : 三类改正携带的 SSR IOD。
 * iodcorr[]: orbit/clock 内部的 correction IOD，用来区分同一卫星改正批次。
 * deph[]   : 径向/切向/法向轨道改正。
 * ddeph[]  : 阶段 1 保留字段，当前 Unicore 2304 打印时保持参考格式。
 * cbias[]  : 按 RTKLIB code index 存差分码偏差。
 * dclk[]   : 钟差多项式，当前 2308 主要填 c0。
 * update   : 当前 frame 解出了这颗卫星，打印后由 finalize_update() 清零。
 */
typedef struct {
    int sow;
    int verify_sow;
    gtime_t t0[3];
    double udi[3];
    int iodssr[3];
    int iodp;
    int iodn;
    uint16_t iodcorr[2];
    double deph[3];
    double ddeph[3];
    int ura;
    float cbias[MAXCODE + 1];
    double dclk[3];
    int update;
} b2b_ssr_t;

/*
 * 整个文件解码上下文。
 *
 * buff/nbyte/len 是流式读文件的 Unicore 帧缓存状态。
 * raw_time/geoprn/mask/ssr 是当前或最近一次解码出的业务状态。
 * *_count 是 summary 的数据来源：
 * - 四类业务计数在各 decode_PPPPB2BINFOx() 成功打印后累加。
 * - CRC/FRAME/UNKNOWN 在校验、边界检查或消息分发时累加。
 */
typedef struct {
    uint8_t buff[UNICORE_MAX_LEN + 4];
    int nbyte;
    int len;
    gtime_t raw_time;
    int geoprn;
    b2b_mask_t mask;
    b2b_ssr_t ssr[B2B_DECODER_MAXSAT];
    b2b_ssr_t prev_ssr[B2B_DECODER_MAXSAT];
    int mask_count;
    int orbit_count;
    int code_bias_count;
    int clock_count;
    int crc_error_count;
    int frame_error_count;
    int unknown_count;
} b2b_decoder_t;

static const int b2b_bds_codebias_mode[B2B_CODE_BIAS_MODE_NUM] = {
    CODE_L2I, CODE_L1D, CODE_L1P, CODE_NONE, CODE_L5D,
    CODE_L5P, CODE_NONE, CODE_L7I, CODE_L7Q, CODE_NONE,
    CODE_NONE, CODE_NONE, CODE_L6I, CODE_NONE, CODE_NONE
};
static const int b2b_gps_codebias_mode[B2B_CODE_BIAS_MODE_NUM] = {
    CODE_L1C, CODE_L1P, CODE_NONE, CODE_NONE, CODE_L1L,
    CODE_L1X, CODE_NONE, CODE_L2L, CODE_L2X, CODE_NONE,
    CODE_NONE, CODE_L5I, CODE_L5Q, CODE_L5X, CODE_NONE
};
static const int b2b_glo_codebias_mode[B2B_CODE_BIAS_MODE_NUM] = {
    CODE_L1C, CODE_L1P, CODE_L2C, CODE_NONE, CODE_NONE,
    CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE,
    CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE
};
static const int b2b_gal_codebias_mode[B2B_CODE_BIAS_MODE_NUM] = {
    CODE_NONE, CODE_L1B, CODE_L1X, CODE_NONE, CODE_L5Q,
    CODE_L5I, CODE_NONE, CODE_L7I, CODE_L7Q, CODE_NONE,
    CODE_NONE, CODE_L6C, CODE_NONE, CODE_NONE, CODE_NONE
};

static const char *rtklib_codes[MAXCODE + 1] = {
    "", "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E",
    "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P",
    "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X",
    "6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8I", "8Q", "8X",
    "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A",
    "5B", "5C", "9A", "9B", "9C", "9X", "1D", "5D", "5P", "5Z",
    "6E", "7D", "7P", "7Z", "8D", "8P", "4A", "4B", "4X"
};

/*
 * Unicore 帧字段是 little-endian，小端读取函数集中放在这里。
 * 例如 message id 在 header bytes 4..5，所以 get_u2(buff+4) 得到 2302 等。
 */
static uint8_t get_u1(const uint8_t *p)
{
    return p[0];
}

static uint16_t get_u2(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t get_s2(const uint8_t *p)
{
    return (int16_t)get_u2(p);
}

static uint32_t get_u4(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int time_is_zero(gtime_t t)
{
    return t.time == 0 && t.sec == 0.0;
}

/*
 * 在字节流中寻找 Unicore 同步头 AA 44 B5。
 *
 * B2bBin 不是“每次 fread 就天然对齐一帧”，而是一串连续二进制字节。
 * 这里用 3 字节滑动窗口，每读入一个新字节就丢掉最老的一个字节：
 *
 *   old buff[1], old buff[2], data
 *
 * 当窗口正好等于 AA 44 B5 时，说明后续字节可以按 Unicore header 解释。
 * 找到同步头后，调用方把 nbyte 设为 3，表示 buff[0..2] 已经是帧开头。
 */
static int sync_unicore(uint8_t *buff, uint8_t data)
{
    buff[0] = buff[1];
    buff[1] = buff[2];
    buff[2] = data;
    return buff[0] == UNICORE_SYNC1 && buff[1] == UNICORE_SYNC2 &&
           buff[2] == UNICORE_SYNC3;
}

#ifndef B2B_USE_RTKCMN
/*
 * 默认构建路径下的最小 RTKLIB 兼容实现。
 *
 * NOTE: 阶段 1 需要 rtk_crc32/gpst2time/time2epoch/satno 等基础函数，
 * 但不想把 rtklib/src 的主流程或大结构拉进来。因此这里复制了足够小的一组
 * 时间、CRC 和卫星号转换工具。Makefile 仍保留 USE_RTKCMN=1 作为可选路径。
 */
#define POLYCRC32 0xEDB88320u
#define MINPRNGPS 1
#define MAXPRNGPS 32
#define NSATGPS 32
#define MINPRNGLO 1
#define MAXPRNGLO 27
#define NSATGLO 27
#define MINPRNGAL 1
#define MAXPRNGAL 36
#define NSATGAL 36
#define MINPRNQZS 193
#define MAXPRNQZS 202
#define NSATQZS 10
#define MINPRNCMP 1
#define MAXPRNCMP 63
#define NSATCMP 63
#define MINPRNIRN 0
#define MAXPRNIRN 0
#define NSATIRN 0
#define MINPRNLEO 0
#define MAXPRNLEO 0
#define NSATLEO 0
#define MINPRNSBS 120
#define MAXPRNSBS 158
#define NSATSBS 39
#define RTK_MAXSAT (NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATSBS + NSATLEO)

static const double gpst0[] = {1980, 1, 6, 0, 0, 0};

static gtime_t epoch2time_local(const double *ep)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = {0};
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 +
           (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

static void time2epoch(gtime_t t, double *ep)
{
    const int mday[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days = (int)(t.time / 86400);
    int sec = (int)(t.time - (time_t)days * 86400);
    int mon, day;

    for (day = days % 1461, mon = 0; mon < 48; mon++) {
        if (day >= mday[mon]) day -= mday[mon];
        else break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}

static gtime_t gpst2time(int week, double sec)
{
    gtime_t t = epoch2time_local(gpst0);

    if (sec < -1E9 || 1E9 < sec) sec = 0.0;
    t.time += (time_t)86400 * 7 * week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}

static gtime_t timeadd_local(gtime_t t, double sec)
{
    double tt;

    t.sec += sec;
    tt = floor(t.sec);
    t.time += (int)tt;
    t.sec -= tt;
    return t;
}

static double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

static gtime_t gpst2bdt(gtime_t t)
{
    return timeadd_local(t, -14.0);
}

static double time2doy(gtime_t t)
{
    double ep[6];

    time2epoch(t, ep);
    ep[1] = ep[2] = 1.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timediff(t, epoch2time_local(ep)) / 86400.0 + 1.0;
}

static uint32_t rtk_crc32(const uint8_t *buff, int len)
{
    uint32_t crc = 0;

    /*
     * 与 RTKLIB rtk_crc32() 兼容的 CRC32。
     * 调用处会传 ctx->buff 和 ctx->len，也就是从 sync 开始到 payload 结束，
     * 不包含末尾 4 字节 CRC 本身。
     */
    for (int i = 0; i < len; i++) {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ POLYCRC32 : crc >> 1;
        }
    }
    return crc;
}

static int satno(int sys, int prn)
{
    if (prn <= 0) return 0;
    switch (sys) {
    case SYS_GPS:
        if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
        return prn - MINPRNGPS + 1;
    case SYS_GLO:
        if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
        return NSATGPS + prn - MINPRNGLO + 1;
    case SYS_GAL:
        if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
        return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
    case SYS_QZS:
        if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
    case SYS_CMP:
        if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
    case SYS_SBS:
        if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN +
               NSATLEO + prn - MINPRNSBS + 1;
    default:
        return 0;
    }
}

static int satsys(int sat, int *prn)
{
    int sys = SYS_NONE;

    if (sat <= 0 || RTK_MAXSAT < sat) sat = 0;
    else if (sat <= NSATGPS) {
        sys = SYS_GPS;
        sat += MINPRNGPS - 1;
    }
    else if ((sat -= NSATGPS) <= NSATGLO) {
        sys = SYS_GLO;
        sat += MINPRNGLO - 1;
    }
    else if ((sat -= NSATGLO) <= NSATGAL) {
        sys = SYS_GAL;
        sat += MINPRNGAL - 1;
    }
    else if ((sat -= NSATGAL) <= NSATQZS) {
        sys = SYS_QZS;
        sat += MINPRNQZS - 1;
    }
    else if ((sat -= NSATQZS) <= NSATCMP) {
        sys = SYS_CMP;
        sat += MINPRNCMP - 1;
    }
    else if ((sat -= NSATCMP) <= NSATSBS) {
        sys = SYS_SBS;
        sat += MINPRNSBS - 1;
    }
    else {
        sat = 0;
    }
    if (prn) *prn = sat;
    return sys;
}

static void satno2id(int sat, char *id)
{
    int prn;

    switch (satsys(sat, &prn)) {
    case SYS_GPS:
        sprintf(id, "G%02d", prn - MINPRNGPS + 1);
        return;
    case SYS_GLO:
        sprintf(id, "R%02d", prn - MINPRNGLO + 1);
        return;
    case SYS_GAL:
        sprintf(id, "E%02d", prn - MINPRNGAL + 1);
        return;
    case SYS_QZS:
        sprintf(id, "J%02d", prn - MINPRNQZS + 1);
        return;
    case SYS_CMP:
        sprintf(id, "C%02d", prn - MINPRNCMP + 1);
        return;
    case SYS_SBS:
        sprintf(id, "%03d", prn);
        return;
    default:
        strcpy(id, "");
        return;
    }
}
#endif

static mjd_gtime_t mjd_time_add(mjd_gtime_t t0, double dt)
{
    mjd_gtime_t t1 = t0;

    t1.sod += dt;
    while (t1.sod >= 86400.0) {
        t1.sod -= 86400.0;
        t1.mjd++;
    }
    while (t1.sod < 0.0) {
        t1.sod += 86400.0;
        t1.mjd--;
    }
    return t1;
}

static double mjd_time_diff(mjd_gtime_t t1, mjd_gtime_t t0)
{
    return (t1.mjd - t0.mjd) * 86400.0 + (t1.sod - t0.sod);
}

static double hms_to_sod(int hh, int mm, double sec)
{
    return hh * 3600.0 + mm * 60.0 + sec;
}

static int yy_to_yyyy(int yy)
{
    if (yy <= 50) return yy + 2000;
    if (yy > 50 && yy < 1900) return yy + 1900;
    return yy;
}

static mjd_gtime_t ymdhms_to_mjd_time(const double *date)
{
    mjd_gtime_t tt = {0};
    int yyyy = yy_to_yyyy((int)floor(date[0]));
    int month = (int)floor(date[1]);
    int day = (int)floor(date[2]);
    double jd;

    tt.sod = hms_to_sod((int)floor(date[3]), (int)floor(date[4]), date[5]);
    if (month <= 2) {
        yyyy--;
        month += 12;
    }
    jd = floor(365.25 * yyyy + 1.0e-9) +
         floor(30.6001 * (month + 1) + 1.0e-9) + day + 1720981.5;
    tt.mjd = (int)floor(jd - 2400000.5);
    return tt;
}

static mjd_gtime_t yrdoy_to_mjd_time(int year, int doy, double sod)
{
    double date[6] = {0};
    mjd_gtime_t jan1;

    date[0] = year;
    date[1] = 1;
    date[2] = 1;
    jan1 = ymdhms_to_mjd_time(date);
    return mjd_time_add(jan1, (doy - 1) * 86400.0 + sod);
}

static mjd_gtime_t bdst_to_gpst_mjd(mjd_gtime_t tt_bds)
{
    return mjd_time_add(tt_bds, 14.0);
}

static int mjd_time_to_gpst(mjd_gtime_t tt, int *week, double *sow)
{
    static const double gpst0[] = {1980, 1, 6, 0, 0, 0};
    mjd_gtime_t t_gpst0 = ymdhms_to_mjd_time(gpst0);
    double delta_day = mjd_time_diff(tt, t_gpst0) / 86400.0;
    int week_tmp = (int)floor(delta_day / 7.0 + 1.0e-9);
    int dow = (int)floor(delta_day - week_tmp * 7 + 1.0e-9);

    if (week) *week = week_tmp;
    if (sow) *sow = dow * 86400.0 + tt.sod;
    return dow;
}

static int adjday_b2b(double header_sod, int header_doy, double data_sod)
{
    double dt = header_sod - data_sod;

    /*
     * B2b payload 只给当天秒 data_sod，不直接给年月日。这里用帧头时间
     * header_sod 估计它属于同一天、前一天还是后一天。
     */
    if (dt > 43200.0) return header_doy + 1;
    if (dt < -43200.0) return header_doy - 1;
    return header_doy;
}

static void adjyear_b2b(int header_year, int *data_year, int *data_doy)
{
    int maxday = ((header_year % 4 == 0 && header_year % 100 != 0) ||
                  header_year % 400 == 0)
                     ? 366
                     : 365;

    if (*data_doy > maxday) {
        *data_year = header_year + 1;
        *data_doy = 1;
    }
    else if (*data_doy < 1) {
        *data_year = header_year - 1;
        *data_doy = ((header_year - 1) % 4 == 0 && (header_year - 1) % 100 != 0) ||
                            (header_year - 1) % 400 == 0
                        ? 366
                        : 365;
    }
    else {
        *data_year = header_year;
    }
}

static gtime_t b2btod2time(gtime_t header_time, double data_sod)
{
    double header_ep[6], data_sow;
    int header_year, header_doy, header_sod, data_year, data_doy, data_week;
    gtime_t header_bdt = gpst2bdt(header_time);
    mjd_gtime_t mjdsod;

    /*
     * raw_time 是 Unicore 帧头给出的 GPS time；B2b payload 的 SOD 是 BDT
     * day-of-second。先把帧头 GPS time 转 BDT，再用 payload SOD 做跨日修正，
     * 最后再转回 GPS time，保证输出 ref_time 能和参考工程按同一时间系打印。
     */
    time2epoch(header_bdt, header_ep);
    header_year = (int)header_ep[0];
    header_sod = (int)header_ep[3] * 3600 + (int)header_ep[4] * 60 +
                 (int)header_ep[5];
    header_doy = (int)time2doy(header_bdt);
    data_doy = adjday_b2b(header_sod, header_doy, data_sod);
    adjyear_b2b(header_year, &data_year, &data_doy);

    mjdsod = yrdoy_to_mjd_time(data_year, data_doy, data_sod);
    mjdsod = bdst_to_gpst_mjd(mjdsod);
    mjd_time_to_gpst(mjdsod, &data_week, &data_sow);
    return gpst2time(data_week, data_sow);
}

static int slot2satno(int slot)
{
    int sys, prn;

    /*
     * B2b mask 不是直接存 G01/C19 这种 PRN，而是一个连续 slot 空间：
     *   1..63    BDS
     *   64..100  GPS
     *   101..137 Galileo
     *   138..174 GLONASS
     * 阶段 1 打印时需要转成 RTKLIB sat number，再由 satno2id() 打成 Cxx/Gxx。
     */
    if (slot >= B2B_BDS_MINSAT && slot <= B2B_BDS_MAXSAT) {
        sys = SYS_CMP;
        prn = slot - B2B_BDS_MINSAT + 1;
    }
    else if (slot >= B2B_GPS_MINSAT && slot <= B2B_GPS_MAXSAT) {
        sys = SYS_GPS;
        prn = slot - B2B_GPS_MINSAT + 1;
    }
    else if (slot >= B2B_GAL_MINSAT && slot <= B2B_GAL_MAXSAT) {
        sys = SYS_GAL;
        prn = slot - B2B_GAL_MINSAT + 1;
    }
    else if (slot >= B2B_GLO_MINSAT && slot <= B2B_GLO_MAXSAT) {
        sys = SYS_GLO;
        prn = slot - B2B_GLO_MINSAT + 1;
    }
    else {
        return 0;
    }
    return satno(sys, prn);
}

static void merge_mask_arrays(const b2b_mask_t *mask, int *merged)
{
    int index = 0;

    memcpy(merged + index, mask->mask_bd, sizeof(mask->mask_bd));
    index += 63;
    memcpy(merged + index, mask->mask_gps, sizeof(mask->mask_gps));
    index += 37;
    memcpy(merged + index, mask->mask_gal, sizeof(mask->mask_gal));
    index += 37;
    memcpy(merged + index, mask->mask_glo, sizeof(mask->mask_glo));
}

static int mask2satno(b2b_mask_t *mask)
{
    int merged[B2B_MAXSAT] = {0};
    int n = 0;

    /*
     * MASK payload 中四个系统分开给 bit mask。这里先合并成 B2b slot 顺序，
     * 再把 bit=1 的 slot 转为 satno[]，供 CLOCK 的 subtype/index 定位使用。
     */
    memset(mask->satno, 0, sizeof(mask->satno));
    merge_mask_arrays(mask, merged);
    for (int i = 0; i < B2B_MAXSAT; i++) {
        if (merged[i]) {
            int sat = slot2satno(i + 1);
            if (sat > 0) mask->satno[n++] = sat;
        }
    }
    mask->satnum = n;
    return n;
}

static void mask_to_binary(const uint8_t mask[32], int binary[256])
{
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 8; j++) {
            /* MASK bit 按高位到低位解释：byte bit7 是当前 8 个 slot 的第一个。 */
            binary[i * 8 + j] = (mask[i] >> (7 - j)) & 1;
        }
    }
}

static void mask_to_str(const int *mask, int len, char *str)
{
    for (int i = 0; i < len; i++) {
        str[i] = mask[i] ? '1' : '0';
    }
    str[len] = '\0';
}

static void satno_to_prn(const b2b_mask_t *mask, char *bds_prn, size_t bds_len,
                         char *gps_prn, size_t gps_len, char *gal_prn,
                         size_t gal_len, char *glo_prn, size_t glo_len)
{
    bds_prn[0] = gps_prn[0] = gal_prn[0] = glo_prn[0] = '\0';

    for (int i = 0; i < mask->satnum; i++) {
        char prn_str[8];
        int prn = 0;
        int sat = mask->satno[i];
        int sys = sat > 0 ? satsys(sat, &prn) : 0;
        char *dst = NULL;
        size_t dst_len = 0;

        switch (sys) {
        case SYS_CMP:
            snprintf(prn_str, sizeof(prn_str), "C%02d ", prn);
            dst = bds_prn;
            dst_len = bds_len;
            break;
        case SYS_GPS:
            snprintf(prn_str, sizeof(prn_str), "G%02d ", prn);
            dst = gps_prn;
            dst_len = gps_len;
            break;
        case SYS_GAL:
            snprintf(prn_str, sizeof(prn_str), "E%02d ", prn);
            dst = gal_prn;
            dst_len = gal_len;
            break;
        case SYS_GLO:
            snprintf(prn_str, sizeof(prn_str), "R%02d ", prn);
            dst = glo_prn;
            dst_len = glo_len;
            break;
        default:
            break;
        }
        if (dst && strlen(dst) + strlen(prn_str) + 1 < dst_len) {
            strcat(dst, prn_str);
        }
    }
}

static int updated_sat_count(const b2b_decoder_t *ctx)
{
    int n = 0;

    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        if (ctx->ssr[i].update) n++;
    }
    return n;
}

/*
 * 打印完成后的收尾。
 *
 * 每类消息先把本帧涉及的卫星标记 update=1，print_b2b_infoX() 只打印这些卫星。
 * 打印后这里计算与上一帧同类改正的间隔 udi，并把当前 ssr 保存到 prev_ssr。
 * 最后清 update，避免下一类消息误打印上一帧残留卫星。
 */
static void finalize_update(b2b_decoder_t *ctx, int type_index)
{
    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        if (!ctx->ssr[i].update) continue;
        if (!time_is_zero(ctx->prev_ssr[i].t0[type_index])) {
            ctx->ssr[i].udi[type_index] =
                timediff(ctx->ssr[i].t0[type_index], ctx->prev_ssr[i].t0[type_index]);
            if (ctx->ssr[i].udi[type_index] > 86400.0) {
                ctx->ssr[i].udi[type_index] = 0.0;
            }
        }
        else {
            ctx->ssr[i].udi[type_index] = 0.0;
        }
        ctx->prev_ssr[i] = ctx->ssr[i];
        ctx->ssr[i].update = 0;
    }
}

/*
 * 打印 2302 MASK。
 *
 * MASK 的业务含义：告诉接收机当前 B2b 改正覆盖哪些卫星，并给出 iod_ssr/iodp。
 * 后续 ORBIT/CODE/CLOCK 都要用这些 IOD 做一致性检查，否则不同批次的数据混用。
 */
static void print_b2b_info1(FILE *out, const b2b_decoder_t *ctx, int udi)
{
    double recv_ep[6], ref_ep[6];
    char str_mask_bd[64], str_mask_gps[38], str_mask_gal[38], str_mask_glo[38];
    char bds_prn[512], gps_prn[512], gal_prn[512], glo_prn[512];

    mask_to_str(ctx->mask.mask_bd, 63, str_mask_bd);
    mask_to_str(ctx->mask.mask_gps, 37, str_mask_gps);
    mask_to_str(ctx->mask.mask_gal, 37, str_mask_gal);
    mask_to_str(ctx->mask.mask_glo, 37, str_mask_glo);
    satno_to_prn(&ctx->mask, bds_prn, sizeof(bds_prn), gps_prn, sizeof(gps_prn),
                 gal_prn, sizeof(gal_prn), glo_prn, sizeof(glo_prn));

    time2epoch(ctx->mask.ref_time, ref_ep);
    time2epoch(ctx->raw_time, recv_ep);

    fprintf(out,
            "> MASK %04d %02d %02d %02d %02d %.1f %d %d B2bSatPrn_%d @ "
            "%04d %02d %02d %02d %02d %.1f\n",
            (int)ref_ep[0], (int)ref_ep[1], (int)ref_ep[2], (int)ref_ep[3],
            (int)ref_ep[4], ref_ep[5], udi, ctx->mask.satnum, ctx->geoprn,
            (int)recv_ep[0], (int)recv_ep[1], (int)recv_ep[2], (int)recv_ep[3],
            (int)recv_ep[4], recv_ep[5]);
    fprintf(out, "IOD:      %d            %d\n", ctx->mask.iod_ssr, ctx->mask.iodp);
    fprintf(out, "BDSMASK:  %s\n", str_mask_bd);
    fprintf(out, "GPSMASK:  %s\n", str_mask_gps);
    fprintf(out, "GALMASK:  %s\n", str_mask_gal);
    fprintf(out, "GLOMASK:  %s\n", str_mask_glo);
    fprintf(out, "BDS PRN:  %s\n", bds_prn);
    fprintf(out, "GPS PRN:  %s\n", gps_prn);
    fprintf(out, "GAL PRN:  %s\n", gal_prn);
    fprintf(out, "GLO PRN:  %s\n", glo_prn);
}

/*
 * 打印 2304 ORBIT_URAI。
 *
 * ORBIT 是卫星轨道改正，deph[0..2] 分别是径向、切向、法向。
 * URAI 是用户距离精度指标，表示这组轨道改正的精度等级。
 */
static void print_b2b_info2(FILE *out, const b2b_decoder_t *ctx)
{
    double recv_ep[6], ref_ep[6] = {0};
    int udi = 0, satnum = 0;

    time2epoch(ctx->raw_time, recv_ep);
    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        if (!ctx->ssr[i].update) continue;
        satnum++;
        udi = (int)ctx->ssr[i].udi[0];
        time2epoch(ctx->ssr[i].t0[0], ref_ep);
    }

    fprintf(out,
            "> ORBIT_URAI %04d %02d %02d %02d %02d %.1f %d %d B2bSatPrn_%d @ "
            "%04d %02d %02d %02d %02d %.1f\n",
            (int)ref_ep[0], (int)ref_ep[1], (int)ref_ep[2], (int)ref_ep[3],
            (int)ref_ep[4], ref_ep[5], udi, satnum, ctx->geoprn,
            (int)recv_ep[0], (int)recv_ep[1], (int)recv_ep[2], (int)recv_ep[3],
            (int)recv_ep[4], recv_ep[5]);

    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        char satid[16];
        const b2b_ssr_t *b2b = &ctx->ssr[i];

        if (!b2b->update) continue;
        satno2id(i, satid);
        fprintf(out, "%-4s %5d %5d %5d %5d %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n",
                satid, b2b->iodssr[0], b2b->iodn, b2b->iodcorr[0], b2b->ura,
                b2b->deph[0], b2b->deph[1], b2b->deph[2],
                b2b->ddeph[0] * 1000.0, b2b->ddeph[1] * 1000.0,
                b2b->ddeph[2] * 1000.0);
    }
}

/*
 * 打印 2306 DIFF_CODE_BIAS。
 *
 * 差分码偏差 DCB 用来修正不同信号码之间的硬件延迟差。
 * payload 里先给 mode，再按系统映射到 RTKLIB 的 code index，例如 GPS L1C/L5I。
 */
static void print_b2b_info3(FILE *out, const b2b_decoder_t *ctx)
{
    double recv_ep[6], ref_ep[6] = {0};
    int udi = 0, satnum = 0;

    time2epoch(ctx->raw_time, recv_ep);
    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        if (!ctx->ssr[i].update) continue;
        satnum++;
        udi = (int)ctx->ssr[i].udi[1];
        time2epoch(ctx->ssr[i].t0[1], ref_ep);
    }

    fprintf(out,
            "> DIFF_CODE_BIAS %04d %02d %02d %02d %02d %.1f %d %d B2bSatPrn_%d @ "
            "%04d %02d %02d %02d %02d %.1f\n",
            (int)ref_ep[0], (int)ref_ep[1], (int)ref_ep[2], (int)ref_ep[3],
            (int)ref_ep[4], ref_ep[5], udi, satnum, ctx->geoprn,
            (int)recv_ep[0], (int)recv_ep[1], (int)recv_ep[2], (int)recv_ep[3],
            (int)recv_ep[4], recv_ep[5]);

    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        char satid[16];
        const b2b_ssr_t *b2b = &ctx->ssr[i];
        int bias_num = 0;

        if (!b2b->update) continue;
        satno2id(i, satid);
        for (int j = 0; j <= MAXCODE; j++) {
            if (b2b->cbias[j] != 0.0f) bias_num++;
        }
        fprintf(out, "%-4s %5d %5d", satid, b2b->iodssr[1], bias_num);
        for (int j = 0; j <= MAXCODE; j++) {
            if (b2b->cbias[j] != 0.0f) {
                fprintf(out, " %7s %7.4f", rtklib_codes[j], b2b->cbias[j]);
            }
        }
        fprintf(out, "\n");
    }
}

/*
 * 打印 2308 CLOCK。
 *
 * CLOCK 是卫星钟差改正。当前 Unicore 2308 解析主要填 dclk[0]，
 * 打印时仍保留 dclk[1]/dclk[2] 列以贴近参考工程格式。
 */
static void print_b2b_info4(FILE *out, const b2b_decoder_t *ctx)
{
    double recv_ep[6], ref_ep[6] = {0};
    int udi = 0, satnum = 0;

    time2epoch(ctx->raw_time, recv_ep);
    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        if (!ctx->ssr[i].update) continue;
        satnum++;
        udi = (int)ctx->ssr[i].udi[2];
        time2epoch(ctx->ssr[i].t0[2], ref_ep);
    }

    fprintf(out,
            "> CLOCK %04d %02d %02d %02d %02d %.1f %d %d B2bSatPrn_%d @ "
            "%04d %02d %02d %02d %02d %.1f\n",
            (int)ref_ep[0], (int)ref_ep[1], (int)ref_ep[2], (int)ref_ep[3],
            (int)ref_ep[4], ref_ep[5], udi, satnum, ctx->geoprn,
            (int)recv_ep[0], (int)recv_ep[1], (int)recv_ep[2], (int)recv_ep[3],
            (int)recv_ep[4], recv_ep[5]);

    for (int i = 1; i < B2B_DECODER_MAXSAT; i++) {
        char satid[16];
        const b2b_ssr_t *b2b = &ctx->ssr[i];

        if (!b2b->update) continue;
        satno2id(i, satid);
        fprintf(out, "%-4s %5d %5d %10.4f %10.4f %10.4f\n", satid,
                b2b->iodssr[2], b2b->iodp, b2b->dclk[0],
                b2b->dclk[1] * 1000.0, b2b->dclk[2] * 1000.0);
    }
}

/*
 * 解码 2302 PPPPB2BINFO1 / MASK。
 *
 * payload 布局中本函数用到的关键偏移：
 *   +0  int16  GEO PRN，参考工程转换为 B2bSatPrn_x
 *   +2  uint8  iod_ssr
 *   +3  uint8  iodp
 *   +4  uint32 BDT day-of-second，转换成 ref_time
 *   +8  32字节 mask bit field，覆盖 BDS/GPS/GAL/GLO slot
 *
 * summary 计数：只有实际打印 MASK 后才执行 ctx->mask_count++。
 */
static int decode_PPPPB2BINFO1(b2b_decoder_t *ctx, const uint8_t *payload,
                               int payload_len, FILE *out)
{
    int binary[256] = {0};
    gtime_t new_ref_time;
    int udi;

    if (payload_len < 40) {
        /* 边界检查：至少要能读到 +8 后的 32 字节 mask。 */
        ctx->frame_error_count++;
        return -1;
    }

    ctx->geoprn = (int)get_s2(payload) - 161 + 1;
    /* GEO PRN 62 在参考工程中跳过；这里保持同样行为，避免输出多一类无效帧。 */
    if (ctx->geoprn == 62) return 0;

    new_ref_time = b2btod2time(ctx->raw_time, get_u4(payload + 4));
    udi = time_is_zero(ctx->mask.ref_time)
              ? 0
              : (int)timediff(new_ref_time, ctx->mask.ref_time);

    memset(&ctx->mask, 0, sizeof(ctx->mask));
    ctx->mask.recv_time = ctx->raw_time;
    ctx->mask.ref_time = new_ref_time;
    ctx->mask.iod_ssr = get_u1(payload + 2);
    ctx->mask.iodp = get_u1(payload + 3);

    mask_to_binary(payload + 8, binary);
    for (int i = 0; i < 63; i++) ctx->mask.mask_bd[i] = binary[i];
    for (int i = 63; i < 100; i++) ctx->mask.mask_gps[i - 63] = binary[i];
    for (int i = 100; i < 137; i++) ctx->mask.mask_gal[i - 100] = binary[i];
    for (int i = 137; i < 174; i++) ctx->mask.mask_glo[i - 137] = binary[i];

    mask2satno(&ctx->mask);
    print_b2b_info1(out, ctx, udi);
    /* summary: MASK 行计数在这里累加。 */
    ctx->mask_count++;
    return 20;
}

/*
 * 解码 2304 PPPPB2BINFO2 / ORBIT_URAI。
 *
 * payload 关键偏移：
 *   +0  GEO PRN
 *   +2  iod_ssr，必须等于当前 MASK 的 iod_ssr
 *   +4  BDT SOD
 *   +8  后面固定 6 颗卫星，每颗 12 字节：
 *       +0 slot, +2 iodn, +4 radial, +6 in-track, +8 cross,
 *       +10 iodcorr, +11 ura
 *
 * WARNING: 如果还没读到匹配 MASK，或者 iod_ssr 不一致，本帧会被跳过。
 */
static int decode_PPPPB2BINFO2(b2b_decoder_t *ctx, const uint8_t *payload,
                               int payload_len, FILE *out)
{
    uint32_t sow;
    gtime_t ref_time;
    double ref_ep[6];
    int verify_sow;

    if (payload_len < 80) {
        /* 8 字节公共头 + 6 * 12 字节卫星块。 */
        ctx->frame_error_count++;
        return -1;
    }

    ctx->geoprn = (int)get_s2(payload) - 161 + 1;
    if (ctx->geoprn == 62) return 0;

    if ((int)get_u1(payload + 2) != ctx->mask.iod_ssr) return 0;

    sow = get_u4(payload + 4);
    ref_time = b2btod2time(ctx->raw_time, sow);
    time2epoch(ref_time, ref_ep);
    verify_sow = (int)(ref_ep[3] * 3600.0 + ref_ep[4] * 60.0 + ref_ep[5]);

    for (int i = 0; i < 6; i++) {
        const uint8_t *p = payload + 8 + i * 12;
        int sat = slot2satno(get_u2(p));
        int radial = get_s2(p + 4);
        int in_track = get_s2(p + 6);
        int cross = get_s2(p + 8);
        b2b_ssr_t *ssr;

        if (sat <= 0 || sat >= B2B_DECODER_MAXSAT) continue;
        ssr = &ctx->ssr[sat];
        ssr->iodssr[0] = get_u1(payload + 2);
        ssr->iodn = get_u2(p + 2);
        ssr->iodcorr[0] = get_u1(p + 10);
        ssr->t0[0] = ref_time;
        ssr->sow = (int)sow;
        ssr->verify_sow = verify_sow;

        if (abs(radial) >= 16383 || abs(in_track) >= 4095 || abs(cross) >= 4095) {
            continue;
        }
        ssr->deph[0] = radial * 0.0016;
        ssr->deph[1] = in_track * 0.0064;
        ssr->deph[2] = cross * 0.0064;
        ssr->ura = get_u1(p + 11);
        ssr->update = 1;
    }

    if (!updated_sat_count(ctx)) return 0;
    print_b2b_info2(out, ctx);
    /* summary: ORBIT_URAI section 计数在成功打印后累加。 */
    ctx->orbit_count++;
    finalize_update(ctx, 0);
    return 20;
}

/*
 * 解码 2306 PPPPB2BINFO3 / DIFF_CODE_BIAS。
 *
 * payload 关键偏移：
 *   +0  GEO PRN
 *   +2  iod_ssr，必须等于当前 MASK 的 iod_ssr
 *   +3  satnum，本帧携带多少颗卫星的 DCB
 *   +4  BDT SOD
 *   +8  后面每颗卫星固定 64 字节：
 *       +0 slot, +2 bias_num, +4 起若干 (mode, corr) 对，每对 4 字节
 *
 * NOTE: mode 只是 B2b 内部信号码，需要按系统映射到 RTKLIB code index。
 */
static int decode_PPPPB2BINFO3(b2b_decoder_t *ctx, const uint8_t *payload,
                               int payload_len, FILE *out)
{
    int satnum;
    uint32_t sow;
    gtime_t ref_time;
    double ref_ep[6];
    int verify_sow;

    if (payload_len < 8) {
        /* 至少需要公共头，才能读 satnum 和 SOD。 */
        ctx->frame_error_count++;
        return -1;
    }

    satnum = get_u1(payload + 3);
    if (payload_len < 8 + satnum * 64) {
        /* 防止 satnum 异常导致读取超过 payload。 */
        ctx->frame_error_count++;
        return -1;
    }

    ctx->geoprn = (int)get_s2(payload) - 161 + 1;
    if (ctx->geoprn == 62) return 0;

    if ((int)get_u1(payload + 2) != ctx->mask.iod_ssr) return 0;

    sow = get_u4(payload + 4);
    ref_time = b2btod2time(ctx->raw_time, sow);
    time2epoch(ref_time, ref_ep);
    verify_sow = (int)(ref_ep[3] * 3600.0 + ref_ep[4] * 60.0 + ref_ep[5]);

    for (int i = 0; i < satnum; i++) {
        const uint8_t *p = payload + 8 + i * 64;
        int sat = slot2satno(get_u2(p));
        int bias_num = get_u2(p + 2);
        int sys;
        const int *cods = NULL;
        b2b_ssr_t *ssr;

        if (sat <= 0 || sat >= B2B_DECODER_MAXSAT) continue;
        sys = satsys(sat, NULL);
        if (sys == SYS_GPS) cods = b2b_gps_codebias_mode;
        else if (sys == SYS_GLO) cods = b2b_glo_codebias_mode;
        else if (sys == SYS_GAL) cods = b2b_gal_codebias_mode;
        else if (sys == SYS_CMP) cods = b2b_bds_codebias_mode;
        else continue;

        ssr = &ctx->ssr[sat];
        ssr->iodssr[1] = get_u1(payload + 2);
        ssr->t0[1] = ref_time;
        ssr->sow = (int)sow;
        ssr->verify_sow = verify_sow;

        if (bias_num > 15) bias_num = 15;
        for (int j = 0; j < bias_num; j++) {
            const uint8_t *q = p + 4 + j * 4;
            int mode = get_u2(q);
            int corr = get_s2(q + 2);
            int type;

            if (abs(corr) >= 2103) continue;
            if (mode < 0 || mode >= B2B_CODE_BIAS_MODE_NUM) continue;
            type = cods[mode];
            if (type == CODE_NONE || type > MAXCODE) continue;
            ssr->cbias[type] = (float)(corr * 0.017);
            ssr->update = 1;
        }
    }

    if (!updated_sat_count(ctx)) return 0;
    print_b2b_info3(out, ctx);
    /* summary: DIFF_CODE_BIAS section 计数在成功打印后累加。 */
    ctx->code_bias_count++;
    finalize_update(ctx, 1);
    return 20;
}

/*
 * 解码 2308 PPPPB2BINFO4 / CLOCK。
 *
 * payload 关键偏移：
 *   +0  GEO PRN
 *   +2  iod_ssr，必须等于当前 MASK 的 iod_ssr
 *   +3  iodp，必须等于当前 MASK 的 iodp
 *   +4  BDT SOD
 *   +8  subtype，一个 CLOCK 子帧覆盖 23 个 mask slot
 *   +12 后面 23 组钟差，每组 4 字节：iodcorr + c0
 *
 * subtype * 23 得到当前 CLOCK 子帧在 MASK 卫星列表中的起始位置。
 */
static int decode_PPPPB2BINFO4(b2b_decoder_t *ctx, const uint8_t *payload,
                               int payload_len, FILE *out)
{
    uint32_t sow;
    gtime_t ref_time;
    double ref_ep[6];
    int verify_sow, subtype, begin;

    if (payload_len < 104) {
        /* 12 字节公共/子类型区域 + 23 * 4 字节钟差。 */
        ctx->frame_error_count++;
        return -1;
    }

    ctx->geoprn = (int)get_s2(payload) - 161 + 1;
    if (ctx->geoprn == 62) return 0;

    if ((int)get_u1(payload + 2) != ctx->mask.iod_ssr ||
        (int)get_u1(payload + 3) != ctx->mask.iodp) {
        return 0;
    }

    subtype = get_u1(payload + 8);
    if (subtype > 31) return 0;
    begin = subtype * 23;

    sow = get_u4(payload + 4);
    ref_time = b2btod2time(ctx->raw_time, sow);
    time2epoch(ref_time, ref_ep);
    verify_sow = (int)(ref_ep[3] * 3600.0 + ref_ep[4] * 60.0 + ref_ep[5]);

    for (int i = 0; i < 23; i++) {
        int mask_index = begin + i;
        const uint8_t *p = payload + 12 + i * 4;
        int iodcorr = get_u2(p);
        int c0 = get_s2(p + 2);
        int sat;
        b2b_ssr_t *ssr;

        if (mask_index < 0 || mask_index >= B2B_MAXSAT) continue;
        sat = ctx->mask.satno[mask_index];
        if (sat <= 0 || sat >= B2B_DECODER_MAXSAT) continue;

        ssr = &ctx->ssr[sat];
        ssr->t0[2] = ref_time;
        ssr->sow = (int)sow;
        ssr->verify_sow = verify_sow;
        ssr->iodssr[2] = ctx->mask.iod_ssr;
        ssr->iodp = ctx->mask.iodp;
        ssr->iodcorr[1] = (uint16_t)iodcorr;
        if (abs(c0) >= 16383 || iodcorr > 7) continue;
        ssr->dclk[0] = c0 * 0.0016;
        ssr->update = 1;
    }

    if (!updated_sat_count(ctx)) return 0;
    print_b2b_info4(out, ctx);
    /* summary: CLOCK section 计数在成功打印后累加。 */
    ctx->clock_count++;
    finalize_update(ctx, 2);
    return 20;
}

static int decode_unicore_frame(b2b_decoder_t *ctx, FILE *out)
{
    /* header bytes 4..5 是 Unicore message id，小端：2302/2304/2306/2308。 */
    int type = get_u2(ctx->buff + 4);
    int stat, week, payload_len;
    double tow;
    /* payload 紧跟 24 字节 header，CRC 在 ctx->buff + ctx->len。 */
    const uint8_t *payload = ctx->buff + UNICORE_HEADER_LEN;

    /*
     * CRC 校验范围：
     *   rtk_crc32(ctx->buff, ctx->len)
     * 覆盖 sync + header + payload，也就是从 AA 44 B5 到 payload 最后一个字节。
     * 不包含末尾 4 字节 CRC。接收机写入的 CRC 存在 ctx->buff + ctx->len。
     */
    if (rtk_crc32(ctx->buff, ctx->len) != get_u4(ctx->buff + ctx->len)) {
        ctx->crc_error_count++;
        return -1;
    }

    /*
     * header byte 9 是时间状态，bytes 10..11 是 GPS week。
     * stat==201 或 week==0 时参考工程会跳过，因为此时帧头时间不可用于
     * B2b SOD -> 完整 ref_time 的换算。
     */
    stat = get_u1(ctx->buff + 9);
    week = get_u2(ctx->buff + 10);
    if (stat == 201 || week == 0) return 0;

    /* bytes 12..15 是 GPS milliseconds-of-week，换成秒后得到 raw_time。 */
    tow = get_u4(ctx->buff + 12) * 0.001;
    ctx->raw_time = gpst2time(week, tow);
    payload_len = ctx->len - UNICORE_HEADER_LEN;

    /*
     * 消息 ID 分发：
     * 2302/2304/2306/2308 分别进入 MASK/ORBIT_URAI/DIFF_CODE_BIAS/CLOCK。
     * 其他 Unicore 消息本阶段不解析，只计入 UNKNOWN。
     */
    switch (type) {
    case PPPPB2BINFO1:
        return decode_PPPPB2BINFO1(ctx, payload, payload_len, out);
    case PPPPB2BINFO2:
        return decode_PPPPB2BINFO2(ctx, payload, payload_len, out);
    case PPPPB2BINFO3:
        return decode_PPPPB2BINFO3(ctx, payload, payload_len, out);
    case PPPPB2BINFO4:
        return decode_PPPPB2BINFO4(ctx, payload, payload_len, out);
    default:
        ctx->unknown_count++;
        return 0;
    }
}

static void print_summary(FILE *out, const b2b_decoder_t *ctx)
{
    /* summary 的格式和计数口径固定，用于和参考 postdecoder -U 输出做数量对比。 */
    fprintf(out, "\nSUMMARY\n");
    fprintf(out, "  MASK:           %d\n", ctx->mask_count);
    fprintf(out, "  ORBIT_URAI:     %d\n", ctx->orbit_count);
    fprintf(out, "  DIFF_CODE_BIAS: %d\n", ctx->code_bias_count);
    fprintf(out, "  CLOCK:          %d\n", ctx->clock_count);
    fprintf(out, "  CRC_ERROR:      %d\n", ctx->crc_error_count);
    fprintf(out, "  FRAME_ERROR:    %d\n", ctx->frame_error_count);
    fprintf(out, "  UNKNOWN:        %d\n", ctx->unknown_count);
}

/*
 * b2b_decode_unicore_file() 是阶段 1 的整体流程入口。
 *
 * 从 B2bBin 到 txt 的主链路：
 *   open file
 *     -> byte-by-byte scan
 *     -> sync AA 44 B5
 *     -> read 24-byte Unicore header
 *     -> get payload length from header bytes 6..7
 *     -> wait until header + payload + CRC all received
 *     -> CRC check
 *     -> message id dispatch
 *     -> decode/print 2302/2304/2306/2308
 *     -> summary
 *
 * WARNING: 这个函数只读一个 Unicore/UM980 B2bBin 文件。输入文件由 CLI 的
 *          argv[1] 传入，代码内部没有硬编码数据文件路径。
 */
int b2b_decode_unicore_file(const char *path, FILE *out)
{
    b2b_decoder_t ctx;
    FILE *fp;
    int ch;

    if (!path || !out) return -1;
    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "test_b2b_decoder: failed to open input file: %s\n", path);
        return -1;
    }

    memset(&ctx, 0, sizeof(ctx));
    while ((ch = fgetc(fp)) != EOF) {
        uint8_t data = (uint8_t)ch;

        if (ctx.nbyte == 0) {
            /*
             * 当前没有进入一帧时，就持续寻找 AA 44 B5。
             * 找到后 nbyte=3，表示 buff[0..2] 已经保存同步头。
             */
            if (sync_unicore(ctx.buff, data)) ctx.nbyte = 3;
            continue;
        }

        ctx.buff[ctx.nbyte++] = data;
        if (ctx.nbyte == 8) {
            /*
             * 读够 header 的前 8 字节后，bytes 6..7 已经可用。
             * Unicore payload length 是 uint16 小端；总校验长度 ctx.len
             * 是 24 字节 header + payload，不含最后 4 字节 CRC。
             */
            ctx.len = (int)get_u2(ctx.buff + 6) + UNICORE_HEADER_LEN;
            if (ctx.len < UNICORE_HEADER_LEN || ctx.len > UNICORE_MAX_LEN) {
                /* 长度异常时丢弃当前帧，重新找同步头，避免缓冲区越界。 */
                ctx.frame_error_count++;
                ctx.nbyte = 0;
                continue;
            }
        }
        /*
         * ctx.len 只有在 nbyte==8 后才有效；ctx.len+4 表示再等 4 字节 CRC。
         * 只有完整帧到齐后才调用 decode_unicore_frame()。
         */
        if (ctx.nbyte < 8 || ctx.nbyte < ctx.len + 4) continue;

        decode_unicore_frame(&ctx, out);
        /* 一帧处理结束，回到同步搜索状态，继续扫描后面的字节流。 */
        ctx.nbyte = 0;
    }

    fclose(fp);
    print_summary(out, &ctx);
    return 0;
}
