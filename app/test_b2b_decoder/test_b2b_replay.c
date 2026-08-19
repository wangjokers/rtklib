#include "rtklib.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308
#define SINO_HEADER_LEN 28
#define SINO_TYPE_POS   (SINO_HEADER_LEN*8+44)

typedef struct {
    raw_t raw;
    FILE *fp;
    char path[MAXSTRPATH];
    int format;
    int initialized;
    int eof;
    int pending;
    gtime_t pending_rx_time;
} b2b_replay_t;

typedef struct {
    long frames[4];
    long sat_updates[4];
    int errors;
    int unknown;
    int raw_consumed;
    int context_isolated;
    int index_zero_unchanged;
    gtime_t first_time;
    gtime_t last_time;
} baseline_t;

extern int b2b_replay_input_format(const char *path);
extern int b2b_replay_format_from_option(int option);
extern int b2b_replay_open(b2b_replay_t *replay, const char *path, int format);
extern void b2b_replay_close(b2b_replay_t *replay);
extern int b2b_replay_update(b2b_replay_t *replay, nav_t *nav,
                             gtime_t obs_time);

int showmsg(const char *format, ...)
{
    va_list arg;

    va_start(arg,format);
    vfprintf(stderr,format,arg);
    va_end(arg);
    return 0;
}

void settspan(gtime_t ts, gtime_t te)
{
    (void)ts;
    (void)te;
}

void settime(gtime_t time)
{
    (void)time;
}

static int frame_type(const raw_t *raw, int format)
{
    if (format==STRFMT_UNICORE) return raw->buff[4]|(raw->buff[5]<<8);
    if (format==STRFMT_SINO&&raw->len>=SINO_HEADER_LEN&&
        SINO_TYPE_POS+6<=raw->len*8) {
        return (int)getbitu(raw->buff,SINO_TYPE_POS,6);
    }
    return -1;
}

static int type_index(int type, int format)
{
    if (format==STRFMT_SINO) return 1<=type&&type<=4?type-1:-1;
    if (type==UNICORE_MSG_B2B_INFO1) return 0;
    if (type==UNICORE_MSG_B2B_INFO2) return 1;
    if (type==UNICORE_MSG_B2B_INFO3) return 2;
    if (type==UNICORE_MSG_B2B_INFO4) return 3;
    return -1;
}

static int raw_update_count(const raw_t *raw)
{
    int sat,n=0;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (raw->nav.B2bssr[sat].update) n++;
    }
    return n;
}

static int nav_update_count(const nav_t *nav)
{
    int sat,n=0;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (nav->B2bssr[sat].update) n++;
    }
    return n;
}

static int nav_products_equal(const nav_t *a, const nav_t *b)
{
    int sat;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (memcmp(a->B2bssr+sat,b->B2bssr+sat,
                   sizeof(a->B2bssr[sat]))) return 0;
    }
    return 1;
}

static int nav_products_empty(const nav_t *nav)
{
    B2bssr_t zero={0};
    int sat;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (memcmp(nav->B2bssr+sat,&zero,sizeof(zero))) return 0;
    }
    return 1;
}

static long cbias_valid_count(const nav_t *nav, int *valid_zero)
{
    long n=0;
    int sat,code;

    *valid_zero=0;
    for (sat=1;sat<=MAXSAT;sat++) {
        for (code=1;code<=MAXCODE;code++) {
            if (!nav->B2bssr[sat].cbias_valid[code]) continue;
            n++;
            if (nav->B2bssr[sat].cbias[code]==0.0f) *valid_zero=1;
        }
    }
    return n;
}

static int decode_full_file(const char *path, int format, nav_t *nav,
                            baseline_t *stats)
{
    const B2bmask_t *mask;
    raw_t *raw,*other;
    B2bssr_t raw_zero,nav_zero;
    FILE *fp;
    int ret,index,n,pending,sat;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(other=(raw_t *)calloc(1,sizeof(*other)))) {
        free(raw);
        free(other);
        return 0;
    }
    if (!(fp=fopen(path,"rb"))) {
        free(raw);
        free(other);
        return 0;
    }
    if (!init_raw(raw,format)||!init_raw(other,format)) {
        free_raw(raw);
        free_raw(other);
        fclose(fp);
        free(raw);
        free(other);
        return 0;
    }
    raw->nav.B2bssr[0].iodn=711;
    raw->nav.B2bssr[0].cbias_valid[CODE_L1C]=1;
    raw->nav.B2bssr[0].update=7;
    nav->B2bssr[0].iodn=722;
    nav->B2bssr[0].cbias_valid[CODE_L2I]=1;
    nav->B2bssr[0].update=5;
    raw_zero=raw->nav.B2bssr[0];
    nav_zero=nav->B2bssr[0];
    stats->raw_consumed=1;
    for (;;) {
        ret=input_rawf(raw,format,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;

        if (!stats->first_time.time) stats->first_time=raw->time;
        stats->last_time=raw->time;
        index=type_index(frame_type(raw,format),format);
        if (index<0) {
            stats->unknown++;
            continue;
        }
        stats->frames[index]++;
        if (index>0) {
            for (sat=1;sat<=MAXSAT;sat++) {
                if (raw->nav.B2bssr[sat].update) stats->sat_updates[index]++;
            }
        }
        pending=raw_update_count(raw);
        n=b2b_update_nav_from_raw(nav,raw);
        if (n!=pending) stats->raw_consumed=0;
        if (raw_update_count(raw)!=0) stats->raw_consumed=0;
    }
    mask=format==STRFMT_UNICORE?unicore_b2b_mask(other):
                               sino_b2b_mask(other);
    stats->context_isolated=mask&&mask->IOD_SSR==-1&&mask->IODP==-1&&
                            mask->satnum==0;
    stats->index_zero_unchanged=
        !memcmp(raw->nav.B2bssr,&raw_zero,sizeof(raw_zero))&&
        !memcmp(nav->B2bssr,&nav_zero,sizeof(nav_zero));
    free_raw(raw);
    free_raw(other);
    fclose(fp);
    free(raw);
    free(other);
    return 1;
}

static int test_input_formats(void)
{
    return b2b_replay_input_format("a.B2bBin")==STRFMT_UNICORE&&
           b2b_replay_input_format("a.b2b")==STRFMT_UNICORE&&
           b2b_replay_input_format("a.B2b")==STRFMT_UNICORE&&
           b2b_replay_input_format("a.txt")<0&&
           b2b_replay_input_format("a.rnx")<0;
}

static int test_config_options(void)
{
    opt_t *format=searchopt("pos1-b2bformat",sysopts);
    opt_t *path=searchopt("file-b2brawfile",sysopts);
    char text[MAXSTRPATH];
    int ok=1;

    if (!format||!path) return 0;
    ok&=str2opt(format,"sino");
    ok&=*(int *)format->var==B2BFMT_SINO;
    memset(text,0,sizeof(text));
    opt2str(format,text);
    ok&=!strcmp(text,"sino");
    ok&=b2b_replay_format_from_option(*(int *)format->var)==STRFMT_SINO;

    ok&=str2opt(format,"unicore");
    ok&=*(int *)format->var==B2BFMT_UNICORE;
    ok&=b2b_replay_format_from_option(*(int *)format->var)==STRFMT_UNICORE;
    ok&=str2opt(format,"off");
    ok&=*(int *)format->var==B2BFMT_OFF;
    ok&=b2b_replay_format_from_option(*(int *)format->var)<0;

    ok&=str2opt(path,"urum-explicit.txt");
    ok&=!strcmp((const char *)path->var,"urum-explicit.txt");
    return ok;
}

static int test_open_formats(const char *unicore_path, const char *sino_path)
{
    b2b_replay_t *unicore,*sino,*invalid;
    int ok=0;

    unicore=(b2b_replay_t *)calloc(1,sizeof(*unicore));
    sino=(b2b_replay_t *)calloc(1,sizeof(*sino));
    invalid=(b2b_replay_t *)calloc(1,sizeof(*invalid));
    if (!unicore||!sino||!invalid) goto done;

    ok=b2b_replay_open(unicore,unicore_path,STRFMT_UNICORE)&&
       b2b_replay_open(sino,sino_path,STRFMT_SINO)&&
       !b2b_replay_open(invalid,sino_path,STRFMT_RTCM2);
done:
    if (unicore) b2b_replay_close(unicore);
    if (sino) b2b_replay_close(sino);
    if (invalid) b2b_replay_close(invalid);
    free(unicore);
    free(sino);
    free(invalid);
    return ok;
}

static int test_index_bounds(void)
{
    raw_t *raw=(raw_t *)calloc(1,sizeof(*raw));
    nav_t *nav=(nav_t *)calloc(1,sizeof(*nav));
    int ok=1;

    if (!raw||!nav) {
        free(raw);
        free(nav);
        return 0;
    }
    nav->B2bssr[0].iodn=77;
    raw->nav.B2bssr[0].iodn=88;
    raw->nav.B2bssr[0].update=1;
    raw->nav.B2bssr[MAXSAT].iodn=99;
    raw->nav.B2bssr[MAXSAT].update=1;

    ok&=b2b_update_nav_from_raw(nav,raw)==1;
    ok&=nav->B2bssr[0].iodn==77;
    ok&=raw->nav.B2bssr[0].update==1;
    ok&=nav->B2bssr[MAXSAT].iodn==99;
    ok&=nav->B2bssr[MAXSAT].update==1;
    ok&=raw->nav.B2bssr[MAXSAT].update==0;

    free(raw);
    free(nav);
    return ok;
}

static int test_epoch_schedule(const char *path, int format,
                               gtime_t first_time)
{
    b2b_replay_t *replay;
    nav_t *nav;
    B2bssr_t *raw_before,*nav_before;
    gtime_t obs_time,pending_time;
    int ok=1,ret,updates,loops=0;

    replay=(b2b_replay_t *)calloc(1,sizeof(*replay));
    nav=(nav_t *)calloc(1,sizeof(*nav));
    raw_before=(B2bssr_t *)malloc(sizeof(replay->raw.nav.B2bssr));
    nav_before=(B2bssr_t *)malloc(sizeof(nav->B2bssr));
    if (!replay||!nav||!raw_before||!nav_before) {
        free(replay);
        free(nav);
        free(raw_before);
        free(nav_before);
        return 0;
    }
    if (!b2b_replay_open(replay,path,format)) {
        free(replay);
        free(nav);
        free(raw_before);
        free(nav_before);
        return 0;
    }

    obs_time=timeadd(first_time,-1.0);
    ret=b2b_replay_update(replay,nav,obs_time);
    ok&=ret==0&&replay->pending&&!replay->eof;
    ok&=timediff(replay->pending_rx_time,obs_time)>DTTOL;
    ok&=nav_products_empty(nav);

    pending_time=replay->pending_rx_time;
    memcpy(raw_before,replay->raw.nav.B2bssr,sizeof(replay->raw.nav.B2bssr));
    ret=b2b_replay_update(replay,nav,obs_time);
    ok&=ret==0&&replay->pending;
    ok&=timediff(replay->pending_rx_time,pending_time)==0.0;
    ok&=!memcmp(raw_before,replay->raw.nav.B2bssr,
                sizeof(replay->raw.nav.B2bssr));
    ok&=nav_products_empty(nav);

    while (replay->pending&&raw_update_count(&replay->raw)==0&&loops++<100) {
        obs_time=replay->pending_rx_time;
        if (b2b_replay_update(replay,nav,obs_time)<0) {
            ok=0;
            break;
        }
    }
    updates=raw_update_count(&replay->raw);
    ok&=replay->pending&&updates>0;
    ok&=timediff(replay->pending_rx_time,obs_time)>DTTOL;

    pending_time=replay->pending_rx_time;
    memcpy(raw_before,replay->raw.nav.B2bssr,sizeof(replay->raw.nav.B2bssr));
    memcpy(nav_before,nav->B2bssr,sizeof(nav->B2bssr));
    ret=b2b_replay_update(replay,nav,obs_time);
    ok&=ret==0&&replay->pending;
    ok&=timediff(replay->pending_rx_time,pending_time)==0.0;
    ok&=!memcmp(raw_before,replay->raw.nav.B2bssr,
                sizeof(replay->raw.nav.B2bssr));
    ok&=!memcmp(nav_before,nav->B2bssr,sizeof(nav->B2bssr));

    ret=b2b_replay_update(replay,nav,pending_time);
    ok&=ret>0&&nav_update_count(nav)>0;
    if (replay->pending) {
        ok&=timediff(replay->pending_rx_time,pending_time)>DTTOL;
    }
    else {
        ok&=raw_update_count(&replay->raw)==0;
    }

    memcpy(nav_before,nav->B2bssr,sizeof(nav->B2bssr));
    ret=b2b_replay_update(replay,nav,pending_time);
    ok&=ret==0;
    ok&=!memcmp(nav_before,nav->B2bssr,sizeof(nav->B2bssr));

    b2b_replay_close(replay);
    free(replay);
    free(nav);
    free(raw_before);
    free(nav_before);
    return ok;
}

static int test_full_replay(const char *path, int format, gtime_t last_time,
                            const nav_t *baseline)
{
    b2b_replay_t *replay;
    nav_t *nav;
    B2bssr_t *before;
    B2bssr_t zero_before;
    int ok=1,ret;

    replay=(b2b_replay_t *)calloc(1,sizeof(*replay));
    nav=(nav_t *)calloc(1,sizeof(*nav));
    before=(B2bssr_t *)malloc(sizeof(nav->B2bssr));
    if (!replay||!nav||!before) {
        free(replay);
        free(nav);
        free(before);
        return 0;
    }
    nav->B2bssr[0].iodn=733;
    nav->B2bssr[0].cbias_valid[CODE_L5I]=1;
    nav->B2bssr[0].update=9;
    zero_before=nav->B2bssr[0];
    if (!b2b_replay_open(replay,path,format)) {
        free(replay);
        free(nav);
        free(before);
        return 0;
    }

    ret=b2b_replay_update(replay,nav,timeadd(last_time,86400.0));
    ok&=ret>0&&replay->eof&&!replay->pending;
    ok&=raw_update_count(&replay->raw)==0;
    ok&=nav_products_equal(nav,baseline);
    ok&=!memcmp(nav->B2bssr,&zero_before,sizeof(zero_before));

    memcpy(before,nav->B2bssr,sizeof(nav->B2bssr));
    ret=b2b_replay_update(replay,nav,timeadd(last_time,172800.0));
    ok&=ret==0;
    ok&=!memcmp(before,nav->B2bssr,sizeof(nav->B2bssr));

    b2b_replay_close(replay);
    free(replay);
    free(nav);
    free(before);
    return ok;
}

static int run_format_suite(const char *label, const char *path, int format,
                            const long expected[4], int require_valid_zero)
{
    baseline_t stats={0};
    nav_t *baseline;
    long valid_biases;
    int valid_zero,schedule,full,counts,cbias,ok;

    if (!(baseline=(nav_t *)calloc(1,sizeof(*baseline)))) return 0;
    if (!decode_full_file(path,format,baseline,&stats)) {
        free(baseline);
        return 0;
    }

    schedule=test_epoch_schedule(path,format,stats.first_time);
    full=test_full_replay(path,format,stats.last_time,baseline);
    valid_biases=cbias_valid_count(baseline,&valid_zero);
    cbias=valid_biases>0&&(!require_valid_zero||valid_zero);
    counts=stats.frames[0]==expected[0]&&stats.frames[1]==expected[1]&&
           stats.frames[2]==expected[2]&&stats.frames[3]==expected[3]&&
           stats.errors==0&&stats.unknown==0&&stats.context_isolated&&
           stats.index_zero_unchanged;

    printf("%s_MASK %ld\n",label,stats.frames[0]);
    printf("%s_ORBIT_URAI %ld\n",label,stats.frames[1]);
    printf("%s_DIFF_CODE_BIAS %ld\n",label,stats.frames[2]);
    printf("%s_CLOCK %ld\n",label,stats.frames[3]);
    printf("%s_CRC_OR_FRAME_ERROR %d\n",label,stats.errors);
    printf("%s_UNKNOWN %d\n",label,stats.unknown);
    printf("%s_CONTEXT_ISOLATION %d\n",label,stats.context_isolated);
    printf("%s_RAW_UPDATE_CONSUMED %d\n",label,stats.raw_consumed);
    printf("%s_INDEX_ZERO_UNCHANGED %d\n",label,
           stats.index_zero_unchanged);
    printf("%s_EPOCH_LOOKAHEAD %d\n",label,schedule);
    printf("%s_FULL_REPLAY_MATCH %d\n",label,full);
    printf("%s_CBIAS_VALID %ld\n",label,valid_biases);
    printf("%s_VALID_ZERO_CBIAS %d\n",label,valid_zero);
    printf("%s_DECODER_COUNTS %d\n",label,counts);

    ok=stats.raw_consumed&&schedule&&full&&cbias&&counts;
    free(baseline);
    return ok;
}

int main(int argc, char **argv)
{
    static const long unicore_expected[4]={3602,13420,13294,86410};
    static const long sino_expected[4]={6241,24697,26684,149749};
    int formats,config,open,index_bounds,unicore,sino,ok;

    if (argc!=3) {
        fprintf(stderr,"Usage: %s <Unicore_B2bBin> <Sino_URUM>\n",argv[0]);
        return 1;
    }

    formats=test_input_formats();
    config=test_config_options();
    open=test_open_formats(argv[1],argv[2]);
    index_bounds=test_index_bounds();
    unicore=run_format_suite("UNICORE",argv[1],STRFMT_UNICORE,
                             unicore_expected,0);
    sino=run_format_suite("SINO",argv[2],STRFMT_SINO,sino_expected,1);

    printf("INPUT_FORMATS %d\n",formats);
    printf("CONFIG_OPTIONS %d\n",config);
    printf("OPEN_FORMATS %d\n",open);
    printf("INDEX_BOUNDS %d\n",index_bounds);
    printf("UNICORE_REPLAY_TEST %d\n",unicore);
    printf("SINO_REPLAY_TEST %d\n",sino);

    ok=formats&&config&&open&&index_bounds&&unicore&&sino;
    return ok?0:1;
}
