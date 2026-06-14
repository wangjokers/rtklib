#include "rtklib.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308

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
    gtime_t first_time;
    gtime_t last_time;
} baseline_t;

extern int b2b_replay_input_format(const char *path);
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

static int frame_type(const raw_t *raw)
{
    return raw->buff[4]|(raw->buff[5]<<8);
}

static int type_index(int type)
{
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
    return !memcmp(a->B2bssr,b->B2bssr,sizeof(a->B2bssr));
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

static int decode_full_file(const char *path, nav_t *nav, baseline_t *stats)
{
    const B2bmask_t *mask;
    raw_t *raw,*other;
    FILE *fp;
    int ret,index,n,sat;

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
    if (!init_raw(raw,STRFMT_UNICORE)||!init_raw(other,STRFMT_UNICORE)) {
        free_raw(raw);
        free_raw(other);
        fclose(fp);
        free(raw);
        free(other);
        return 0;
    }
    stats->raw_consumed=1;
    for (;;) {
        ret=input_rawf(raw,STRFMT_UNICORE,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;

        if (!stats->first_time.time) stats->first_time=raw->time;
        stats->last_time=raw->time;
        index=type_index(frame_type(raw));
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
        n=b2b_update_nav_from_raw(nav,raw);
        if (index>0&&n<=0) stats->raw_consumed=0;
        if (raw_update_count(raw)!=0) stats->raw_consumed=0;
    }
    mask=unicore_b2b_mask(other);
    stats->context_isolated=mask&&mask->IOD_SSR==-1&&mask->IODP==-1&&
                            mask->satnum==0;
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
           b2b_replay_input_format("a.rnx")<0;
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

static int test_epoch_schedule(const char *path, gtime_t first_time)
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
    if (!b2b_replay_open(replay,path,STRFMT_UNICORE)) {
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

static int test_full_replay(const char *path, gtime_t last_time,
                            const nav_t *baseline)
{
    b2b_replay_t *replay;
    nav_t *nav;
    B2bssr_t *before;
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
    if (!b2b_replay_open(replay,path,STRFMT_UNICORE)) {
        free(replay);
        free(nav);
        free(before);
        return 0;
    }

    ret=b2b_replay_update(replay,nav,timeadd(last_time,86400.0));
    ok&=ret>0&&replay->eof&&!replay->pending;
    ok&=raw_update_count(&replay->raw)==0;
    ok&=nav_products_equal(nav,baseline);
    ok&=nav->B2bssr[0].update==0;

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

int main(int argc, char **argv)
{
    baseline_t stats={0};
    nav_t *baseline;
    int formats,index_bounds,schedule,full,counts,ok;

    if (argc!=2) {
        fprintf(stderr,"Usage: %s <Unicore_B2bBin>\n",argv[0]);
        return 1;
    }
    if (!(baseline=(nav_t *)calloc(1,sizeof(*baseline)))) return 1;
    if (!decode_full_file(argv[1],baseline,&stats)) {
        free(baseline);
        return 1;
    }

    formats=test_input_formats();
    index_bounds=test_index_bounds();
    schedule=test_epoch_schedule(argv[1],stats.first_time);
    full=test_full_replay(argv[1],stats.last_time,baseline);
    counts=stats.frames[0]==3602&&stats.frames[1]==13420&&
           stats.frames[2]==13294&&stats.frames[3]==86410&&
           stats.errors==0&&stats.unknown==0&&stats.context_isolated;

    printf("MASK %ld\n",stats.frames[0]);
    printf("ORBIT_URAI %ld\n",stats.frames[1]);
    printf("DIFF_CODE_BIAS %ld\n",stats.frames[2]);
    printf("CLOCK %ld\n",stats.frames[3]);
    printf("CRC_OR_FRAME_ERROR %d\n",stats.errors);
    printf("UNKNOWN %d\n",stats.unknown);
    printf("CONTEXT_ISOLATION %d\n",stats.context_isolated);
    printf("RAW_UPDATE_CONSUMED %d\n",stats.raw_consumed);
    printf("INPUT_FORMATS %d\n",formats);
    printf("INDEX_BOUNDS %d\n",index_bounds);
    printf("EPOCH_LOOKAHEAD %d\n",schedule);
    printf("FULL_REPLAY_MATCH %d\n",full);
    printf("DECODER_COUNTS %d\n",counts);

    ok=stats.raw_consumed&&formats&&index_bounds&&schedule&&full&&counts;
    free(baseline);
    return ok?0:1;
}
