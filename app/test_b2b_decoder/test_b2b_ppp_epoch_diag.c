#include "rtklib.h"
#include "rcv/unicore.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308

static const char *rinex_opt="-SYS=G -GL1C -GL2X";
static const int gps_prns[]={10,12,24,25,31,32};

typedef struct {
    long frames[4];
    long nav_updates;
    long errors;
    gtime_t last_time;
    gtime_t first_future_time;
} b2b_replay_stats_t;

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

static gtime_t diagnostic_time(void)
{
    double ep[]={2024,12,25,0,30,0.0};

    return epoch2time(ep);
}

static void time_text(gtime_t time, char *buff, int n)
{
    if (!time.time) {
        snprintf(buff,n,"none");
        return;
    }
    time2str(time,buff,0);
}

static int frame_index(const raw_t *raw)
{
    int type=raw->buff[4]|(raw->buff[5]<<8);

    if (type==UNICORE_MSG_B2B_INFO1) return 0;
    if (type==UNICORE_MSG_B2B_INFO2) return 1;
    if (type==UNICORE_MSG_B2B_INFO3) return 2;
    if (type==UNICORE_MSG_B2B_INFO4) return 3;
    return -1;
}

static const char *code_name(int code, char *buff, int n)
{
    const char *obs=code2obs((uint8_t)code);

    if (obs&&*obs) snprintf(buff,n,"%s",obs);
    else snprintf(buff,n,"code%d",code);
    return buff;
}

static void print_code_value(int code, double value)
{
    char name[16];

    printf(" %s(%d)=%.3f",code_name(code,name,sizeof(name)),code,value);
}

static int init_unicore_raw(raw_t *raw)
{
    memset(raw,0,sizeof(*raw));
    raw->format=STRFMT_UNICORE;
    return init_unicore_b2b(raw);
}

static int replay_b2b_until(const char *path, gtime_t target, nav_t *nav,
                            b2b_replay_stats_t *stats)
{
    raw_t *raw;
    FILE *fp;
    int ret,index;

    memset(stats,0,sizeof(*stats));
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))) {
        fprintf(stderr,"failed to allocate raw_t\n");
        return 0;
    }
    if (!(fp=fopen(path,"rb"))) {
        fprintf(stderr,"failed to open B2bBin: %s\n",path);
        free(raw);
        return 0;
    }
    if (!init_unicore_raw(raw)) {
        fprintf(stderr,"failed to initialize Unicore B2b raw context\n");
        fclose(fp);
        free(raw);
        return 0;
    }
    for (;;) {
        ret=input_unicoref(raw,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;
        if (timediff(raw->time,target)>DTTOL) {
            stats->first_future_time=raw->time;
            break;
        }
        if ((index=frame_index(raw))>=0) stats->frames[index]++;
        stats->last_time=raw->time;
        stats->nav_updates+=b2b_update_nav_from_raw(nav,raw);
    }
    free_unicore_b2b(raw);
    fclose(fp);
    free(raw);
    return 1;
}

static int read_inputs(const char *obs_path, int nnav, char **nav_paths,
                       gtime_t target, obs_t *obs, nav_t *nav, sta_t *sta)
{
    gtime_t zero={0};
    int i;

    memset(obs,0,sizeof(*obs));
    memset(nav,0,sizeof(*nav));
    memset(sta,0,sizeof(*sta));

    if (readrnxt(obs_path,1,target,target,0.0,rinex_opt,obs,nav,sta)<0) {
        fprintf(stderr,"failed to read observation RINEX: %s\n",obs_path);
        return 0;
    }
    for (i=0;i<nnav;i++) {
        if (readrnxt(nav_paths[i],1,zero,zero,0.0,"",obs,nav,NULL)<0) {
            fprintf(stderr,"failed to read navigation RINEX: %s\n",
                    nav_paths[i]);
            return 0;
        }
    }
    sortobs(obs);
    uniqnav(nav);
    if (obs->n<=0) {
        fprintf(stderr,"no observation records at diagnostic epoch: %s\n",
                obs_path);
    }
    if (nav->n<=0&&nav->ng<=0&&nav->ns<=0) {
        fprintf(stderr,"no broadcast navigation records were loaded\n");
    }
    return obs->n>0&&(nav->n>0||nav->ng>0||nav->ns>0);
}

static const obsd_t *find_obs_record(const obs_t *obs, gtime_t target, int sat)
{
    int i;

    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat&&fabs(timediff(obs->data[i].time,target))<=DTTOL) {
            return obs->data+i;
        }
    }
    return NULL;
}

static void print_obs_codes(const obsd_t *obs, const nav_t *nav)
{
    char name[16];
    int i;

    if (!obs) {
        printf("  OBS_PRESENT 0\n");
        return;
    }
    printf("  OBS_PRESENT 1\n");
    printf("  PPP_NFREQ_CODES");
    for (i=0;i<NFREQ;i++) {
        if (obs->code[i]<=CODE_NONE) {
            printf(" f%d=none",i+1);
            continue;
        }
        printf(" f%d=%s(%d)",i+1,code_name(obs->code[i],name,sizeof(name)),
               obs->code[i]);
        printf("/P=%.3f/L=%.3f/freq=%.3f",obs->P[i],obs->L[i],
               sat2freq(obs->sat,obs->code[i],nav));
    }
    printf("\n");

    printf("  OBS_ALL_CODES");
    for (i=0;i<NFREQ+NEXOBS;i++) {
        if (obs->code[i]<=CODE_NONE) continue;
        printf(" slot%d=%s(%d)",i+1,code_name(obs->code[i],name,sizeof(name)),
               obs->code[i]);
    }
    printf("\n");
}

static int count_matching_brdc_eph(const nav_t *nav, gtime_t target, int sat,
                                   int iodn, int *total, int *healthy,
                                   double *best_age)
{
    int i,match=0;

    *total=*healthy=0;
    *best_age=0.0;
    if (!nav||!nav->eph) return 0;
    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;
        double age;

        if (eph->sat!=sat) continue;
        (*total)++;
        if (!eph->svh) (*healthy)++;
        if (eph->svh||eph->iode!=iodn) continue;
        age=fabs(timediff(eph->toe,target));
        if (!match||age<*best_age) *best_age=age;
        match++;
    }
    return match;
}

static int print_cbias_valid(const B2bssr_t *b2b)
{
    int code,count=0;

    printf("  B2B_CBIAS_VALID");
    for (code=1;code<=MAXCODE;code++) {
        if (!b2b->cbias_valid[code]) continue;
        print_code_value(code,b2b->cbias[code]);
        count++;
    }
    if (!count) printf(" none");
    printf("\n");
    return count;
}

static int print_required_cbias_checks(const obsd_t *obs, const B2bssr_t *b2b,
                                       gtime_t target)
{
    int i,required=0,missing=0;

    if (!obs) {
        printf("  PPP_REQUIRED_CBIAS none\n");
        printf("  CODE_BIAS_MISMATCH 0\n");
        return 0;
    }
    printf("  PPP_REQUIRED_CBIAS");
    for (i=0;i<2&&i<NFREQ;i++) {
        double bias=0.0,age=0.0;
        char name[16];
        int ready;

        if (obs->code[i]<=CODE_NONE||obs->P[i]==0.0) continue;
        required++;
        ready=b2b_cbias_ready(target,b2b,obs->code[i],&bias,&age);
        if (!ready) missing++;
        printf(" f%d=%s(%d):ready=%d:bias=%.3f:age=%.0f",
               i+1,code_name(obs->code[i],name,sizeof(name)),obs->code[i],
               ready,bias,age);
    }
    if (!required) printf(" none");
    printf("\n");
    printf("  CODE_BIAS_MISMATCH %d\n",required>0&&missing>0);
    return required>0&&missing>0;
}

static int print_satellite_diag(const obs_t *obs, const nav_t *nav,
                                gtime_t target, int sat)
{
    const obsd_t *obs_record=find_obs_record(obs,target,sat);
    const B2bssr_t *b2b=nav->B2bssr+sat;
    double orbit_age=0.0,clock_age=0.0,best_eph_age=0.0;
    int orbit_ok,clock_ok,orbit_clock_ok,cbias_count;
    int eph_total=0,eph_healthy=0,eph_match;
    char id[16],t0[32],t1[32],t2[32];
    int mismatch;

    satno2id(sat,id);
    time_text(b2b->t0[0],t0,sizeof(t0));
    time_text(b2b->t0[1],t1,sizeof(t1));
    time_text(b2b->t0[2],t2,sizeof(t2));

    printf("SAT %s sat=%d\n",id,sat);
    print_obs_codes(obs_record,nav);

    orbit_ok=b2b_orbit_age_valid(target,b2b,&orbit_age);
    clock_ok=b2b_clock_age_valid(target,b2b,&clock_age);
    orbit_clock_ok=b2b_orbit_clock_ready(target,b2b,&orbit_age,&clock_age);
    printf("  B2B_ORBIT_CLOCK orbit_age_ok=%d clock_age_ok=%d combined_ready=%d "
           "orbit_age=%.0f clock_age=%.0f iodssr=%d/%d iodcorr=%u/%u "
           "iodn=%d ura=%d t0=%s/%s/%s\n",
           orbit_ok,clock_ok,orbit_clock_ok,orbit_age,clock_age,
           b2b->iodssr[0],b2b->iodssr[2],
           (unsigned int)b2b->iodcorr[0],(unsigned int)b2b->iodcorr[1],
           b2b->iodn,b2b->ura,t0,t1,t2);

    eph_match=count_matching_brdc_eph(nav,target,sat,b2b->iodn,&eph_total,
                                      &eph_healthy,&best_eph_age);
    printf("  BRDC_IOD_MATCH total=%d healthy=%d iod_match=%d best_toe_age=%.0f\n",
           eph_total,eph_healthy,eph_match,best_eph_age);

    cbias_count=print_cbias_valid(b2b);
    mismatch=print_required_cbias_checks(obs_record,b2b,target);
    printf("  CBIAS_VALID_COUNT %d\n",cbias_count);
    return mismatch;
}

int main(int argc, char **argv)
{
    obs_t obs;
    nav_t nav;
    sta_t sta;
    b2b_replay_stats_t stats;
    gtime_t target=diagnostic_time();
    char target_text[32],last_text[32],future_text[32];
    int i,mismatch_sats=0;

    if (argc<4) {
        fprintf(stderr,"Usage: %s <obs_rnx> <b2b_bin> <nav_rnx> [nav_rnx...]\n",
                argv[0]);
        return 1;
    }
    if (!read_inputs(argv[1],argc-3,argv+3,target,&obs,&nav,&sta)) {
        fprintf(stderr,"input read failed or empty\n");
        return 1;
    }
    if (!replay_b2b_until(argv[2],target,&nav,&stats)) {
        freeobs(&obs);
        freenav(&nav,0x7F);
        return 1;
    }

    time_text(target,target_text,sizeof(target_text));
    time_text(stats.last_time,last_text,sizeof(last_text));
    time_text(stats.first_future_time,future_text,sizeof(future_text));

    printf("TARGET_TIME %s GPST\n",target_text);
    printf("RINEX_OPT %s\n",rinex_opt);
    printf("OBS_RECORDS_AT_TARGET %d\n",obs.n);
    printf("STATION name=%s marker=%s antenna=%s %s\n",sta.name,sta.marker,
           sta.antdes,sta.antsno);
    printf("NAV_COUNTS eph=%d geph=%d seph=%d\n",nav.n,nav.ng,nav.ns);
    printf("B2B_REPLAY last=%s first_future=%s updates=%ld errors=%ld\n",
           last_text,future_text,stats.nav_updates,stats.errors);
    printf("B2B_FRAMES mask=%ld orbit=%ld code_bias=%ld clock=%ld\n",
           stats.frames[0],stats.frames[1],stats.frames[2],stats.frames[3]);

    for (i=0;i<(int)(sizeof(gps_prns)/sizeof(gps_prns[0]));i++) {
        int sat=satno(SYS_GPS,gps_prns[i]);

        if (sat<=0) continue;
        mismatch_sats+=print_satellite_diag(&obs,&nav,target,sat);
    }
    printf("SUMMARY_CODE_BIAS_MISMATCH_SATS %d\n",mismatch_sats);

    freeobs(&obs);
    freenav(&nav,0x7F);
    return 0;
}
