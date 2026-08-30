/* Minimal realtime PPP-B2b launcher for two RTKLIB time-tagged file streams. */
#include "rtklib.h"

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

typedef struct {
    const char *obs;
    const char *sino;
    const char *config;
    const char *output;
    const char *trace;
    const char *start_time;
    double speed;
    double timeout;
} cli_options_t;

typedef struct {
    int stream_type;
    char path[MAXSTRPATH];
    long expected_bytes;
} input_spec_t;

typedef struct {
    uint32_t rtcm_frames;
    uint32_t obs_epochs;
    uint32_t sino_eph_messages;
    uint32_t b2b_publish_events;
    int eph_sats;
    int orbit_sats;
    int clock_sats;
    int cbias_sats;
    int index0_unused;
} server_stats_t;

static volatile sig_atomic_t stop_requested=0;

/* RTKLIB application callbacks. */
extern int showmsg(const char *format, ...)
{
    va_list ap;

    va_start(ap,format);
    vfprintf(stderr,format,ap);
    fputc('\n',stderr);
    va_end(ap);
    return 0;
}

extern void settspan(gtime_t ts, gtime_t te)
{
    (void)ts;
    (void)te;
}

extern void settime(gtime_t time)
{
    (void)time;
}

static void on_signal(int sig)
{
    (void)sig;
    stop_requested=1;
}

static void usage(FILE *fp, const char *program)
{
    fprintf(fp,
        "usage: %s --obs INPUT --sino INPUT --config FILE --output FILE "
        "[--trace FILE] [--speed N] [--timeout SECONDS]\n"
        "\n"
        "INPUT is a time-tagged file or tcpcli://HOST:PORT. Both inputs must "
        "use the same mode.\n"
        "Files require RTKLIB time-tag sidecars (FILE.tag). --speed applies "
        "only to files.\n"
        "Historical TCP replay may set GPST with --start-time "
        "\"YYYY MM DD hh mm ss\".\n"
        "TCP mode waits for both connected peers to disconnect; --timeout 0 "
        "otherwise waits indefinitely.\n",
        program);
}

static int parse_double(const char *text, double *value)
{
    char *end;
    double parsed;

    errno=0;
    parsed=strtod(text,&end);
    if (errno||end==text||*end!='\0'||!isfinite(parsed)) return 0;
    *value=parsed;
    return 1;
}

static int parse_args(int argc, char **argv, cli_options_t *opt)
{
    int i;

    memset(opt,0,sizeof(*opt));
    opt->speed=1.0;
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"--help")||!strcmp(argv[i],"-h")) {
            usage(stdout,argv[0]);
            return 1;
        }
        if (i+1>=argc) return 0;
        if      (!strcmp(argv[i],"--obs"))     opt->obs=argv[++i];
        else if (!strcmp(argv[i],"--sino"))    opt->sino=argv[++i];
        else if (!strcmp(argv[i],"--config"))  opt->config=argv[++i];
        else if (!strcmp(argv[i],"--output"))  opt->output=argv[++i];
        else if (!strcmp(argv[i],"--trace"))   opt->trace=argv[++i];
        else if (!strcmp(argv[i],"--start-time")) opt->start_time=argv[++i];
        else if (!strcmp(argv[i],"--speed")) {
            if (!parse_double(argv[++i],&opt->speed)||opt->speed<=0.0) {
                return 0;
            }
        }
        else if (!strcmp(argv[i],"--timeout")) {
            if (!parse_double(argv[++i],&opt->timeout)||opt->timeout<0.0) {
                return 0;
            }
        }
        else return 0;
    }
    return opt->obs&&opt->sino&&opt->config&&opt->output?2:0;
}

static long file_size(const char *path)
{
    struct stat st;
    return !stat(path,&st)?(long)st.st_size:-1L;
}

static int tagged_path(char *dst, size_t size, const char *path, double speed)
{
    int n=snprintf(dst,size,"%s::T::x%.6f::P=4",path,speed);
    return n>0&&(size_t)n<size;
}

static int input_spec(const char *text, double speed, input_spec_t *spec)
{
    static const char tcpcli_prefix[]="tcpcli://";
    int n;

    memset(spec,0,sizeof(*spec));
    if (!strncmp(text,tcpcli_prefix,sizeof(tcpcli_prefix)-1)) {
        text+=sizeof(tcpcli_prefix)-1;
        if (!*text) return 0;
        n=snprintf(spec->path,sizeof(spec->path),"%s",text);
        if (n<=0||(size_t)n>=sizeof(spec->path)) return 0;
        spec->stream_type=STR_TCPCLI;
        spec->expected_bytes=-1L;
        return 1;
    }
    spec->stream_type=STR_FILE;
    spec->expected_bytes=file_size(text);
    return spec->expected_bytes>0&&
           tagged_path(spec->path,sizeof(spec->path),text,speed);
}

static const eph_t *find_current_eph(const nav_t *nav, int sat)
{
    const eph_t *eph;
    int set;

    if (!nav||!nav->eph||sat<=0||sat>MAXSAT) return NULL;
    for (set=0;set<2;set++) {
        eph=nav->eph+sat-1+MAXSAT*set;
        if (eph->sat==sat&&eph->toe.time) return eph;
    }
    return NULL;
}

static uint32_t sum_rtcm3_messages(const rtcm_t *rtcm)
{
    uint32_t total=0;
    int i;

    for (i=0;i<400;i++) total+=rtcm->nmsg3[i];
    return total;
}

static void collect_server_stats(const rtksvr_t *svr, server_stats_t *stats)
{
    B2bssr_t zero={0};
    int code,sat;

    memset(stats,0,sizeof(*stats));
    stats->rtcm_frames=sum_rtcm3_messages(&svr->rtcm[0]);
    stats->obs_epochs=svr->nmsg[0][0];
    stats->sino_eph_messages=svr->nmsg[1][1];
    stats->b2b_publish_events=svr->nmsg[1][8];
    stats->index0_unused=memcmp(svr->nav.B2bssr,&zero,sizeof(zero))==0;

    for (sat=1;sat<=MAXSAT;sat++) {
        const B2bssr_t *b2b=svr->nav.B2bssr+sat;
        int has_cbias=0;

        if (find_current_eph(&svr->nav,sat)) stats->eph_sats++;
        if (b2b->t0[0].time&&isfinite(b2b->deph[0])&&
            isfinite(b2b->deph[1])&&isfinite(b2b->deph[2])) {
            stats->orbit_sats++;
        }
        if (b2b->t0[2].time&&isfinite(b2b->dclk[0])) stats->clock_sats++;
        for (code=1;code<=MAXCODE;code++) {
            if (b2b->cbias_valid[code]&&isfinite(b2b->cbias[code])) {
                has_cbias=1;
                break;
            }
        }
        if (has_cbias) stats->cbias_sats++;
    }
}

static int count_solutions(const char *path, long *solutions, long *q6)
{
    char line[4096],date[64],time[64];
    double c1,c2,c3;
    FILE *fp;
    int quality;

    *solutions=*q6=0;
    if (!(fp=fopen(path,"r"))) return 0;
    while (fgets(line,sizeof(line),fp)) {
        if (line[0]=='%'||line[0]=='#') continue;
        if (sscanf(line,"%63s %63s %lf %lf %lf %d",date,time,
                   &c1,&c2,&c3,&quality)!=6) continue;
        (*solutions)++;
        if (quality==SOLQ_PPP) (*q6)++;
    }
    fclose(fp);
    return 1;
}

int main(int argc, char **argv)
{
    cli_options_t cli;
    input_spec_t obs_input,sino_input;
    server_stats_t stats;
    rtksvr_t *svr=NULL;
    prcopt_t prcopt=prcopt_default;
    solopt_t solopt[2]={solopt_default,solopt_default};
    gtime_t replay_time={0};
    int streams[8]={STR_NONE,STR_NONE,STR_NONE,STR_FILE,STR_NONE,
                    STR_NONE,STR_NONE,STR_NONE};
    int formats[3]={STRFMT_RTCM3,STRFMT_SINO,STRFMT_RTCM3};
    char errmsg[1024]="",obs_msg[1024],sino_msg[1024];
    char empty[]="",ephall[]="-EPHALL";
    char *paths[8],*cmds[3]={NULL,NULL,NULL};
    char *periodic[3]={NULL,NULL,NULL};
    char *rcvopts[3]={empty,ephall,empty};
    double nmeapos[3]={0.0,0.0,0.0};
    uint32_t obs_read=0,sino_read=0;
    long obs_expected,sino_expected,elapsed_ms=0,timeout_ms=0,quiet_ms=0;
    long solutions=0,q6=0;
    int arg_status,completed=0,trace_open=0,exit_status=1,tcp_mode=0;
    int obs_started=0,sino_started=0,obs_disconnected=0,sino_disconnected=0;

    arg_status=parse_args(argc,argv,&cli);
    if (arg_status==1) return 0;
    if (arg_status!=2) {
        usage(stderr,argv[0]);
        return 2;
    }
    if (!strcmp(cli.obs,cli.output)||!strcmp(cli.sino,cli.output)) {
        fprintf(stderr,"output must differ from both input files\n");
        return 2;
    }
    if (!input_spec(cli.obs,cli.speed,&obs_input)||
        !input_spec(cli.sino,cli.speed,&sino_input)) {
        fprintf(stderr,"invalid input path or file stat failed\n");
        return 2;
    }
    if (obs_input.stream_type!=sino_input.stream_type) {
        fprintf(stderr,"obs and sino inputs must use the same stream mode\n");
        return 2;
    }
    tcp_mode=obs_input.stream_type==STR_TCPCLI;
    if (tcp_mode&&fabs(cli.speed-1.0)>1E-12) {
        fprintf(stderr,"--speed applies only to file inputs\n");
        return 2;
    }
    if (cli.start_time&&(!tcp_mode||
        str2time(cli.start_time,0,(int)strlen(cli.start_time),&replay_time))) {
        fprintf(stderr,"--start-time requires TCP mode and GPST "
                       "\"YYYY MM DD hh mm ss\"\n");
        return 2;
    }
    obs_expected=obs_input.expected_bytes;
    sino_expected=sino_input.expected_bytes;
    streams[0]=obs_input.stream_type;
    streams[1]=sino_input.stream_type;
    if (cli.timeout>0.0) timeout_ms=(long)(cli.timeout*1000.0);

    resetsysopts();
    if (!loadopts(cli.config,sysopts)) {
        fprintf(stderr,"processing config load failed: %s\n",cli.config);
        return 2;
    }
    getsysopts(&prcopt,solopt,NULL);
    solopt[1]=solopt[0];

    paths[0]=obs_input.path; paths[1]=sino_input.path; paths[2]=empty;
    paths[3]=(char *)cli.output;
    paths[4]=empty; paths[5]=empty; paths[6]=empty; paths[7]=empty;

    signal(SIGINT,on_signal);
    signal(SIGTERM,on_signal);
    if (cli.trace) {
        traceopen(cli.trace);
        tracelevel(4);
        trace_open=1;
    }
    if (!(svr=(rtksvr_t *)calloc(1,sizeof(*svr)))) {
        fprintf(stderr,"rtksvr allocation failed\n");
        goto cleanup;
    }
    if (!rtksvrinit(svr)) {
        fprintf(stderr,"rtksvrinit failed\n");
        goto cleanup;
    }
    if (cli.start_time) timeset(gpst2utc(replay_time));
    if (!rtksvrstart(svr,10,65536,streams,paths,formats,0,cmds,periodic,
                     rcvopts,0,0,nmeapos,&prcopt,solopt,NULL,errmsg)) {
        fprintf(stderr,"rtksvrstart failed: %s\n",errmsg);
        goto cleanup;
    }
    if (tcp_mode) {
        /* A completed fixture connection is terminal; do not reconnect. */
        strsettimeout(svr->stream,0,-1);
        strsettimeout(svr->stream+1,0,-1);
    }

    while (!stop_requested) {
        int obs_state=0,sino_state=0,buffers_empty;

        sleepms(20);
        elapsed_ms+=20;
        rtksvrlock(svr);
        obs_read=svr->stream[0].inb;
        sino_read=svr->stream[1].inb;
        buffers_empty=svr->nb[0]==0&&svr->nb[1]==0;
        rtksvrunlock(svr);
        if (tcp_mode) {
            obs_state=strstat(svr->stream,obs_msg);
            sino_state=strstat(svr->stream+1,sino_msg);
            /* strstat() returns 3 while a connected stream is active. */
            if (obs_state>=2||obs_read>0) obs_started=1;
            if (sino_state>=2||sino_read>0) sino_started=1;
            if (obs_started&&obs_state<2&&
                (!strcmp(obs_msg,"disconnected")||
                 !strncmp(obs_msg,"recv error",10))) obs_disconnected=1;
            if (sino_started&&sino_state<2&&
                (!strcmp(sino_msg,"disconnected")||
                 !strncmp(sino_msg,"recv error",10))) sino_disconnected=1;
            if (obs_disconnected&&sino_disconnected&&buffers_empty) {
                quiet_ms+=20;
                completed=quiet_ms>=500;
            }
            else quiet_ms=0;
        }
        else {
            completed=obs_read==(uint32_t)obs_expected&&
                      sino_read==(uint32_t)sino_expected&&buffers_empty;
        }
        if (completed) break;
        if (timeout_ms>0&&elapsed_ms>=timeout_ms) break;
    }
    if (completed&&!tcp_mode) sleepms(250);

    rtksvrlock(svr);
    obs_read=svr->stream[0].inb;
    sino_read=svr->stream[1].inb;
    collect_server_stats(svr,&stats);
    rtksvrunlock(svr);
    rtksvrstop(svr,cmds);

    count_solutions(cli.output,&solutions,&q6);
    if (tcp_mode) {
        printf("rtppp: mode=tcp completed=%d obs_bytes=%u sino_bytes=%u "
               "obs_disconnected=%d sino_disconnected=%d\n",completed,
               obs_read,sino_read,obs_disconnected,sino_disconnected);
    }
    else {
        printf("rtppp: mode=file completed=%d obs_bytes=%u/%ld "
               "sino_bytes=%u/%ld\n",completed,obs_read,obs_expected,
               sino_read,sino_expected);
    }
    printf("rtppp: rtcm_frames=%u obs_epochs=%u sino_eph=%u b2b_events=%u "
           "eph_sats=%d orbit_sats=%d clock_sats=%d cbias_sats=%d "
           "index0_unused=%d\n",stats.rtcm_frames,stats.obs_epochs,
           stats.sino_eph_messages,stats.b2b_publish_events,stats.eph_sats,
           stats.orbit_sats,stats.clock_sats,stats.cbias_sats,
           stats.index0_unused);
    printf("rtppp: solutions=%ld q6=%ld\n",solutions,q6);
    exit_status=completed&&stats.rtcm_frames>0&&stats.sino_eph_messages>0&&
                stats.b2b_publish_events>0&&stats.index0_unused&&q6>0?0:1;

cleanup:
    if (svr) {
        rtksvrfree(svr);
        free(svr);
    }
    if (trace_open) traceclose();
    if (stop_requested) return 130;
    return exit_status;
}
