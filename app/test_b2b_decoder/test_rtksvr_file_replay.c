/* Phase 1: drive the unmodified RTKLIB realtime server with two STR_FILE inputs. */
#include "rtklib.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define SINO_HEADER_LEN 28
#define SINO_MSG_BD3EPH 72
#define SINO_MSG_B2B 1697
#define SINO_TYPE_POS (SINO_HEADER_LEN*8+44)
#define OBSERVED_BDS3_COUNT 9

static const int observed_bds3_prns[OBSERVED_BDS3_COUNT]={
    23,24,25,31,32,33,34,38,41
};

typedef struct {
    long frames;
    long message72;
    long message1697;
    long type1;
    long type2;
    long type3;
    long type4;
    long type63;
    long crc_errors;
    long length_errors;
} sino_fixture_stats_t;

typedef struct {
    int sino_format;
    uint32_t publish_events;
    int eph_sats;
    int b2b_sats;
    int orbit_sats;
    int clock_sats;
    int cbias_sats;
    int cbias_valid_signals;
    int zero_cbias_valid_signals;
    int ready_sats;
    int paired_sats;
    int iod_match_sats;
    int main_update_sats;
    int raw_pending_updates;
    int index0_unused;
    int gate_unicore;
    int gate_sino;
    int gate_rtcm3;
    int gate_oem4;
    long bridge_calls;
    long bridge_transferred;
    int observed_eph[OBSERVED_BDS3_COUNT];
    int observed_b2b[OBSERVED_BDS3_COUNT];
    int observed_iod_match[OBSERVED_BDS3_COUNT];
} server_product_stats_t;


/* RTKLIB leaves these application callbacks to each executable entry point. */
extern int showmsg(const char *format, ...)
{
    (void)format;
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


extern int rtksvr_test_b2b_ret20_gate(int format, int *raw_update,
                                      int *nav_update);


static long file_size(const char *path)
{
    struct stat st;
    return !stat(path,&st)?(long)st.st_size:-1L;
}


static long count_trace_lines(const char *path, const char *needle)
{
    FILE *fp;
    char line[8192];
    long count=0;

    if (!(fp=fopen(path,"r"))) return -1;
    while (fgets(line,sizeof(line),fp)) {
        if (strstr(line,needle)) count++;
    }
    fclose(fp);
    return count;
}


static long count_trace_lines2(const char *path, const char *needle1,
                               const char *needle2)
{
    FILE *fp;
    char line[8192];
    long count=0;

    if (!(fp=fopen(path,"r"))) return -1;
    while (fgets(line,sizeof(line),fp)) {
        if (strstr(line,needle1)&&strstr(line,needle2)) count++;
    }
    fclose(fp);
    return count;
}


static void parse_bridge_trace(const char *path, long *calls,
                               long *transferred)
{
    FILE *fp;
    char line[8192],*p;
    int format,index,n;

    *calls=*transferred=0;
    if (!(fp=fopen(path,"r"))) {
        *calls=*transferred=-1;
        return;
    }
    while (fgets(line,sizeof(line),fp)) {
        if (!(p=strstr(line,"update_b2b_ssr:"))||
            sscanf(p,"update_b2b_ssr: index=%d format=%d transferred=%d",
                   &index,&format,&n)!=3) continue;
        if (index==1&&format==STRFMT_SINO) {
            (*calls)++;
            *transferred+=n;
        }
    }
    fclose(fp);
}


static int check_ret20_gate(int format, int expected)
{
    int nav_update=-1,raw_update=-1;
    int events=rtksvr_test_b2b_ret20_gate(format,&raw_update,&nav_update);

    return expected?events==1&&raw_update==0&&nav_update==1:
                    events==0&&raw_update==1&&nav_update==0;
}


static uint16_t get_u2le(const uint8_t *p)
{
    return (uint16_t)p[0]|((uint16_t)p[1]<<8);
}


static uint32_t get_u4le(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|
           ((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}


static int scan_sino_fixture(const char *path, sino_fixture_stats_t *stats)
{
    uint8_t frame[MAXRAWLEN];
    FILE *fp;
    size_t n;
    int payload_len,total,type;

    if (!stats||(fp=fopen(path,"rb"))==NULL) return 0;
    memset(stats,0,sizeof(*stats));
    for (;;) {
        n=fread(frame,1,SINO_HEADER_LEN,fp);
        if (n==0&&feof(fp)) break;
        if (n!=SINO_HEADER_LEN||frame[0]!=0xAA||frame[1]!=0x44||
            frame[2]!=0x12||frame[3]!=SINO_HEADER_LEN) {
            stats->length_errors++;
            break;
        }
        payload_len=(int)get_u2le(frame+8);
        total=SINO_HEADER_LEN+payload_len+4;
        if (total>MAXRAWLEN||
            fread(frame+SINO_HEADER_LEN,1,(size_t)payload_len+4,fp)!=
            (size_t)payload_len+4) {
            stats->length_errors++;
            break;
        }
        if (rtk_crc32(frame,total-4)!=get_u4le(frame+total-4)) {
            stats->crc_errors++;
        }
        stats->frames++;
        if (get_u2le(frame+4)==SINO_MSG_BD3EPH) stats->message72++;
        if (get_u2le(frame+4)!=SINO_MSG_B2B) continue;
        stats->message1697++;
        if (SINO_TYPE_POS+6>total*8) {
            stats->length_errors++;
            continue;
        }
        type=(int)getbitu(frame,SINO_TYPE_POS,6);
        if      (type==1) stats->type1++;
        else if (type==2) stats->type2++;
        else if (type==3) stats->type3++;
        else if (type==4) stats->type4++;
        else if (type==63) stats->type63++;
    }
    fclose(fp);
    return 1;
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


static void collect_server_products(const rtksvr_t *svr,
                                    server_product_stats_t *stats)
{
    B2bssr_t zero={0};
    gtime_t time;
    int i,code,sat;

    memset(stats,0,sizeof(*stats));
    stats->sino_format=svr->format[1];
    stats->publish_events=svr->nmsg[1][8];
    stats->index0_unused=
        memcmp(svr->nav.B2bssr,&zero,sizeof(zero))==0&&
        memcmp(svr->raw[1].nav.B2bssr,&zero,sizeof(zero))==0;
    time=svr->raw[1].time;

    for (sat=1;sat<=MAXSAT;sat++) {
        const B2bssr_t *b2b=svr->nav.B2bssr+sat;
        const eph_t *eph=find_current_eph(&svr->nav,sat);
        int has_cbias=0,has_clock=0,has_orbit=0;

        if (eph) stats->eph_sats++;
        has_orbit=b2b->t0[0].time&&isfinite(b2b->deph[0])&&
                  isfinite(b2b->deph[1])&&isfinite(b2b->deph[2]);
        has_clock=b2b->t0[2].time&&isfinite(b2b->dclk[0]);
        if (has_orbit) stats->orbit_sats++;
        if (has_clock) stats->clock_sats++;
        for (code=1;code<=MAXCODE;code++) {
            if (!b2b->cbias_valid[code]||!isfinite(b2b->cbias[code])) continue;
            has_cbias=1;
            stats->cbias_valid_signals++;
            if (b2b->cbias[code]==0.0) stats->zero_cbias_valid_signals++;
        }
        if (has_cbias) stats->cbias_sats++;
        if (has_orbit&&has_clock) {
            stats->b2b_sats++;
            if (eph) {
                stats->paired_sats++;
                if (b2b->iodn==eph->iodc) stats->iod_match_sats++;
            }
        }
        if (time.time&&b2b_orbit_clock_ready(time,b2b,NULL,NULL)) {
            stats->ready_sats++;
        }
        if (b2b->update) stats->main_update_sats++;
        if (svr->raw[1].nav.B2bssr[sat].update) {
            stats->raw_pending_updates++;
        }
    }
    for (i=0;i<OBSERVED_BDS3_COUNT;i++) {
        const B2bssr_t *b2b;
        const eph_t *eph;

        sat=satno(SYS_CMP,observed_bds3_prns[i]);
        if (sat<=0||sat>MAXSAT) continue;
        b2b=svr->nav.B2bssr+sat;
        eph=find_current_eph(&svr->nav,sat);
        stats->observed_eph[i]=eph!=NULL;
        stats->observed_b2b[i]=b2b->t0[0].time&&b2b->t0[2].time;
        stats->observed_iod_match[i]=eph&&stats->observed_b2b[i]&&
                                     b2b->iodn==eph->iodc;
    }
}


static uint32_t sum_rtcm3_messages(const rtcm_t *rtcm)
{
    uint32_t total=0;
    int i;

    for (i=0;i<400;i++) total+=rtcm->nmsg3[i];
    return total;
}


static int write_summary(const char *path, int completed, double speed,
                         long obs_expected, long sino_expected,
                         uint32_t obs_read, uint32_t sino_read,
                         uint32_t rtcm_frames, uint32_t obs_epochs,
                         long server_threads, long strreads,
                         long decoderaw_obs, long decoderaw_sino,
                         long updates, long sino_ret2, long sino_ret20,
                         long rtkpos_calls, long writesol_calls, long pos_bytes,
                         long trace_bytes, const sino_fixture_stats_t *fixture,
                         const server_product_stats_t *products)
{
    FILE *fp=fopen(path,"w");
    int i,pass;

    if (!fp) return 0;
    pass=completed&&obs_read==(uint32_t)obs_expected&&
         sino_read==(uint32_t)sino_expected&&rtcm_frames>0&&obs_epochs>0&&
         server_threads>0&&strreads>0&&decoderaw_obs>0&&decoderaw_sino>0&&
         updates>0&&sino_ret2==580&&sino_ret20>0&&rtkpos_calls>0&&
         writesol_calls>0&&pos_bytes>0&&trace_bytes>0&&
         fixture->frames==2772&&fixture->message72==580&&
         fixture->message1697==2192&&fixture->type1==9&&fixture->type2==40&&
         fixture->type3==41&&fixture->type4==221&&fixture->type63==1881&&
         fixture->crc_errors==0&&fixture->length_errors==0&&
         products->sino_format==STRFMT_SINO&&products->publish_events>0&&
         products->orbit_sats>0&&products->clock_sats>0&&
         products->cbias_sats>0&&products->zero_cbias_valid_signals>0&&
         products->paired_sats>0&&products->iod_match_sats>0&&
         products->main_update_sats>0&&products->raw_pending_updates==0&&
         products->index0_unused&&products->gate_unicore&&
         products->gate_sino&&products->gate_rtcm3&&products->gate_oem4&&
         products->bridge_calls==sino_ret20&&products->bridge_calls>0&&
         products->bridge_transferred>0;
    fprintf(fp,
        "{\n"
        "  \"status\": \"%s\",\n"
        "  \"replay_speed\": %.3f,\n"
        "  \"completed_input\": %s,\n"
        "  \"stream_bytes\": {\"obs_read\": %u, \"obs_expected\": %ld, "
        "\"sino_read\": %u, \"sino_expected\": %ld},\n"
        "  \"rtcm_decoder_frames\": %u,\n"
        "  \"observation_epochs_published\": %u,\n"
        "  \"trace_calls\": {\"rtksvrthread\": %ld, \"strread\": %ld, "
        "\"decoderaw_obs\": %ld, \"decoderaw_sino\": %ld, "
        "\"update_svr\": %ld, \"sino_ret2_update_svr\": %ld, "
        "\"sino_ret20_update_svr\": %ld, "
        "\"rtkpos\": %ld, \"writesol\": %ld},\n"
        "  \"sino_fixture\": {\"frames\": %ld, \"message72\": %ld, "
        "\"message1697\": %ld, \"type1\": %ld, \"type2\": %ld, "
        "\"type3\": %ld, \"type4\": %ld, \"type63\": %ld, "
        "\"crc_errors\": %ld, \"length_errors\": %ld},\n"
        "  \"server_products\": {\"sino_format\": %d, "
        "\"publish_events\": %u, \"eph_sats\": %d, \"b2b_sats\": %d, "
        "\"orbit_sats\": %d, \"clock_sats\": %d, \"cbias_sats\": %d, "
        "\"cbias_valid_signals\": %d, \"zero_cbias_valid_signals\": %d, "
        "\"ready_sats\": %d, \"paired_sats\": %d, "
        "\"iod_match_sats\": %d, \"main_update_sats\": %d, "
        "\"raw_pending_updates\": %d, \"index0_unused\": %s},\n"
        "  \"ret20_gate\": {\"unicore_allowed\": %s, "
        "\"sino_allowed\": %s, \"rtcm3_rejected\": %s, "
        "\"oem4_rejected\": %s},\n"
        "  \"b2b_bridge\": {\"calls\": %ld, \"transferred\": %ld},\n"
        "  \"observed_bds3\": {",
        pass?"PASS":"FAIL",speed,completed?"true":"false",
        obs_read,obs_expected,sino_read,sino_expected,rtcm_frames,obs_epochs,
        server_threads,strreads,decoderaw_obs,decoderaw_sino,updates,sino_ret2,
        sino_ret20,rtkpos_calls,writesol_calls,
        fixture->frames,fixture->message72,fixture->message1697,
        fixture->type1,fixture->type2,fixture->type3,fixture->type4,
        fixture->type63,fixture->crc_errors,fixture->length_errors,
        products->sino_format,products->publish_events,products->eph_sats,
        products->b2b_sats,products->orbit_sats,products->clock_sats,
        products->cbias_sats,products->cbias_valid_signals,
        products->zero_cbias_valid_signals,products->ready_sats,
        products->paired_sats,products->iod_match_sats,
        products->main_update_sats,products->raw_pending_updates,
        products->index0_unused?"true":"false",
        products->gate_unicore?"true":"false",
        products->gate_sino?"true":"false",
        products->gate_rtcm3?"true":"false",
        products->gate_oem4?"true":"false",products->bridge_calls,
        products->bridge_transferred);
    for (i=0;i<OBSERVED_BDS3_COUNT;i++) {
        fprintf(fp,"%s\"C%02d\": {\"eph\": %s, \"b2b\": %s, "
                   "\"iod_match\": %s}",i?", ":"",observed_bds3_prns[i],
                products->observed_eph[i]?"true":"false",
                products->observed_b2b[i]?"true":"false",
                products->observed_iod_match[i]?"true":"false");
    }
    fprintf(fp,"},\n  \"outputs\": {\"pos_bytes\": %ld, "
               "\"trace_bytes\": %ld}\n}\n",pos_bytes,trace_bytes);
    fclose(fp);
    return pass;
}


int main(int argc, char **argv)
{
    rtksvr_t *svr=NULL;
    sino_fixture_stats_t fixture;
    server_product_stats_t products;
    prcopt_t prcopt=prcopt_default;
    solopt_t solopt[2]={solopt_default,solopt_default};
    int streams[8]={STR_FILE,STR_FILE,STR_NONE,STR_FILE,STR_NONE,
                    STR_NONE,STR_NONE,STR_NONE};
    int formats[3]={STRFMT_RTCM3,STRFMT_SINO,STRFMT_RTCM3};
    char obs_path[MAXSTRPATH],sino_path[MAXSTRPATH],errmsg[1024]="";
    char *paths[8],*cmds[3]={NULL,NULL,NULL};
    char *periodic[3]={NULL,NULL,NULL};
    char empty[]="",ephall[]="-EPHALL";
    char *rcvopts[3]={empty,ephall,empty};
    double nmeapos[3]={0.0,0.0,0.0},speed,timeout_seconds;
    uint32_t obs_read=0,sino_read=0,rtcm_frames=0,obs_epochs=0;
    long obs_expected,sino_expected,deadline,server_threads,strreads;
    long decoderaw_obs,decoderaw_sino,updates,sino_ret2,sino_ret20,rtkpos_calls;
    long writesol_calls,pos_bytes,trace_bytes;
    int completed=0,pass=0;

    if (argc!=8) {
        fprintf(stderr,
            "usage: %s OBS SINO POS TRACE SUMMARY SPEED TIMEOUT_SECONDS\n",
            argv[0]);
        return 2;
    }
    speed=atof(argv[6]);
    timeout_seconds=atof(argv[7]);
    if (speed<=0.0||timeout_seconds<=0.0) {
        fprintf(stderr,"invalid replay speed or timeout\n");
        return 2;
    }
    obs_expected=file_size(argv[1]);
    sino_expected=file_size(argv[2]);
    if (obs_expected<=0||sino_expected<=0||
        !scan_sino_fixture(argv[2],&fixture)) {
        fprintf(stderr,"input file stat failed: %s\n",strerror(errno));
        return 2;
    }
    snprintf(obs_path,sizeof(obs_path),"%s::T::x%.6f::P=4",argv[1],speed);
    snprintf(sino_path,sizeof(sino_path),"%s::T::x%.6f::P=4",argv[2],speed);
    paths[0]=obs_path; paths[1]=sino_path; paths[2]=empty; paths[3]=argv[3];
    paths[4]=empty; paths[5]=empty; paths[6]=empty; paths[7]=empty;

    prcopt.mode=PMODE_SINGLE;
    prcopt.navsys=SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP|SYS_IRN;
    solopt[0].outhead=1;

    traceopen(argv[4]);
    tracelevel(4);
    if (!(svr=(rtksvr_t *)calloc(1,sizeof(*svr)))) {
        fprintf(stderr,"rtksvr allocation failed\n");
        traceclose();
        return 2;
    }
    if (!rtksvrinit(svr)) {
        fprintf(stderr,"rtksvrinit failed\n");
        free(svr);
        traceclose();
        return 2;
    }
    if (!rtksvrstart(svr,10,65536,streams,paths,formats,0,cmds,periodic,
                     rcvopts,0,0,nmeapos,&prcopt,solopt,NULL,errmsg)) {
        fprintf(stderr,"rtksvrstart failed: %s\n",errmsg);
        rtksvrfree(svr);
        free(svr);
        traceclose();
        return 2;
    }

    deadline=(long)(timeout_seconds*1000.0);
    while (deadline>0) {
        sleepms(20);
        deadline-=20;
        rtksvrlock(svr);
        obs_read=svr->stream[0].inb;
        sino_read=svr->stream[1].inb;
        completed=obs_read==(uint32_t)obs_expected&&
                  sino_read==(uint32_t)sino_expected&&
                  svr->nb[0]==0&&svr->nb[1]==0;
        rtksvrunlock(svr);
        if (completed) break;
    }
    if (completed) sleepms(250);

    rtksvrlock(svr);
    obs_read=svr->stream[0].inb;
    sino_read=svr->stream[1].inb;
    rtcm_frames=sum_rtcm3_messages(&svr->rtcm[0]);
    obs_epochs=svr->nmsg[0][0];
    collect_server_products(svr,&products);
    products.gate_unicore=check_ret20_gate(STRFMT_UNICORE,1);
    products.gate_sino=check_ret20_gate(STRFMT_SINO,1);
    products.gate_rtcm3=check_ret20_gate(STRFMT_RTCM3,0);
    products.gate_oem4=check_ret20_gate(STRFMT_OEM4,0);
    rtksvrunlock(svr);
    rtksvrstop(svr,cmds);
    rtksvrfree(svr);
    free(svr);
    traceclose();

    server_threads=count_trace_lines(argv[4],"rtksvrthread:");
    strreads=count_trace_lines(argv[4],"strread:");
    decoderaw_obs=count_trace_lines(argv[4],"decoderaw: index=0");
    decoderaw_sino=count_trace_lines(argv[4],"decoderaw: index=1");
    updates=count_trace_lines(argv[4],"updatesvr:");
    sino_ret2=count_trace_lines2(argv[4],"updatesvr: ret=2 ","index=1");
    sino_ret20=count_trace_lines2(argv[4],"updatesvr: ret=20 ","index=1");
    rtkpos_calls=count_trace_lines(argv[4],"rtkpos  :");
    writesol_calls=count_trace_lines(argv[4],"writesol:");
    pos_bytes=file_size(argv[3]);
    trace_bytes=file_size(argv[4]);
    parse_bridge_trace(argv[4],&products.bridge_calls,
                       &products.bridge_transferred);
    pass=write_summary(argv[5],completed,speed,obs_expected,sino_expected,
                       obs_read,sino_read,rtcm_frames,obs_epochs,server_threads,
                       strreads,decoderaw_obs,decoderaw_sino,updates,sino_ret2,
                       sino_ret20,rtkpos_calls,writesol_calls,pos_bytes,
                       trace_bytes,&fixture,&products);
    printf("test_rtksvr_file_replay: %s\n",pass?"PASS":"FAIL");
    printf("fixture: frames=%ld message72=%ld message1697=%ld "
           "type1=%ld type2=%ld type3=%ld type4=%ld type63=%ld\n",
           fixture.frames,fixture.message72,fixture.message1697,fixture.type1,
           fixture.type2,fixture.type3,fixture.type4,fixture.type63);
    printf("server: sino_format=%d ret2=%ld ret20=%ld publish_events=%u "
           "eph=%d b2b=%d orbit=%d clock=%d cbias_sats=%d "
           "cbias_valid=%d zero_valid=%d ready=%d paired=%d iod_match=%d "
           "main_update=%d raw_pending=%d index0_unused=%d\n",
           products.sino_format,sino_ret2,sino_ret20,products.publish_events,
           products.eph_sats,products.b2b_sats,products.orbit_sats,
           products.clock_sats,products.cbias_sats,
           products.cbias_valid_signals,products.zero_cbias_valid_signals,
           products.ready_sats,products.paired_sats,products.iod_match_sats,
           products.main_update_sats,products.raw_pending_updates,
           products.index0_unused);
    printf("ret20_gate: unicore=%d sino=%d rtcm3_rejected=%d "
           "oem4_rejected=%d bridge_calls=%ld bridge_transferred=%ld\n",
           products.gate_unicore,products.gate_sino,products.gate_rtcm3,
           products.gate_oem4,products.bridge_calls,
           products.bridge_transferred);
    printf("observed_bds3:");
    for (int i=0;i<OBSERVED_BDS3_COUNT;i++) {
        printf(" C%02d[eph=%d,b2b=%d,iod=%d]",observed_bds3_prns[i],
               products.observed_eph[i],products.observed_b2b[i],
               products.observed_iod_match[i]);
    }
    printf("\n");
    return pass?0:1;
}
