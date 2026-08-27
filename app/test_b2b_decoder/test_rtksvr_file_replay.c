/* Phase 1: drive the unmodified RTKLIB realtime server with two STR_FILE inputs. */
#include "rtklib.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>


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
                         long updates, long sino_ret20, long rtkpos_calls,
                         long writesol_calls, long pos_bytes, long trace_bytes)
{
    FILE *fp=fopen(path,"w");
    int pass;

    if (!fp) return 0;
    pass=completed&&obs_read==(uint32_t)obs_expected&&
         sino_read==(uint32_t)sino_expected&&rtcm_frames>0&&obs_epochs>0&&
         server_threads>0&&strreads>0&&decoderaw_obs>0&&decoderaw_sino>0&&
         updates>0&&sino_ret20>0&&rtkpos_calls>0&&writesol_calls>0&&
         pos_bytes>0&&trace_bytes>0;
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
        "\"update_svr\": %ld, \"sino_ret20_update_svr\": %ld, "
        "\"rtkpos\": %ld, \"writesol\": %ld},\n"
        "  \"outputs\": {\"pos_bytes\": %ld, \"trace_bytes\": %ld}\n"
        "}\n",
        pass?"PASS":"FAIL",speed,completed?"true":"false",
        obs_read,obs_expected,sino_read,sino_expected,rtcm_frames,obs_epochs,
        server_threads,strreads,decoderaw_obs,decoderaw_sino,updates,sino_ret20,
        rtkpos_calls,writesol_calls,pos_bytes,trace_bytes);
    fclose(fp);
    return pass;
}


int main(int argc, char **argv)
{
    rtksvr_t *svr=NULL;
    prcopt_t prcopt=prcopt_default;
    solopt_t solopt[2]={solopt_default,solopt_default};
    int streams[8]={STR_FILE,STR_FILE,STR_NONE,STR_FILE,STR_NONE,
                    STR_NONE,STR_NONE,STR_NONE};
    int formats[3]={STRFMT_RTCM3,STRFMT_SINO,STRFMT_RTCM3};
    char obs_path[MAXSTRPATH],sino_path[MAXSTRPATH],errmsg[1024]="";
    char *paths[8],*cmds[3]={NULL,NULL,NULL};
    char *periodic[3]={NULL,NULL,NULL};
    char empty[]="";
    char *rcvopts[3]={empty,empty,empty};
    double nmeapos[3]={0.0,0.0,0.0},speed,timeout_seconds;
    uint32_t obs_read=0,sino_read=0,rtcm_frames=0,obs_epochs=0;
    long obs_expected,sino_expected,deadline,server_threads,strreads;
    long decoderaw_obs,decoderaw_sino,updates,sino_ret20,rtkpos_calls;
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
    if (obs_expected<=0||sino_expected<=0) {
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
    sino_ret20=count_trace_lines(argv[4],"updatesvr: ret=20");
    rtkpos_calls=count_trace_lines(argv[4],"rtkpos  :");
    writesol_calls=count_trace_lines(argv[4],"writesol:");
    pos_bytes=file_size(argv[3]);
    trace_bytes=file_size(argv[4]);
    pass=write_summary(argv[5],completed,speed,obs_expected,sino_expected,
                       obs_read,sino_read,rtcm_frames,obs_epochs,server_threads,
                       strreads,decoderaw_obs,decoderaw_sino,updates,sino_ret20,
                       rtkpos_calls,writesol_calls,pos_bytes,trace_bytes);
    printf("test_rtksvr_file_replay: %s\n",pass?"PASS":"FAIL");
    return pass?0:1;
}
