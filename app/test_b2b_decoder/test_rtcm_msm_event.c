/* Verify that an unsupported MSM frame cannot replay a completed obs epoch. */
#include "../../src/rtklib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NMSM_TYPES 7

typedef struct {
    int type;
    long frames;
    long sync0;
    long sync1;
    long ret1;
} msm_stat_t;

static msm_stat_t msm_stats[NMSM_TYPES]={
    {1077,0,0,0,0},
    {1087,0,0,0,0},
    {1097,0,0,0,0},
    {1107,0,0,0,0},
    {1117,0,0,0,0},
    {1127,0,0,0,0},
    {1137,0,0,0,0}
};

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

static msm_stat_t *find_stat(int type)
{
    int i;

    for (i=0;i<NMSM_TYPES;i++) {
        if (msm_stats[i].type==type) return msm_stats+i;
    }
    return NULL;
}

static int check_supported_msm(void)
{
    int i;

    for (i=0;i<5;i++) {
        if (msm_stats[i].frames!=600||msm_stats[i].sync1!=600||
            msm_stats[i].sync0!=0||msm_stats[i].ret1!=0) return 0;
    }
    return msm_stats[5].frames==1200&&msm_stats[5].sync1==600&&
           msm_stats[5].sync0==600&&msm_stats[5].ret1==600;
}

int main(int argc, char **argv)
{
    rtcm_t *rtcm;
    msm_stat_t *stat;
    FILE *fp;
    gtime_t last_event={0};
    long frames=0,decode_errors=0,obs_events=0,unique_events=0;
    long duplicate_events=0,empty_events=0,stale_1137=0;
    int data,before,ret,type,sync,pass,i;

    if (argc!=2) {
        fprintf(stderr,"usage: %s OBS_RTCM3\n",argv[0]);
        return 2;
    }
    if (!(rtcm=(rtcm_t *)calloc(1,sizeof(*rtcm)))) return 2;
    if (!init_rtcm(rtcm)||!(fp=fopen(argv[1],"rb"))) {
        free_rtcm(rtcm);
        free(rtcm);
        return 2;
    }
    while ((data=fgetc(fp))!=EOF) {
        before=rtcm->nbyte;
        ret=input_rtcm3(rtcm,(uint8_t)data);
        if (before<=0||rtcm->nbyte!=0) continue;

        frames++;
        if (ret<0) decode_errors++;
        type=(int)getbitu(rtcm->buff,24,12);
        if (!(stat=find_stat(type))) continue;

        sync=(int)getbitu(rtcm->buff,24+12+12+30,1);
        stat->frames++;
        if (sync) stat->sync1++; else stat->sync0++;
        if (ret==1) stat->ret1++;

        if (type==1137&&!sync&&rtcm->obs.n>0) stale_1137++;
        if (ret!=1) continue;

        obs_events++;
        if (rtcm->obs.n<=0) {
            empty_events++;
            continue;
        }
        if (last_event.time&&
            fabs(timediff(rtcm->obs.data[0].time,last_event))<=1E-9) {
            duplicate_events++;
        }
        else {
            unique_events++;
            last_event=rtcm->obs.data[0].time;
        }
    }
    fclose(fp);

    pass=frames==4834&&decode_errors==0&&NSATIRN==0&&
         satno(SYS_IRN,9)==0&&check_supported_msm()&&
         msm_stats[6].frames==600&&msm_stats[6].sync0==600&&
         msm_stats[6].sync1==0&&msm_stats[6].ret1==0&&
         obs_events==600&&unique_events==600&&duplicate_events==0&&
         empty_events==0&&stale_1137==0;

    printf("test_rtcm_msm_event: %s frames=%ld errors=%ld events=%ld "
           "unique=%ld duplicate=%ld stale1137=%ld ret1137=%ld\n",
           pass?"PASS":"FAIL",frames,decode_errors,obs_events,unique_events,
           duplicate_events,stale_1137,msm_stats[6].ret1);
    for (i=0;i<NMSM_TYPES;i++) {
        printf("  type=%d frames=%ld sync1=%ld sync0=%ld ret1=%ld\n",
               msm_stats[i].type,msm_stats[i].frames,msm_stats[i].sync1,
               msm_stats[i].sync0,msm_stats[i].ret1);
    }
    free_rtcm(rtcm);
    free(rtcm);
    return pass?0:1;
}
