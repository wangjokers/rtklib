#include "rtklib.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int showmsg(const char *format, ...)
{
    va_list arg;

    va_start(arg,format);
    vfprintf(stderr,format,arg);
    va_end(arg);
    return 0;
}

static int target_prn(int prn)
{
    return prn==23||prn==25||prn==32;
}

int main(int argc, char **argv)
{
    const char *opt="-SYS=C -CL2I -CL7I";
    const double ep[]={2024,12,25,0,30,0};
    obs_t *obs;
    nav_t *nav;
    sta_t *sta;
    gtime_t time=epoch2time(ep);
    int i,j,prn,records=0,ok=1;
    char id[16];

    if (argc!=2) {
        fprintf(stderr,"Usage: %s <obs_rnx>\n",argv[0]);
        return 1;
    }
    printf("INDEX 2I=%d 7I=%d PRIORITY 2I=%d 7I=%d\n",
           code2idx(SYS_CMP,CODE_L2I),code2idx(SYS_CMP,CODE_L7I),
           getcodepri(SYS_CMP,CODE_L2I,opt),
           getcodepri(SYS_CMP,CODE_L7I,opt));

    if (!(obs=(obs_t *)calloc(1,sizeof(*obs)))||
        !(nav=(nav_t *)calloc(1,sizeof(*nav)))||
        !(sta=(sta_t *)calloc(1,sizeof(*sta)))) {
        fprintf(stderr,"allocation failed\n");
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }
    if (readrnxt(argv[1],1,time,time,0.0,opt,obs,nav,sta)<0) {
        fprintf(stderr,"readrnxt failed\n");
        ok=0;
    }
    for (i=0;i<obs->n;i++) {
        if (satsys(obs->data[i].sat,&prn)!=SYS_CMP||!target_prn(prn)) continue;
        satno2id(obs->data[i].sat,id);
        printf("OBS %s",id);
        for (j=0;j<NFREQ;j++) {
            printf(" f%d=%s(%u):P=%.3f:L=%.3f",j+1,
                   code2obs(obs->data[i].code[j]),
                   (unsigned int)obs->data[i].code[j],obs->data[i].P[j],
                   obs->data[i].L[j]);
        }
        printf("\n");
        records++;
        if (obs->data[i].code[0]!=CODE_L2I||
            obs->data[i].code[1]!=CODE_L7I||
            obs->data[i].P[0]==0.0||obs->data[i].P[1]==0.0||
            obs->data[i].L[0]==0.0||obs->data[i].L[1]==0.0) ok=0;
    }
    printf("BDS_DUAL_FREQ_SLOTS %d records=%d\n",ok&&records==3,records);
    freeobs(obs);
    freenav(nav,0x7F);
    free(obs);
    free(nav);
    free(sta);
    return ok&&records==3?0:1;
}
