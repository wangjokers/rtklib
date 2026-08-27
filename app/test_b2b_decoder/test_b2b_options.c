#include "rtklib.h"

#include <stdio.h>
#include <string.h>

extern void trace(int level, const char *format, ...)
{
    (void)level;
    (void)format;
}

const prcopt_t prcopt_default={0};
const solopt_t solopt_default={0};

extern int satid2no(const char *id)
{
    (void)id;
    return 0;
}

extern void satno2id(int sat, char *id)
{
    (void)sat;
    if (id) strcpy(id,"");
}

extern void ecef2pos(const double *r, double *pos)
{
    (void)r;
    if (pos) pos[0]=pos[1]=pos[2]=0.0;
}

extern void pos2ecef(const double *pos, double *r)
{
    (void)pos;
    if (r) r[0]=r[1]=r[2]=0.0;
}

static int test_b2b_sateph_mapping(void)
{
    opt_t *opt=searchopt("pos1-sateph",sysopts);
    char text[64];
    int ok=1;

    if (!opt) return 0;

    ok&=str2opt(opt,"brdc+b2b-apc");
    ok&=*(int *)opt->var==EPHOPT_B2b;

    memset(text,0,sizeof(text));
    opt2str(opt,text);
    ok&=!strcmp(text,"brdc+b2b-apc");

    ok&=str2opt(opt,"brdc");
    ok&=*(int *)opt->var==EPHOPT_BRDC;

    ok&=str2opt(opt,"brdc+ssrcom");
    ok&=*(int *)opt->var==EPHOPT_SSRCOM;
    return ok;
}

static int test_b2b_xbias_mapping(void)
{
    opt_t *opt=searchopt("pos1-b2bxbias",sysopts);
    char text[64];
    int ok=1;

    if (!opt) return 0;

    ok&=str2opt(opt,"data");
    ok&=*(int *)opt->var==B2BXBIAS_DATA;
    ok&=str2opt(opt,"pilot");
    ok&=*(int *)opt->var==B2BXBIAS_PILOT;
    ok&=str2opt(opt,"mean");
    ok&=*(int *)opt->var==B2BXBIAS_MEAN;

    memset(text,0,sizeof(text));
    opt2str(opt,text);
    ok&=!strcmp(text,"mean");

    ok&=str2opt(opt,"off");
    ok&=*(int *)opt->var==B2BXBIAS_OFF;
    return ok;
}

static int test_config_file_paths(void)
{
    static const char *names[]={
        "file-obsfile","file-navfile","file-sp3file","file-clkfile",
        "file-outfile"
    };
    static const char *values[]={
        "rover_MO.rnx","broadcast_MN.rnx","precise.SP3","precise.CLK",
        "result.pos"
    };
    opt_t *opt;
    int i,ok=1;

    for (i=0;i<5;i++) {
        if (!(opt=searchopt(names[i],sysopts))) return 0;
        ok&=str2opt(opt,values[i]);
        ok&=!strcmp((const char *)opt->var,values[i]);
    }
    return ok;
}

int main(void)
{
    int mapping=test_b2b_sateph_mapping();
    int xbias=test_b2b_xbias_mapping();
    int files=test_config_file_paths();

    printf("B2B_SATEPH_MAPPING %d\n",mapping);
    printf("B2B_XBIAS_MAPPING %d\n",xbias);
    printf("CONFIG_FILE_PATHS %d\n",files);
    printf("ALL_B2B_OPTIONS %d\n",mapping&&xbias&&files);
    return mapping&&xbias&&files?0:1;
}
