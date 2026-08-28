#include "rtklib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern double b2b_test_prange(const obsd_t *obs, const nav_t *nav,
                              const prcopt_t *opt, double *var);

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

typedef struct {
    const char *id;
    double p1;
    double p2;
    double tgd_b1cp;
    double tgd_b2ap;
    double tgd_b2bi;
} realtime_case_t;

static double square(double x)
{
    return x*x;
}

static double if_prange(double p1, double p2, double gamma,
                        double tgd1, double tgd2)
{
    const double b1=tgd1*CLIGHT;
    const double b2=tgd2*CLIGHT;
    return ((p2-gamma*p1)-(b2-gamma*b1))/(1.0-gamma);
}

static int close_enough(double actual, double expected, double tol,
                        const char *what)
{
    if (fabs(actual-expected)<=tol) return 1;
    fprintf(stderr,"FAIL %s: actual=%.12f expected=%.12f diff=%.12g\n",
            what,actual,expected,actual-expected);
    return 0;
}

static void init_obs_nav(obsd_t *obs, nav_t *nav, eph_t *eph, int prn)
{
    double ep[6]={2026,8,26,12,20,30};

    memset(obs,0,sizeof(*obs));
    memset(nav,0,sizeof(*nav));
    memset(eph,0,sizeof(*eph));
    obs->time=epoch2time(ep);
    obs->sat=(uint8_t)satno(SYS_CMP,prn);
    eph->sat=obs->sat;
    eph->toe=obs->time;
    nav->eph=eph;
    nav->n=nav->nmax=1;
}

static int test_realtime_b1cp_b2ap(void)
{
    static const realtime_case_t cases[]={
        {"C23",24650377.922,24650392.070, 2.299202606081963E-8,
         -2.270098775625229E-9,-2.037268131971359E-9},
        {"C31",23929837.438,23929857.906,-1.571606844663620E-9,
         -5.180481821298599E-9,-5.355104804039001E-9},
        {"C41",24352484.922,24352515.180,-2.008164301514626E-8,
          5.820766091346741E-9, 7.275957614183426E-9}
    };
    const double gamma=square(FREQ1/FREQ5);
    const double legacy_gamma=square(FREQ1/FREQ2_CMP);
    prcopt_t opt=prcopt_default;
    obsd_t obs;
    nav_t nav;
    eph_t eph;
    double actual,expected,legacy_freq,legacy_full,var;
    int i,ok=1;

    opt.ionoopt=IONOOPT_IFLC;
    ok&=close_enough(gamma,1.793270321361059,1E-15,"B1C/B2a gamma");

    for (i=0;i<(int)(sizeof(cases)/sizeof(cases[0]));i++) {
        init_obs_nav(&obs,&nav,&eph,atoi(cases[i].id+1));
        obs.code[0]=CODE_L1P;
        obs.code[1]=CODE_L5P;
        obs.P[0]=cases[i].p1;
        obs.P[1]=cases[i].p2;
        eph.code=EPHCODE_BDS_CNV1;
        eph.tgd[1]=cases[i].tgd_b2bi;
        eph.tgd[2]=cases[i].tgd_b1cp;
        eph.tgd[3]=cases[i].tgd_b2ap;

        ok&=close_enough(sat2freq(obs.sat,obs.code[0],&nav),FREQ1,1E-6,
                         "CODE_L1P frequency");
        ok&=close_enough(sat2freq(obs.sat,obs.code[1],&nav),FREQ5,1E-6,
                         "CODE_L5P frequency");
        expected=if_prange(obs.P[0],obs.P[1],gamma,eph.tgd[2],eph.tgd[3]);
        actual=b2b_test_prange(&obs,&nav,&opt,&var);
        legacy_freq=if_prange(obs.P[0],obs.P[1],legacy_gamma,
                              eph.tgd[2],eph.tgd[3]);
        legacy_full=if_prange(obs.P[0],obs.P[1],legacy_gamma,
                              eph.tgd[2],eph.tgd[1]);
        ok&=close_enough(actual,expected,1E-6,cases[i].id);
        printf("%s gamma=%.15f corrected=%.6f frequency_error=%+.6f "
               "legacy_total_error=%+.6f\n",cases[i].id,gamma,actual,
               legacy_freq-expected,legacy_full-expected);
    }
    return ok;
}

static int test_legacy_b1i_b2i(void)
{
    const double gamma=square(FREQ1_CMP/FREQ2_CMP);
    prcopt_t opt=prcopt_default;
    obsd_t obs;
    nav_t nav;
    eph_t eph;
    double actual,expected,var;

    opt.ionoopt=IONOOPT_IFLC;
    init_obs_nav(&obs,&nav,&eph,23);
    obs.code[0]=CODE_L2I;
    obs.code[1]=CODE_L7I;
    obs.P[0]=24650380.0;
    obs.P[1]=24650395.0;
    eph.code=0;
    eph.tgd[0]=1.25E-9;
    eph.tgd[1]=-2.75E-9;

    expected=if_prange(obs.P[0],obs.P[1],gamma,eph.tgd[0],eph.tgd[1]);
    actual=b2b_test_prange(&obs,&nav,&opt,&var);
    printf("B1I+B2I gamma=%.15f corrected=%.6f\n",gamma,actual);
    return close_enough(actual,expected,1E-6,"legacy B1I+B2I");
}

int main(void)
{
    int ok=test_realtime_b1cp_b2ap()&&test_legacy_b1i_b2i();
    printf("test_pntpos_bds_prange: %s\n",ok?"PASS":"FAIL");
    return ok?0:1;
}
