#include "rtklib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_EPS 1E-12
#define TEST_ORBIT_VALIDITY 126.0
#define TEST_CLOCK_VALIDITY 42.0

static int close_value(double actual, double expected, double tolerance)
{
    return fabs(actual-expected)<=tolerance;
}

static gtime_t test_time(void)
{
    return gpst2time(2300,100000.0);
}

static int test_age_boundaries(void)
{
    B2bssr_t b2b={0};
    gtime_t t0=test_time();
    double age;
    int ok=1;

    b2b.t0[0]=b2b.t0[1]=b2b.t0[2]=t0;
    b2b.udi[0]=b2b.udi[1]=b2b.udi[2]=999999.0;

    ok&=b2b_orbit_age_valid(timeadd(t0,-DTTOL),&b2b,&age);
    ok&=close_value(age,-DTTOL,TEST_EPS);
    ok&=!b2b_orbit_age_valid(timeadd(t0,-DTTOL-0.001),&b2b,&age);
    ok&=b2b_orbit_age_valid(timeadd(t0,96.0+DTTOL+0.001),&b2b,&age);
    ok&=b2b_orbit_age_valid(timeadd(t0,TEST_ORBIT_VALIDITY+DTTOL),
                            &b2b,&age);
    ok&=!b2b_orbit_age_valid(timeadd(t0,TEST_ORBIT_VALIDITY+DTTOL+0.001),
                             &b2b,&age);

    ok&=b2b_cbias_age_valid(timeadd(t0,86400.0+DTTOL),&b2b,&age);
    ok&=!b2b_cbias_age_valid(timeadd(t0,86400.0+DTTOL+0.001),
                             &b2b,&age);
    ok&=b2b_clock_age_valid(timeadd(t0,12.0+DTTOL+0.001),&b2b,&age);
    ok&=b2b_clock_age_valid(timeadd(t0,TEST_CLOCK_VALIDITY+DTTOL),
                            &b2b,&age);
    ok&=!b2b_clock_age_valid(timeadd(t0,TEST_CLOCK_VALIDITY+DTTOL+0.001),
                             &b2b,&age);

    b2b.t0[0].time=0;
    ok&=!b2b_orbit_age_valid(t0,&b2b,&age);
    ok&=!b2b_orbit_age_valid(t0,NULL,&age);
    return ok;
}

static void init_ready_b2b(B2bssr_t *b2b)
{
    gtime_t t0=test_time();

    memset(b2b,0,sizeof(*b2b));
    b2b->t0[0]=t0;
    b2b->t0[2]=t0;
    b2b->iodssr[0]=4;
    b2b->iodssr[2]=4;
    b2b->iodcorr[0]=7;
    b2b->iodcorr[1]=7;
    b2b->deph[0]=6.0;
    b2b->deph[1]=8.0;
    b2b->dclk[0]=1E-6*CLIGHT;
    b2b->udi[0]=999999.0;
    b2b->udi[2]=999999.0;
    b2b->update=0;
}

static int test_orbit_clock_readiness(void)
{
    B2bssr_t b2b;
    gtime_t time=test_time();
    double orbit_age,clock_age;
    int ok=1;

    init_ready_b2b(&b2b);
    ok&=b2b_orbit_clock_ready(time,&b2b,&orbit_age,&clock_age);
    ok&=orbit_age==0.0&&clock_age==0.0;

    b2b.iodssr[2]++;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);
    init_ready_b2b(&b2b);
    b2b.iodcorr[1]++;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);

    init_ready_b2b(&b2b);
    b2b.t0[0].time=0;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);
    init_ready_b2b(&b2b);
    b2b.t0[2].time=0;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);

    init_ready_b2b(&b2b);
    b2b.deph[0]=NAN;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);
    init_ready_b2b(&b2b);
    b2b.dclk[0]=INFINITY;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);
    init_ready_b2b(&b2b);
    b2b.dclk[1]=NAN;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);

    init_ready_b2b(&b2b);
    b2b.deph[1]=8.001;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);
    init_ready_b2b(&b2b);
    b2b.dclk[0]=1E-6*CLIGHT+0.001;
    ok&=!b2b_orbit_clock_ready(time,&b2b,NULL,NULL);

    init_ready_b2b(&b2b);
    ok&=!b2b_orbit_clock_ready(timeadd(time,TEST_CLOCK_VALIDITY+DTTOL+0.001),
                               &b2b,NULL,NULL);
    ok&=!b2b_orbit_clock_ready(time,NULL,NULL,NULL);
    return ok;
}

static int test_urai(void)
{
    static const int urai[]={0,1,8,16,24,32,40,48,56,62};
    static const double sigma[]={
        0.0005,0.00025,0.002,0.008,0.026,
        0.080,0.242,0.728,2.186,5.4665
    };
    double variance;
    int i,ok=1;

    for (i=0;i<(int)(sizeof(urai)/sizeof(urai[0]));i++) {
        ok&=b2b_urai_variance(urai[i],&variance);
        ok&=close_value(variance,sigma[i]*sigma[i],1E-10);
    }
    variance=123.0;
    ok&=!b2b_urai_variance(63,&variance)&&variance==0.0;
    ok&=!b2b_urai_variance(-1,&variance);
    ok&=!b2b_urai_variance(64,&variance);
    ok&=!b2b_urai_variance(1,NULL);
    return ok;
}

static int test_rac(void)
{
    const double r1[]={7000.0,0.0,0.0};
    const double v1[]={0.0,4.0,0.0};
    const double rac1[]={1.0,-2.0,3.0};
    const double r2[]={0.0,7000.0,0.0};
    const double v2[]={-4.0,0.0,0.0};
    const double rac2[]={1.0,2.0,3.0};
    const double zero[]={0.0,0.0,0.0};
    const double parallel[]={2.0,0.0,0.0};
    double invalid_r[]={NAN,0.0,0.0};
    double ecef[3];
    int ok=1;

    ok&=b2b_rac_to_ecef(r1,v1,rac1,ecef);
    ok&=close_value(ecef[0],1.0,TEST_EPS);
    ok&=close_value(ecef[1],-2.0,TEST_EPS);
    ok&=close_value(ecef[2],3.0,TEST_EPS);

    ok&=b2b_rac_to_ecef(r2,v2,rac2,ecef);
    ok&=close_value(ecef[0],-2.0,TEST_EPS);
    ok&=close_value(ecef[1],1.0,TEST_EPS);
    ok&=close_value(ecef[2],3.0,TEST_EPS);

    ok&=!b2b_rac_to_ecef(r1,zero,rac1,ecef);
    ok&=!b2b_rac_to_ecef(r1,parallel,rac1,ecef);
    ok&=!b2b_rac_to_ecef(invalid_r,v1,rac1,ecef);
    ok&=!b2b_rac_to_ecef(r1,v1,rac1,NULL);
    return ok;
}

static int test_clock_sign(void)
{
    double corrected;
    int ok=1;

    ok&=b2b_clock_correct(10E-6,CLIGHT*1E-6,&corrected);
    ok&=close_value(corrected,9E-6,TEST_EPS);
    ok&=b2b_clock_correct(10E-6,-CLIGHT*1E-6,&corrected);
    ok&=close_value(corrected,11E-6,TEST_EPS);
    ok&=!b2b_clock_correct(NAN,0.0,&corrected);
    ok&=!b2b_clock_correct(0.0,INFINITY,&corrected);
    ok&=!b2b_clock_correct(0.0,0.0,NULL);
    return ok;
}

static int test_update_clear(void)
{
    nav_t *nav=(nav_t *)calloc(1,sizeof(*nav));
    B2bssr_t before0,before1,before_max;
    int ok=1;

    if (!nav) return 0;

    nav->B2bssr[0].update=1;
    nav->B2bssr[0].iodn=100;
    nav->B2bssr[1].update=1;
    nav->B2bssr[1].t0[0]=test_time();
    nav->B2bssr[1].iodssr[0]=3;
    nav->B2bssr[1].iodcorr[0]=4;
    nav->B2bssr[1].deph[0]=1.25;
    nav->B2bssr[1].dclk[0]=-0.5;
    nav->B2bssr[1].cbias[1]=0.75f;
    nav->B2bssr[MAXSAT].update=1;
    nav->B2bssr[MAXSAT].t0[2]=test_time();
    nav->B2bssr[MAXSAT].iodssr[2]=5;
    nav->B2bssr[MAXSAT].iodcorr[1]=6;
    nav->B2bssr[MAXSAT].dclk[0]=2.5;

    before0=nav->B2bssr[0];
    before1=nav->B2bssr[1];
    before_max=nav->B2bssr[MAXSAT];
    before1.update=0;
    before_max.update=0;

    ok&=b2b_clear_nav_updates(nav)==2;
    ok&=!memcmp(nav->B2bssr,&before0,sizeof(before0));
    ok&=!memcmp(nav->B2bssr+1,&before1,sizeof(before1));
    ok&=!memcmp(nav->B2bssr+MAXSAT,&before_max,sizeof(before_max));
    ok&=b2b_clear_nav_updates(nav)==0;
    ok&=b2b_clear_nav_updates(NULL)==0;

    free(nav);
    return ok;
}

int main(void)
{
    int age=test_age_boundaries();
    int readiness=test_orbit_clock_readiness();
    int urai=test_urai();
    int rac=test_rac();
    int clock=test_clock_sign();
    int update=test_update_clear();
    int all=age&&readiness&&urai&&rac&&clock&&update;

    printf("AGE_BOUNDARIES %d\n",age);
    printf("ORBIT_CLOCK_READINESS %d\n",readiness);
    printf("URAI_VARIANCE %d\n",urai);
    printf("RAC_TO_ECEF %d\n",rac);
    printf("CLOCK_SIGN %d\n",clock);
    printf("UPDATE_CLEAR %d\n",update);
    printf("ALL_HELPERS %d\n",all);
    return all?0:1;
}
