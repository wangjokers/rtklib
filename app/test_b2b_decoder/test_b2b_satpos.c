#include "rtklib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_EPS_POS 1E-6
#define TEST_EPS_CLK 1E-13
#define TEST_MU_CMP  3.986004418E14

extern char *time_str(gtime_t t, int n)
{
    static char buff[32];

    (void)t;
    (void)n;
    strcpy(buff,"test-time");
    return buff;
}

extern void satno2id(int sat, char *id)
{
    int prn=0,sys=satsys(sat,&prn);

    if (!id) return;
    if (sys==SYS_GPS) sprintf(id,"G%02d",prn);
    else if (sys==SYS_GLO) sprintf(id,"R%02d",prn);
    else if (sys==SYS_GAL) sprintf(id,"E%02d",prn);
    else if (sys==SYS_CMP) sprintf(id,"C%02d",prn);
    else sprintf(id,"%02d",sat);
}

extern double dot(const double *a, const double *b, int n)
{
    double c=0.0;
    int i;

    for (i=0;i<n;i++) c+=a[i]*b[i];
    return c;
}

extern double norm(const double *a, int n)
{
    return sqrt(dot(a,a,n));
}

extern void cross3(const double *a, const double *b, double *c)
{
    c[0]=a[1]*b[2]-a[2]*b[1];
    c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}

extern int normv3(const double *a, double *b)
{
    double r=norm(a,3);
    int i;

    if (r<=0.0) return 0;
    for (i=0;i<3;i++) b[i]=a[i]/r;
    return 1;
}

extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    (void)time;
    (void)sat;
    (void)nav;
    (void)opt;
    if (rs) memset(rs,0,sizeof(double)*6);
    if (dts) dts[0]=dts[1]=0.0;
    if (var) *var=0.0;
    return 0;
}

extern void satantoff(gtime_t time, const double *rs, int sat,
                      const nav_t *nav, double *dant)
{
    (void)time;
    (void)rs;
    (void)sat;
    (void)nav;
    if (dant) dant[0]=dant[1]=dant[2]=0.0;
}

extern int sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs,
                      double *dts, double *var)
{
    (void)time;
    (void)sat;
    (void)nav;
    (void)rs;
    (void)dts;
    (void)var;
    return 0;
}

static int close_value(double actual, double expected, double tolerance)
{
    return fabs(actual-expected)<=tolerance;
}

static int close_vec3(const double *actual, const double *expected,
                      double tolerance)
{
    return close_value(actual[0],expected[0],tolerance)&&
           close_value(actual[1],expected[1],tolerance)&&
           close_value(actual[2],expected[2],tolerance);
}

static gtime_t test_time(void)
{
    return gpst2time(2300,100000.0);
}

static void init_eph(eph_t *eph, int sat, int iode, int iodc,
                     gtime_t toe)
{
    memset(eph,0,sizeof(*eph));
    eph->sat=sat;
    eph->iode=iode;
    eph->iodc=iodc;
    eph->sva=0;
    eph->svh=0;
    eph->week=2300;
    eph->toe=toe;
    eph->toc=toe;
    eph->ttr=toe;
    eph->toes=time2gpst(toe,NULL);
    eph->A=26560000.0;
    eph->e=0.01;
    eph->i0=0.94;
    eph->OMG0=1.0;
    eph->omg=0.7;
    eph->M0=0.3;
    eph->deln=0.0;
    eph->OMGd=-8.0E-9;
    eph->idot=0.0;
    eph->f0=2.0E-5;
    eph->f1=1.0E-12;
    eph->f2=0.0;
}

static void init_b2b(B2bssr_t *b2b, gtime_t time, int iodn)
{
    memset(b2b,0,sizeof(*b2b));
    b2b->t0[0]=time;
    b2b->t0[2]=time;
    b2b->iodssr[0]=5;
    b2b->iodssr[2]=5;
    b2b->iodcorr[0]=9;
    b2b->iodcorr[1]=9;
    b2b->iodn=iodn;
    b2b->ura=0;
    b2b->deph[0]=1.0;
    b2b->deph[1]=-2.0;
    b2b->deph[2]=0.5;
    b2b->dclk[0]=2.5;
    b2b->update=1;
}

static int run_satpos(const nav_t *nav, int sat, int ephopt, double *rs,
                      double *dts, double *var, int *svh)
{
    memset(rs,0,sizeof(double)*6);
    dts[0]=dts[1]=0.0;
    *var=0.0;
    *svh=0;
    return satpos(test_time(),test_time(),sat,ephopt,nav,rs,dts,var,svh);
}

static int test_iodn_match_success(void)
{
    eph_t eph[2],ref_eph[1];
    nav_t nav={0},ref_nav={0};
    double rs[6],dts[2],var,rs_ref[6],dts_ref[2],var_ref;
    int sat=satno(SYS_GPS,1),svh,svh_ref,ok=1;

    init_eph(eph+0,sat,99,99,test_time());
    eph[0].M0+=0.2;
    eph[0].f0+=1E-4;
    init_eph(eph+1,sat,11,11,test_time());
    nav.eph=eph;
    nav.n=2;
    init_b2b(nav.B2bssr+sat,test_time(),11);
    nav.B2bssr[sat].deph[0]=0.0;
    nav.B2bssr[sat].deph[1]=0.0;
    nav.B2bssr[sat].deph[2]=0.0;
    nav.B2bssr[sat].dclk[0]=0.0;

    ref_eph[0]=eph[1];
    ref_nav.eph=ref_eph;
    ref_nav.n=1;
    ok&=run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh);
    ok&=run_satpos(&ref_nav,sat,EPHOPT_BRDC,rs_ref,dts_ref,&var_ref,&svh_ref);
    ok&=svh==0&&svh_ref==0;
    ok&=close_vec3(rs,rs_ref,TEST_EPS_POS);
    ok&=close_value(dts[0],dts_ref[0],TEST_EPS_CLK);
    return ok;
}

static int test_iodn_mismatch_fails(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var;
    int sat=satno(SYS_GPS,2),svh;

    init_eph(&eph,sat,10,10,test_time());
    nav.eph=&eph;
    nav.n=1;
    init_b2b(nav.B2bssr+sat,test_time(),11);
    return !run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;
}

static int test_bds_iodc_match(void)
{
    eph_t eph[2],ref_eph[1];
    nav_t nav={0},ref_nav={0};
    double rs[6],dts[2],var,rs_ref[6],dts_ref[2],var_ref;
    int sat=satno(SYS_CMP,19),svh,svh_ref,ok=1;

    init_eph(eph+0,sat,77,5,test_time());
    eph[0].M0+=0.1;
    init_eph(eph+1,sat,12,77,test_time());
    eph[1].code=EPHCODE_BDS_CNV1;
    nav.eph=eph;
    nav.n=2;
    init_b2b(nav.B2bssr+sat,test_time(),77);
    nav.B2bssr[sat].deph[0]=0.0;
    nav.B2bssr[sat].deph[1]=0.0;
    nav.B2bssr[sat].deph[2]=0.0;
    nav.B2bssr[sat].dclk[0]=0.0;

    ref_eph[0]=eph[1];
    ref_nav.eph=ref_eph;
    ref_nav.n=1;
    ok&=run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh);
    ok&=run_satpos(&ref_nav,sat,EPHOPT_BRDC,rs_ref,dts_ref,&var_ref,&svh_ref);
    ok&=svh==0&&svh_ref==0;
    ok&=close_vec3(rs,rs_ref,TEST_EPS_POS);
    ok&=close_value(dts[0],dts_ref[0],TEST_EPS_CLK);
    return ok;
}

static int test_bds_cnv1_orbit_rates(void)
{
    eph_t cnv1,equivalent,without_rates;
    double rs_cnv1[3],rs_equivalent[3],rs_without_rates[3],delta[3];
    double dts,var,tk=600.0;
    int sat=satno(SYS_CMP,19),ok=1;

    init_eph(&cnv1,sat,12,77,timeadd(test_time(),-tk));
    cnv1.code=EPHCODE_BDS_CNV1;
    cnv1.Adot=0.02;
    cnv1.ndot=2E-13;

    equivalent=cnv1;
    equivalent.code=0;
    equivalent.A+=cnv1.Adot*tk;
    equivalent.M0+=(sqrt(TEST_MU_CMP/(cnv1.A*cnv1.A*cnv1.A))+cnv1.deln+
                    0.5*cnv1.ndot*tk-
                    sqrt(TEST_MU_CMP/(equivalent.A*equivalent.A*equivalent.A))-
                    equivalent.deln)*tk;

    without_rates=cnv1;
    without_rates.Adot=0.0;
    without_rates.ndot=0.0;

    eph2pos(test_time(),&cnv1,rs_cnv1,&dts,&var);
    eph2pos(test_time(),&equivalent,rs_equivalent,&dts,&var);
    eph2pos(test_time(),&without_rates,rs_without_rates,&dts,&var);
    ok&=close_vec3(rs_cnv1,rs_equivalent,TEST_EPS_POS);
    ok&=norm(rs_cnv1,3)>0.0;
    ok&=norm(rs_without_rates,3)>0.0;
    delta[0]=rs_cnv1[0]-rs_without_rates[0];
    delta[1]=rs_cnv1[1]-rs_without_rates[1];
    delta[2]=rs_cnv1[2]-rs_without_rates[2];
    ok&=norm(delta,3)>1.0;
    return ok;
}

static int test_failure_conditions(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var;
    int sat=satno(SYS_GPS,3),svh,ok=1;

    init_eph(&eph,sat,15,15,test_time());
    nav.eph=&eph;
    nav.n=1;

    ok&=!run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;

    init_b2b(nav.B2bssr+sat,test_time(),15);
    nav.B2bssr[sat].t0[2]=timeadd(test_time(),-12.0-DTTOL-0.1);
    ok&=!run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;

    init_b2b(nav.B2bssr+sat,test_time(),15);
    nav.B2bssr[sat].ura=63;
    ok&=!run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;

    init_b2b(nav.B2bssr+sat,test_time(),15);
    eph.svh=1;
    ok&=!run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;
    return ok;
}

static int test_zero_correction_reproduces_brdc(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var,rs_ref[6],dts_ref[2],var_ref;
    int sat=satno(SYS_GPS,4),svh,svh_ref,ok=1;

    init_eph(&eph,sat,22,22,test_time());
    nav.eph=&eph;
    nav.n=1;
    init_b2b(nav.B2bssr+sat,test_time(),22);
    nav.B2bssr[sat].deph[0]=0.0;
    nav.B2bssr[sat].deph[1]=0.0;
    nav.B2bssr[sat].deph[2]=0.0;
    nav.B2bssr[sat].dclk[0]=0.0;

    ok&=run_satpos(&nav,sat,EPHOPT_BRDC,rs_ref,dts_ref,&var_ref,&svh_ref);
    ok&=run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh);
    ok&=svh==0&&svh_ref==0;
    ok&=close_vec3(rs,rs_ref,TEST_EPS_POS);
    ok&=close_value(dts[0],dts_ref[0],TEST_EPS_CLK);
    return ok;
}

static int test_rac_and_clock_sign(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var,rs_ref[6],dts_ref[2],var_ref,delta[3];
    int sat=satno(SYS_GPS,5),svh,svh_ref,ok=1,i;

    init_eph(&eph,sat,31,31,test_time());
    nav.eph=&eph;
    nav.n=1;
    init_b2b(nav.B2bssr+sat,test_time(),31);

    ok&=run_satpos(&nav,sat,EPHOPT_BRDC,rs_ref,dts_ref,&var_ref,&svh_ref);
    ok&=b2b_rac_to_ecef(rs_ref,rs_ref+3,nav.B2bssr[sat].deph,delta);
    ok&=run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh);
    for (i=0;i<3;i++) ok&=close_value(rs[i],rs_ref[i]-delta[i],TEST_EPS_POS);
    ok&=close_value(dts[0],dts_ref[0]-nav.B2bssr[sat].dclk[0]/CLIGHT,
                    TEST_EPS_CLK);
    ok&=dts[0]<dts_ref[0];
    ok&=svh==0&&svh_ref==0&&var>0.0;
    return ok;
}

static int test_disabled_systems(void)
{
    eph_t gal;
    nav_t nav={0};
    double rs[6],dts[2],var;
    int gal_sat=satno(SYS_GAL,1),glo_sat=satno(SYS_GLO,1),svh,ok=1;

    init_eph(&gal,gal_sat,8,8,test_time());
    nav.eph=&gal;
    nav.n=1;
    init_b2b(nav.B2bssr+gal_sat,test_time(),8);
    ok&=!run_satpos(&nav,gal_sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;

    init_b2b(nav.B2bssr+glo_sat,test_time(),8);
    ok&=!run_satpos(&nav,glo_sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==-1;
    return ok;
}

static int test_b2b_indexing(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var;
    int sat=1,svh,ok=1;

    init_eph(&eph,sat,44,44,test_time());
    nav.eph=&eph;
    nav.n=1;
    init_b2b(nav.B2bssr+0,test_time(),44);
    ok&=!run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh);

    init_b2b(nav.B2bssr+sat,test_time(),44);
    ok&=run_satpos(&nav,sat,EPHOPT_B2b,rs,dts,&var,&svh)&&svh==0;

    memset(nav.B2bssr,0,sizeof(nav.B2bssr));
    init_b2b(nav.B2bssr+MAXSAT,test_time(),44);
    ok&=!run_satpos(&nav,MAXSAT,EPHOPT_B2b,rs,dts,&var,&svh);
    return ok;
}

static int test_existing_branches(void)
{
    eph_t eph;
    nav_t nav={0};
    double rs[6],dts[2],var;
    int sat=satno(SYS_GPS,6),svh,ok=1;

    init_eph(&eph,sat,50,50,test_time());
    nav.eph=&eph;
    nav.n=1;
    ok&=run_satpos(&nav,sat,EPHOPT_BRDC,rs,dts,&var,&svh)&&svh==0;
    ok&=!run_satpos(&nav,sat,EPHOPT_SSRAPC,rs,dts,&var,&svh);
    return ok;
}

int main(void)
{
    int iodn_success=test_iodn_match_success();
    int iodn_mismatch=test_iodn_mismatch_fails();
    int bds_iodc=test_bds_iodc_match();
    int bds_cnv1_rates=test_bds_cnv1_orbit_rates();
    int failures=test_failure_conditions();
    int zero_brdc=test_zero_correction_reproduces_brdc();
    int rac_clock=test_rac_and_clock_sign();
    int disabled=test_disabled_systems();
    int indexing=test_b2b_indexing();
    int existing=test_existing_branches();
    int all=iodn_success&&iodn_mismatch&&bds_iodc&&bds_cnv1_rates&&failures&&
            zero_brdc&&rac_clock&&disabled&&indexing&&existing;

    printf("IODN_MATCH_SUCCESS %d\n",iodn_success);
    printf("IODN_MISMATCH_FAIL %d\n",iodn_mismatch);
    printf("BDS_IODC_MATCH %d\n",bds_iodc);
    printf("BDS_CNV1_ORBIT_RATES %d\n",bds_cnv1_rates);
    printf("B2B_FAILURE_CONDITIONS %d\n",failures);
    printf("ZERO_CORRECTION_BRDC %d\n",zero_brdc);
    printf("RAC_AND_CLOCK_SIGN %d\n",rac_clock);
    printf("GAL_GLO_DISABLED %d\n",disabled);
    printf("B2B_INDEXING %d\n",indexing);
    printf("BRDC_SSR_BRANCHES %d\n",existing);
    printf("ALL_SATPOS_B2B %d\n",all);
    return all?0:1;
}
