#include "rtklib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_EPS 1E-12

static int close_value(double actual, double expected, double tolerance)
{
    return fabs(actual-expected)<=tolerance;
}

static gtime_t test_time(void)
{
    return gpst2time(2300,100000.0);
}

static void init_cbias(B2bssr_t *b2b, int code, float bias)
{
    memset(b2b,0,sizeof(*b2b));
    b2b->t0[1]=test_time();
    b2b->iodssr[1]=7;
    b2b->cbias[code]=bias;
    b2b->cbias_valid[code]=1;
    b2b->update=0;
}

static void set_cbias(B2bssr_t *b2b, int code, float bias)
{
    b2b->cbias[code]=bias;
    b2b->cbias_valid[code]=1;
}

static void init_xbias(B2bssr_t *b2b)
{
    memset(b2b,0,sizeof(*b2b));
    b2b->t0[1]=test_time();
    set_cbias(b2b,CODE_L1D,0.10f);
    set_cbias(b2b,CODE_L1P,0.30f);
    set_cbias(b2b,CODE_L5D,0.50f);
    set_cbias(b2b,CODE_L5P,0.90f);
}

static int apply_b2b_cbias_for_test(int ephopt, gtime_t time,
                                    const B2bssr_t *b2b, int code,
                                    double raw_p, double *corrected_p)
{
    double bias=0.0;

    if (!corrected_p) return 0;
    *corrected_p=raw_p;
    if (ephopt!=EPHOPT_B2b) return 1;
    if (!b2b_cbias_ready(time,b2b,code,&bias,NULL)) return 0;
    *corrected_p=raw_p-bias;
    return 1;
}

static int test_valid_bias_applies_to_pseudorange(void)
{
    B2bssr_t b2b;
    double corrected,bias,age;
    int code=CODE_L1C,ok=1;

    init_cbias(&b2b,code,1.25f);
    ok&=b2b_cbias_ready(test_time(),&b2b,code,&bias,&age);
    ok&=close_value(bias,1.25,1E-7);
    ok&=close_value(age,0.0,TEST_EPS);
    ok&=apply_b2b_cbias_for_test(EPHOPT_B2b,test_time(),&b2b,code,
                                 100.0,&corrected);
    ok&=close_value(corrected,98.75,1E-7);
    return ok;
}

static int test_zero_bias_is_valid_when_flagged(void)
{
    B2bssr_t b2b;
    double corrected,bias=-1.0;
    int code=CODE_L2I,ok=1;

    init_cbias(&b2b,code,0.0f);
    ok&=b2b_cbias_ready(test_time(),&b2b,code,&bias,NULL);
    ok&=close_value(bias,0.0,TEST_EPS);
    ok&=apply_b2b_cbias_for_test(EPHOPT_B2b,test_time(),&b2b,code,
                                 123.5,&corrected);
    ok&=close_value(corrected,123.5,TEST_EPS);
    return ok;
}

static int test_missing_valid_flag_rejects_zero_bias(void)
{
    B2bssr_t b2b;
    double corrected;
    int code=CODE_L1C;

    memset(&b2b,0,sizeof(b2b));
    b2b.t0[1]=test_time();
    b2b.cbias[code]=0.0f;
    return !b2b_cbias_ready(test_time(),&b2b,code,NULL,NULL)&&
           !apply_b2b_cbias_for_test(EPHOPT_B2b,test_time(),&b2b,code,
                                     100.0,&corrected);
}

static int test_code_bounds(void)
{
    B2bssr_t b2b;
    int ok=1;

    init_cbias(&b2b,CODE_L1C,0.5f);
    b2b.cbias[0]=9.0f;
    b2b.cbias_valid[0]=1;

    ok&=!b2b_cbias_ready(test_time(),&b2b,0,NULL,NULL);
    ok&=!b2b_cbias_ready(test_time(),&b2b,MAXCODE+1,NULL,NULL);
    ok&=!b2b_cbias_ready(test_time(),NULL,CODE_L1C,NULL,NULL);
    return ok;
}

static int test_age_and_missing_time(void)
{
    B2bssr_t b2b;
    double age=0.0;
    int code=CODE_L1C,ok=1;

    init_cbias(&b2b,code,0.5f);
    ok&=!b2b_cbias_ready(timeadd(test_time(),86400.0+DTTOL+0.001),
                         &b2b,code,NULL,&age);
    ok&=age>86400.0;

    init_cbias(&b2b,code,0.5f);
    b2b.t0[1].time=0;
    b2b.t0[1].sec=0.0;
    ok&=!b2b_cbias_ready(test_time(),&b2b,code,NULL,&age);
    return ok;
}

static int test_non_finite_rejected(void)
{
    B2bssr_t b2b;
    int code=CODE_L1C,ok=1;

    init_cbias(&b2b,code,(float)NAN);
    ok&=!b2b_cbias_ready(test_time(),&b2b,code,NULL,NULL);

    init_cbias(&b2b,code,(float)INFINITY);
    ok&=!b2b_cbias_ready(test_time(),&b2b,code,NULL,NULL);
    return ok;
}

static int test_update_is_not_readiness(void)
{
    B2bssr_t b2b;
    int code=CODE_L1C;

    init_cbias(&b2b,code,-0.75f);
    b2b.update=0;
    return b2b_cbias_ready(test_time(),&b2b,code,NULL,NULL);
}

static int test_non_b2b_mode_keeps_pseudorange(void)
{
    B2bssr_t b2b;
    double corrected=0.0;
    int code=CODE_L1C,ok=1;

    memset(&b2b,0,sizeof(b2b));
    ok&=apply_b2b_cbias_for_test(EPHOPT_BRDC,test_time(),&b2b,code,
                                 100.0,&corrected);
    ok&=close_value(corrected,100.0,TEST_EPS);
    return ok;
}

static int test_xbias_off_rejects_combined_codes(void)
{
    B2bssr_t b2b;

    init_xbias(&b2b);
    return !b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                              B2BXBIAS_OFF,NULL,NULL,NULL)&&
           !b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L5X,
                              B2BXBIAS_OFF,NULL,NULL,NULL);
}

static int test_xbias_data_profile(void)
{
    B2bssr_t b2b;
    double bias=0.0;
    int used=0,ok=1;

    init_xbias(&b2b);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                          B2BXBIAS_DATA,&bias,NULL,&used);
    ok&=used==B2BXBIAS_DATA&&close_value(bias,0.10,1E-7);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L5X,
                          B2BXBIAS_DATA,&bias,NULL,&used);
    ok&=used==B2BXBIAS_DATA&&close_value(bias,0.50,1E-7);
    return ok;
}

static int test_xbias_pilot_profile(void)
{
    B2bssr_t b2b;
    double bias=0.0;
    int used=0,ok=1;

    init_xbias(&b2b);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                          B2BXBIAS_PILOT,&bias,NULL,&used);
    ok&=used==B2BXBIAS_PILOT&&close_value(bias,0.30,1E-7);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L5X,
                          B2BXBIAS_PILOT,&bias,NULL,&used);
    ok&=used==B2BXBIAS_PILOT&&close_value(bias,0.90,1E-7);
    return ok;
}

static int test_xbias_mean_profile(void)
{
    B2bssr_t b2b;
    double bias=0.0,age=-1.0;
    int used=0,ok=1;

    init_xbias(&b2b);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                          B2BXBIAS_MEAN,&bias,&age,&used);
    ok&=used==B2BXBIAS_MEAN&&close_value(bias,0.20,1E-7);
    ok&=close_value(age,0.0,TEST_EPS);
    ok&=b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L5X,
                          B2BXBIAS_MEAN,&bias,&age,&used);
    ok&=used==B2BXBIAS_MEAN&&close_value(bias,0.70,1E-7);
    return ok;
}

static int test_xbias_mean_requires_both_components(void)
{
    B2bssr_t b2b;

    init_xbias(&b2b);
    b2b.cbias_valid[CODE_L1P]=0;
    return !b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                              B2BXBIAS_MEAN,NULL,NULL,NULL);
}

static int test_exact_x_bias_overrides_experiment(void)
{
    B2bssr_t b2b;
    double bias=0.0;
    int used=-1;

    init_xbias(&b2b);
    set_cbias(&b2b,CODE_L1X,1.25f);
    return b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                             B2BXBIAS_DATA,&bias,NULL,&used)&&
           used==B2BXBIAS_OFF&&close_value(bias,1.25,1E-7);
}

static int test_xbias_scope_and_invalid_mode(void)
{
    B2bssr_t b2b;

    init_xbias(&b2b);
    return !b2b_resolve_cbias(test_time(),&b2b,SYS_GPS,CODE_L1X,
                              B2BXBIAS_DATA,NULL,NULL,NULL)&&
           !b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L7X,
                              B2BXBIAS_DATA,NULL,NULL,NULL)&&
           !b2b_resolve_cbias(test_time(),&b2b,SYS_CMP,CODE_L1X,
                              99,NULL,NULL,NULL);
}

int main(void)
{
    int applied=test_valid_bias_applies_to_pseudorange();
    int zero=test_zero_bias_is_valid_when_flagged();
    int missing=test_missing_valid_flag_rejects_zero_bias();
    int bounds=test_code_bounds();
    int age=test_age_and_missing_time();
    int finite=test_non_finite_rejected();
    int update=test_update_is_not_readiness();
    int non_b2b=test_non_b2b_mode_keeps_pseudorange();
    int xoff=test_xbias_off_rejects_combined_codes();
    int xdata=test_xbias_data_profile();
    int xpilot=test_xbias_pilot_profile();
    int xmean=test_xbias_mean_profile();
    int xboth=test_xbias_mean_requires_both_components();
    int xexact=test_exact_x_bias_overrides_experiment();
    int xscope=test_xbias_scope_and_invalid_mode();
    int all=applied&&zero&&missing&&bounds&&age&&finite&&update&&non_b2b&&
            xoff&&xdata&&xpilot&&xmean&&xboth&&xexact&&xscope;

    printf("B2B_CBIAS_APPLIES %d\n",applied);
    printf("B2B_CBIAS_ZERO_VALID %d\n",zero);
    printf("B2B_CBIAS_MISSING_REJECTED %d\n",missing);
    printf("B2B_CBIAS_CODE_BOUNDS %d\n",bounds);
    printf("B2B_CBIAS_AGE_AND_T0 %d\n",age);
    printf("B2B_CBIAS_FINITE %d\n",finite);
    printf("B2B_CBIAS_UPDATE_IGNORED %d\n",update);
    printf("B2B_CBIAS_NON_B2B_UNCHANGED %d\n",non_b2b);
    printf("B2B_XBIAS_OFF_STRICT %d\n",xoff);
    printf("B2B_XBIAS_DATA %d\n",xdata);
    printf("B2B_XBIAS_PILOT %d\n",xpilot);
    printf("B2B_XBIAS_MEAN %d\n",xmean);
    printf("B2B_XBIAS_MEAN_REQUIRES_BOTH %d\n",xboth);
    printf("B2B_XBIAS_EXACT_OVERRIDES %d\n",xexact);
    printf("B2B_XBIAS_SCOPE %d\n",xscope);
    printf("ALL_B2B_CBIAS %d\n",all);
    return all?0:1;
}
