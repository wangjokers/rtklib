/*------------------------------------------------------------------------------
* b2b.c : PPP-B2b common structures helpers
*
* This file contains B2b helpers shared by receiver decoders and the controlled
* raw-to-main-nav update path. Receiver framing and PPP correction application
* remain outside this module.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <float.h>

/* The ICD nominal ages are 96 s for orbit and 12 s for clock. The default
 * policy adds one 30 s observation interval, matching the reference project
 * while retaining the stricter causal, IOD, finite-value and magnitude gates.
 * Diagnostic builds may override the effective limits independently. */
#define B2B_ORBIT_NOMINAL_VALIDITY 96.0
#define B2B_CLOCK_NOMINAL_VALIDITY 12.0
#define B2B_AGE_GRACE 30.0
#ifndef B2B_ORBIT_VALIDITY
#define B2B_ORBIT_VALIDITY (B2B_ORBIT_NOMINAL_VALIDITY+B2B_AGE_GRACE)
#endif
#define B2B_CBIAS_VALIDITY 86400.0
#ifndef B2B_CLOCK_VALIDITY
#define B2B_CLOCK_VALIDITY (B2B_CLOCK_NOMINAL_VALIDITY+B2B_AGE_GRACE)
#endif
#define B2B_MAX_ORBIT_CORR 10.0
#define B2B_MAX_CLOCK_CORR (1E-6*CLIGHT)
#define B2B_TIME_EPS 1E-9

/* finite-value check shared by the correction readiness/math helpers --------*/
static int b2b_finite(double value)
{
    return value==value&&value<=DBL_MAX&&value>=-DBL_MAX;
}

/* Euclidean norm for one three-component correction/vector ------------------*/
static double b2b_norm3(const double *value)
{
    return sqrt(value[0]*value[0]+value[1]*value[1]+
                value[2]*value[2]);
}

/* check one fixed-validity PPP-B2b product age ------------------------------*/
static int b2b_age_valid(gtime_t time, gtime_t product_time,
                         double validity, double *age)
{
    double value;

    if (age) *age=0.0;
    if (!product_time.time) return 0;

    value=timediff(time,product_time);
    if (age) *age=value;
    return b2b_finite(value)&&value>=-DTTOL-B2B_TIME_EPS&&
           value<=validity+DTTOL+B2B_TIME_EPS;
}

/* merge B2b system masks into one slot-ordered array -------------------------*/
static void merge_b2b_mask_arrays(const B2bmask_t *mask, int *merged)
{
    int index=0;

    memcpy(merged+index,mask->MASK_BD,sizeof(mask->MASK_BD)); index+=63;
    memcpy(merged+index,mask->MASK_GPS,sizeof(mask->MASK_GPS)); index+=37;
    memcpy(merged+index,mask->MASK_GAL,sizeof(mask->MASK_GAL)); index+=37;
    memcpy(merged+index,mask->MASK_GLO,sizeof(mask->MASK_GLO));
}

/* convert PPP-B2b broadcast slot to RTKLIB satellite number ------------------
* convert a PPP-B2b broadcast slot to an RTKLIB satellite number
* args   : int       slot        I   PPP-B2b slot number (1-174)
* return : RTKLIB satellite number (0: invalid or disabled satellite system)
* notes  : B2b slot order is BDS, GPS, Galileo and GLONASS. satno() performs
*          the final check against systems enabled at compile time.
*-----------------------------------------------------------------------------*/
extern int b2b_slot2satno(int slot)
{
    int sys=SYS_NONE,prn=0;

    if (slot>=B2B_BDS_MINSAT&&slot<=B2B_BDS_MAXSAT) {
        sys=SYS_CMP; prn=slot-B2B_BDS_MINSAT+1;
    }
    else if (slot>=B2B_GPS_MINSAT&&slot<=B2B_GPS_MAXSAT) {
        sys=SYS_GPS; prn=slot-B2B_GPS_MINSAT+1;
    }
    else if (slot>=B2B_GAL_MINSAT&&slot<=B2B_GAL_MAXSAT) {
        sys=SYS_GAL; prn=slot-B2B_GAL_MINSAT+1;
    }
    else if (slot>=B2B_GLO_MINSAT&&slot<=B2B_GLO_MAXSAT) {
        sys=SYS_GLO; prn=slot-B2B_GLO_MINSAT+1;
    }
    else {
        return 0;
    }
    return satno(sys,prn);
}

/* convert a PPP-B2b MASK to RTKLIB satellite numbers -------------------------
* convert active slots in a PPP-B2b MASK to RTKLIB satellite numbers
* args   : B2bmask_t *mask       IO  receiver-local PPP-B2b MASK state
* return : number of valid satellites stored in mask->satno[] (0: error/none)
* notes  : mask->satno[] is rebuilt in B2b slot order and mask->satnum is
*          updated. CLOCK messages use this receiver-local list for slot-index
*          lookup.
*-----------------------------------------------------------------------------*/
extern int b2b_mask2satno(B2bmask_t *mask)
{
    int merged[B2B_MAXSAT]={0};
    int i,n=0;

    if (!mask) return 0;

    memset(mask->satno,0,sizeof(mask->satno));
    merge_b2b_mask_arrays(mask,merged);

    for (i=0;i<B2B_MAXSAT;i++) {
        if (merged[i]) {
            int sat=b2b_slot2satno(i+1);
            if (sat>0) mask->satno[n++]=sat;
        }
    }
    mask->satnum=n;
    return n;
}

/* convert B2b BDT seconds-of-day to RTKLIB GPST ------------------------------
* convert PPP-B2b BDT seconds-of-day to a complete RTKLIB GPST epoch
* args   : gtime_t   header_time I   receiver frame header time (GPST)
*          double    bdt_sod     I   PPP-B2b payload time (BDT seconds-of-day)
* return : complete product reference time (GPST), {0} if input is invalid
* notes  : the BDT calendar date is derived from header_time. A half-day rule
*          resolves a product epoch that crosses midnight relative to the
*          receiver frame header.
*-----------------------------------------------------------------------------*/
extern gtime_t b2b_tod2time(gtime_t header_time, double bdt_sod)
{
    gtime_t zero={0},header_bdt,day_bdt,data_bdt;
    double ep[6],header_sod,dt;

    if (!header_time.time||bdt_sod<0.0||bdt_sod>=86400.0) return zero;

    header_bdt=gpst2bdt(header_time);
    time2epoch(header_bdt,ep);
    header_sod=ep[3]*3600.0+ep[4]*60.0+ep[5];

    ep[3]=ep[4]=ep[5]=0.0;
    day_bdt=epoch2time(ep);
    data_bdt=timeadd(day_bdt,bdt_sod);

    dt=header_sod-bdt_sod;
    if (dt>43200.0) {
        data_bdt=timeadd(data_bdt,86400.0);
    }
    else if (dt<-43200.0) {
        data_bdt=timeadd(data_bdt,-86400.0);
    }
    return bdt2gpst(data_bdt);
}

/* consume decoded PPP-B2b products into the main navigation object -----------
* consume receiver-decoded PPP-B2b products into the main navigation object
* args   : nav_t     *nav         IO  main navigation data
*          raw_t     *raw         IO  receiver raw data control
* return : number of satellites transferred in this call (0: no update/error)
* notes  : only raw->nav.B2bssr[sat] entries with update=1 are copied. The
*          receiver decoder accumulates orbit, code-bias and clock fields in one
*          B2bssr_t, so the complete structure is copied to preserve products
*          arriving in separate messages.
*
*          B2b storage uses [sat] indexing: valid indices are 1..MAXSAT and
*          element 0 is unused. After a successful copy, raw-side update is
*          cleared and main-nav update remains 1 for downstream observation.
*-----------------------------------------------------------------------------*/
extern int b2b_update_nav_from_raw(nav_t *nav, raw_t *raw)
{
    int sat,n=0;

    if (!nav||!raw) return 0;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (!raw->nav.B2bssr[sat].update) continue;

        nav->B2bssr[sat]=raw->nav.B2bssr[sat];
        nav->B2bssr[sat].update=1;
        raw->nav.B2bssr[sat].update=0;
        n++;
    }
    return n;
}

/* check PPP-B2b orbit age against the effective validity --------------------
* args   : gtime_t         time I   requested signal/solution time (GPST)
*          const B2bssr_t *b2b  I   satellite PPP-B2b products
*          double         *age  O   orbit age (s), optional
* return : 1 if -DTTOL <= age <= 126 s + DTTOL by default, otherwise 0
* notes  : the default includes a 30 s engineering grace after the ICD
*          nominal 96 s. b2b->udi[0] never extends this validity.
*-----------------------------------------------------------------------------*/
extern int b2b_orbit_age_valid(gtime_t time, const B2bssr_t *b2b,
                               double *age)
{
    if (!b2b) {
        if (age) *age=0.0;
        return 0;
    }
    return b2b_age_valid(time,b2b->t0[0],B2B_ORBIT_VALIDITY,age);
}

/* check PPP-B2b code-bias age against the ICD nominal validity --------------*/
extern int b2b_cbias_age_valid(gtime_t time, const B2bssr_t *b2b,
                               double *age)
{
    if (!b2b) {
        if (age) *age=0.0;
        return 0;
    }
    return b2b_age_valid(time,b2b->t0[1],B2B_CBIAS_VALIDITY,age);
}

/* validate one signal-specific PPP-B2b code-bias correction -----------
* args   : gtime_t         time I   requested signal/solution time
*          const B2bssr_t *b2b  I   satellite PPP-B2b products
*          int             code I   RTKLIB CODE_* observation code
*          double         *bias O   code bias (m), optional
*          double         *age  O   code-bias age (s), optional
* return : 1 if the requested code bias is present, current and finite
* notes  : update is an event flag and is deliberately ignored. A real 0.0 m
*          bias is valid only when cbias_valid[code] is set.
*-----------------------------------------------------------------------------*/
extern int b2b_cbias_ready(gtime_t time, const B2bssr_t *b2b,
                           int code, double *bias, double *age)
{
    double age_value=0.0,value;

    if (bias) *bias=0.0;
    if (age) *age=0.0;
    if (!b2b||code<=CODE_NONE||code>MAXCODE) return 0;
    if (!b2b_cbias_age_valid(time,b2b,&age_value)) {
        if (age) *age=age_value;
        return 0;
    }
    if (!b2b->cbias_valid[code]) {
        if (age) *age=age_value;
        return 0;
    }
    value=b2b->cbias[code];
    if (!b2b_finite(value)) {
        if (age) *age=age_value;
        return 0;
    }
    if (bias) *bias=value;
    if (age) *age=age_value;
    return 1;
}

/* resolve an exact or explicitly enabled experimental BDS X-code bias ------
* notes  : exact signal-specific products always take priority. Experimental
*          fallback is restricted to BDS C1X/C5X and never guesses a missing
*          D/P component in mean mode.
*-----------------------------------------------------------------------------*/
extern int b2b_resolve_cbias(gtime_t time, const B2bssr_t *b2b,
                             int sys, int code, int xbias_mode,
                             double *bias, double *age, int *used_mode)
{
    double data_bias=0.0,pilot_bias=0.0,data_age=0.0,pilot_age=0.0;
    int data_code=CODE_NONE,pilot_code=CODE_NONE;

    if (bias) *bias=0.0;
    if (age) *age=0.0;
    if (used_mode) *used_mode=B2BXBIAS_OFF;

    /* A future exact X product must override every experimental profile. */
    if (b2b_cbias_ready(time,b2b,code,bias,age)) return 1;
    if (sys!=SYS_CMP||xbias_mode==B2BXBIAS_OFF) return 0;

    if (code==CODE_L1X) {
        data_code=CODE_L1D;
        pilot_code=CODE_L1P;
    }
    else if (code==CODE_L5X) {
        data_code=CODE_L5D;
        pilot_code=CODE_L5P;
    }
    else return 0;

    if (xbias_mode==B2BXBIAS_DATA) {
        if (!b2b_cbias_ready(time,b2b,data_code,bias,age)) return 0;
    }
    else if (xbias_mode==B2BXBIAS_PILOT) {
        if (!b2b_cbias_ready(time,b2b,pilot_code,bias,age)) return 0;
    }
    else if (xbias_mode==B2BXBIAS_MEAN) {
        int data_ok=b2b_cbias_ready(time,b2b,data_code,&data_bias,&data_age);
        int pilot_ok=b2b_cbias_ready(time,b2b,pilot_code,&pilot_bias,&pilot_age);

        if (age) *age=data_age>pilot_age?data_age:pilot_age;
        if (!data_ok||!pilot_ok) return 0;
        if (bias) *bias=(data_bias+pilot_bias)*0.5;
    }
    else return 0;

    if (used_mode) *used_mode=xbias_mode;
    return 1;
}
/* check PPP-B2b clock age against the effective validity --------------------
* return : 1 if -DTTOL <= age <= 42 s + DTTOL by default, otherwise 0
* notes  : the default includes a 30 s engineering grace after the ICD
*          nominal 12 s. Future products beyond DTTOL remain invalid.
*-----------------------------------------------------------------------------*/
extern int b2b_clock_age_valid(gtime_t time, const B2bssr_t *b2b,
                               double *age)
{
    if (!b2b) {
        if (age) *age=0.0;
        return 0;
    }
    return b2b_age_valid(time,b2b->t0[2],B2B_CLOCK_VALIDITY,age);
}

/* validate one coherent PPP-B2b orbit/clock correction set ------------------
* args   : gtime_t         time       I   requested signal/solution time
*          const B2bssr_t *b2b        I   satellite PPP-B2b products
*          double         *orbit_age  O   orbit age (s), optional
*          double         *clock_age  O   clock age (s), optional
* return : 1 if both products are present, current, coherent and bounded
* notes  : update is an event flag and is deliberately ignored. The protection
*          limits match standard RTKLIB SSR: 10 m orbit norm and 1 us clock
*          range. UDI is diagnostic only and does not change product age.
*-----------------------------------------------------------------------------*/
extern int b2b_orbit_clock_ready(gtime_t time, const B2bssr_t *b2b,
                                 double *orbit_age, double *clock_age)
{
    int i,orbit_ok,clock_ok;

    if (!b2b) {
        if (orbit_age) *orbit_age=0.0;
        if (clock_age) *clock_age=0.0;
        return 0;
    }
    orbit_ok=b2b_orbit_age_valid(time,b2b,orbit_age);
    clock_ok=b2b_clock_age_valid(time,b2b,clock_age);
    if (!orbit_ok||!clock_ok) return 0;
    if (b2b->iodssr[0]!=b2b->iodssr[2]||
        b2b->iodcorr[0]!=b2b->iodcorr[1]) return 0;

    for (i=0;i<3;i++) {
        if (!b2b_finite(b2b->deph[i])||
            !b2b_finite(b2b->ddeph[i])||
            !b2b_finite(b2b->dclk[i])) return 0;
    }
    if (b2b_norm3(b2b->deph)>B2B_MAX_ORBIT_CORR||
        fabs(b2b->dclk[0])>B2B_MAX_CLOCK_CORR) return 0;
    return 1;
}

/* convert PPP-B2b 6-bit URAI to satellite-position variance -----------------
* args   : int     urai      I   PPP-B2b URAI (0-63)
*          double *variance  O   variance (m^2)
* return : 1 if URAI is usable, otherwise 0
* notes  : URAI 0 is the ICD minimum sigma of 0.5 mm. URAI 63 and values
*          outside the 6-bit range explicitly reject the correction.
*-----------------------------------------------------------------------------*/
extern int b2b_urai_variance(int urai, double *variance)
{
    double sigma;
    int class_,value;

    if (!variance) return 0;
    *variance=0.0;
    if (urai<0||urai>=63) return 0;
    if (urai==0) {
        sigma=0.0005;
    }
    else {
        class_=(urai>>3)&7;
        value=urai&7;
        sigma=(pow(3.0,class_)*(1.0+value/4.0)-1.0)*1E-3;
    }
    if (!b2b_finite(sigma)) return 0;
    *variance=sigma*sigma;
    return b2b_finite(*variance);
}

/* form the PPP-B2b RAC orbit correction vector in ECEF ----------------------
* args   : double *r     I   broadcast satellite position ECEF (m)
*          double *v     I   broadcast satellite velocity ECEF (m/s)
*          double *rac   I   {radial,along-track,cross-track} correction (m)
*          double *ecef  O   correction vector in ECEF (m)
* return : 1 on success, 0 for null, non-finite or degenerate vectors
* notes  : ea=normalize(v), ec=normalize(r x v), er=ea x ec. Therefore
*          ecef=er*radial+ea*along+ec*cross. This helper does not subtract the
*          correction from a satellite position or modify navigation data.
*-----------------------------------------------------------------------------*/
extern int b2b_rac_to_ecef(const double *r, const double *v,
                           const double *rac, double *ecef)
{
    double ea[3],ec[3],er[3],cross[3],nv,nc;
    int i;

    if (!r||!v||!rac||!ecef) return 0;
    for (i=0;i<3;i++) {
        if (!b2b_finite(r[i])||!b2b_finite(v[i])||
            !b2b_finite(rac[i])) return 0;
    }
    nv=b2b_norm3(v);
    cross[0]=r[1]*v[2]-r[2]*v[1];
    cross[1]=r[2]*v[0]-r[0]*v[2];
    cross[2]=r[0]*v[1]-r[1]*v[0];
    nc=b2b_norm3(cross);
    if (!b2b_finite(nv)||!b2b_finite(nc)||nv<=0.0||nc<=0.0) return 0;

    for (i=0;i<3;i++) {
        ea[i]=v[i]/nv;
        ec[i]=cross[i]/nc;
    }
    er[0]=ea[1]*ec[2]-ea[2]*ec[1];
    er[1]=ea[2]*ec[0]-ea[0]*ec[2];
    er[2]=ea[0]*ec[1]-ea[1]*ec[0];
    for (i=0;i<3;i++) {
        ecef[i]=er[i]*rac[0]+ea[i]*rac[1]+ec[i]*rac[2];
        if (!b2b_finite(ecef[i])) return 0;
    }
    return 1;
}

/* apply PPP-B2b clock range correction to a broadcast clock -----------------
* args   : double  broadcast_dts I   broadcast satellite clock bias (s)
*          double  dclk          I   PPP-B2b clock range correction (m)
*          double *corrected_dts O   corrected satellite clock bias (s)
* return : 1 on success, 0 for null or non-finite input/output
* notes  : PPP-B2b keeps its specified negative sign:
*          corrected = broadcast - dclk / CLIGHT.
*-----------------------------------------------------------------------------*/
extern int b2b_clock_correct(double broadcast_dts, double dclk,
                             double *corrected_dts)
{
    if (!corrected_dts||!b2b_finite(broadcast_dts)||!b2b_finite(dclk)) {
        return 0;
    }
    *corrected_dts=broadcast_dts-dclk/CLIGHT;
    return b2b_finite(*corrected_dts);
}

/* clear main-nav PPP-B2b update events without clearing products ------------
* args   : nav_t *nav IO  main navigation data
* return : number of update flags cleared (0: none/null)
* notes  : WARNING: B2b storage uses [sat] indexing. Only indices 1..MAXSAT
*          are visited; index 0 and all product fields remain untouched.
*-----------------------------------------------------------------------------*/
extern int b2b_clear_nav_updates(nav_t *nav)
{
    int sat,n=0;

    if (!nav) return 0;
    for (sat=1;sat<=MAXSAT;sat++) {
        if (!nav->B2bssr[sat].update) continue;
        nav->B2bssr[sat].update=0;
        n++;
    }
    return n;
}
