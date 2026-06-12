/*------------------------------------------------------------------------------
* b2b.c : PPP-B2b common structures helpers
*
* This file contains B2b helpers shared by receiver decoders and the controlled
* raw-to-main-nav update path. Receiver framing and PPP correction application
* remain outside this module.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

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
