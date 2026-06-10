/*------------------------------------------------------------------------------
* b2b.c : PPP-B2b common structures helpers
*
* This file is the stage 3A landing point for B2b helpers shared by future
* receiver decoders and nav update code. It intentionally does not contain a
* Unicore/SinoGNSS decoder, raw dispatch, nav update, or PPP correction logic.
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
*
* B2b MASK/CLOCK messages address satellites by B2b slot, not by RTKLIB's
* compact sat number. The slot ranges are ordered as BDS, GPS, Galileo and
* GLONASS. satno() performs the final check against the systems enabled at
* compile time, so slots outside the local RTKLIB build return 0.
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
*
* The receiver decoder keeps the latest MASK in B2bmask_t. Orbit and code-bias
* messages carry explicit slots, while CLOCK messages often carry an index into
* this MASK list. b2b_mask2satno() prepares that list once after every MASK.
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
*
* Unicore PPPB2BINFO messages carry the product reference time as BDT
* seconds-of-day. The receiver frame header already has a full GPST time. This
* helper converts the header time to BDT calendar date, attaches the payload SOD,
* fixes cross-day cases with the same half-day rule used in the stage 1 decoder,
* and converts the result back to GPST for RTKLIB storage.
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

