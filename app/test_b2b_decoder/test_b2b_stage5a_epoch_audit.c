#include "rtklib.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STAGE5A_ORBIT_STRICT_AGE 96.0
#define STAGE5A_CLOCK_STRICT_AGE 12.0
#define STAGE5A_ORBIT_EFFECTIVE_AGE 126.0
#define STAGE5A_CLOCK_EFFECTIVE_AGE 42.0
#define STAGE5A_ELEVATION_MASK (10.0*D2R)

typedef struct {
    raw_t raw;
    FILE *stream;
    int initialized;
    int eof;
    int pending;
    gtime_t pending_time;
    long decoded_events;
    long nav_updates;
    long errors;
} replay_t;

int showmsg(const char *format, ...)
{
    va_list arg;

    va_start(arg,format);
    vfprintf(stderr,format,arg);
    va_end(arg);
    return 0;
}

void settspan(gtime_t ts, gtime_t te)
{
    (void)ts;
    (void)te;
}

void settime(gtime_t time)
{
    (void)time;
}

static int finite_b2b_products(const B2bssr_t *b2b)
{
    int i;

    if (!b2b) return 0;
    for (i=0;i<3;i++) {
        if (!isfinite(b2b->deph[i])||!isfinite(b2b->ddeph[i])||
            !isfinite(b2b->dclk[i])) return 0;
    }
    return norm(b2b->deph,3)<=10.0&&fabs(b2b->dclk[0])<=1E-6*CLIGHT;
}

static int exact_cnv1_match(const nav_t *nav, gtime_t time, int sat, int iodn)
{
    double best_age=MAXDTOE_CMP+2.0;
    int i,selected=-1;

    if (!nav||!nav->eph) return 0;
    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;
        double age;

        if (eph->sat!=sat||eph->code!=EPHCODE_BDS_CNV1||eph->svh||
            eph->iodc!=iodn) continue;
        age=fabs(timediff(eph->toe,time));
        if (age>MAXDTOE_CMP+1.0) continue;
        if (selected<0||age<best_age-DTTOL) {
            selected=i;
            best_age=age;
        }
    }
    return selected>=0;
}

static int replay_open(replay_t *replay, const char *path)
{
    if (!replay||!path) return 0;
    memset(replay,0,sizeof(*replay));
    if (!init_raw(&replay->raw,STRFMT_SINO)) return 0;
    replay->initialized=1;
    if (!(replay->stream=fopen(path,"rb"))) {
        free_raw(&replay->raw);
        memset(replay,0,sizeof(*replay));
        return 0;
    }
    return 1;
}

static void replay_close(replay_t *replay)
{
    if (!replay) return;
    if (replay->stream) fclose(replay->stream);
    if (replay->initialized) free_raw(&replay->raw);
    memset(replay,0,sizeof(*replay));
}

static int replay_update(replay_t *replay, nav_t *nav, gtime_t obs_time,
                         int *epoch_updates)
{
    int ret,n=0;

    if (epoch_updates) *epoch_updates=0;
    if (!replay||!nav||!replay->stream) return 0;

    if (replay->pending) {
        if (timediff(replay->pending_time,obs_time)>DTTOL) return 1;
        n+=b2b_update_nav_from_raw(nav,&replay->raw);
        replay->pending=0;
        replay->pending_time.time=0;
        replay->pending_time.sec=0.0;
    }
    while (!replay->eof) {
        ret=input_rawf(&replay->raw,STRFMT_SINO,replay->stream);
        if (ret==-2) {
            replay->eof=1;
            break;
        }
        if (ret<0) {
            replay->errors++;
            return 0;
        }
        if (ret==0) continue;
        replay->decoded_events++;
        if (timediff(replay->raw.time,obs_time)>DTTOL) {
            replay->pending=1;
            replay->pending_time=replay->raw.time;
            break;
        }
        n+=b2b_update_nav_from_raw(nav,&replay->raw);
    }
    replay->nav_updates+=n;
    if (epoch_updates) *epoch_updates=n;
    return 1;
}

static int read_inputs(const char *obs_path, const char *nav_path, obs_t *obs,
                       nav_t *nav, sta_t *sta)
{
    gtime_t zero={0};

    memset(obs,0,sizeof(*obs));
    memset(nav,0,sizeof(*nav));
    memset(sta,0,sizeof(*sta));
    if (readrnxt(obs_path,1,zero,zero,0.0,"-SYS=C",obs,nav,sta)<0) {
        fprintf(stderr,"failed to read observation RINEX: %s\n",obs_path);
        return 0;
    }
    if (readrnxt(nav_path,1,zero,zero,0.0,"-SYS=C",obs,nav,NULL)<0) {
        fprintf(stderr,"failed to read navigation RINEX: %s\n",nav_path);
        return 0;
    }
    sortobs(obs);
    uniqnav(nav);
    return obs->n>0&&nav->n>0;
}

static const char *invalid_reason(int excluded, int dual_obs, int above_mask,
                                  int orbit_present, int clock_present,
                                  double orbit_age, double clock_age,
                                  int orbit_effective, int clock_effective,
                                  int iodssr_match, int iodcorr_match,
                                  int numeric_valid, int urai_current,
                                  int cnv1_match, int cbias_ready,
                                  int satpos_ok)
{
    if (excluded) return "EXCLUDED_CONFIG";
    if (!dual_obs) return "MISSING_DUAL_OBS";
    if (!orbit_present) return "ORBIT_MISSING";
    if (!clock_present) return "CLOCK_MISSING";
    if (orbit_age<-DTTOL) return "ORBIT_FUTURE";
    if (clock_age<-DTTOL) return "CLOCK_FUTURE";
    if (!orbit_effective) return "ORBIT_STALE";
    if (!clock_effective) return "CLOCK_STALE";
    if (!iodssr_match) return "IODSSR_MISMATCH";
    if (!iodcorr_match) return "IODCORR_MISMATCH";
    if (!numeric_valid) return "NUMERIC_INVALID";
    if (!urai_current) return "URAI_CURRENT_INVALID";
    if (!cnv1_match) return "IODN_CNV1_MISSING";
    if (!cbias_ready) return "CBIAS_MISSING";
    if (!satpos_ok) return "SATPOS_FAILED";
    if (!above_mask) return "BELOW_ELEVATION";
    return "NONE";
}

static int write_epoch_rows(FILE *output, int epoch_index, const obsd_t *data,
                            int n, nav_t *nav, const sta_t *sta,
                            int replay_updates)
{
    double *rs=NULL,*dts=NULL,*var=NULL,*rs_brdc=NULL,*dts_brdc=NULL;
    double *var_brdc=NULL,receiver_pos[3]={0},pos[3]={0};
    int *svh=NULL,*svh_brdc=NULL,i,receiver_ok=0;
    char epoch[32];

    if (!(rs=(double *)calloc((size_t)n*6,sizeof(*rs)))||
        !(dts=(double *)calloc((size_t)n*2,sizeof(*dts)))||
        !(var=(double *)calloc((size_t)n,sizeof(*var)))||
        !(svh=(int *)calloc((size_t)n,sizeof(*svh)))||
        !(rs_brdc=(double *)calloc((size_t)n*6,sizeof(*rs_brdc)))||
        !(dts_brdc=(double *)calloc((size_t)n*2,sizeof(*dts_brdc)))||
        !(var_brdc=(double *)calloc((size_t)n,sizeof(*var_brdc)))||
        !(svh_brdc=(int *)calloc((size_t)n,sizeof(*svh_brdc)))) {
        free(rs); free(dts); free(var); free(svh);
        free(rs_brdc); free(dts_brdc); free(var_brdc); free(svh_brdc);
        return 0;
    }
    if (sta&&norm(sta->pos,3)>RE_WGS84/2.0) {
        for (i=0;i<3;i++) receiver_pos[i]=sta->pos[i];
        ecef2pos(receiver_pos,pos);
        receiver_ok=1;
    }
    satposs(data[0].time,data,n,nav,EPHOPT_BRDC,rs_brdc,dts_brdc,
            var_brdc,svh_brdc);
    satposs(data[0].time,data,n,nav,EPHOPT_B2b,rs,dts,var,svh);
    time2str(data[0].time,epoch,0);

    for (i=0;i<n;i++) {
        const obsd_t *record=data+i;
        const B2bssr_t *b2b;
        double orbit_age=0.0,clock_age=0.0,bias=0.0,bias_age=0.0;
        double variance=0.0,elevation=NAN,e[3];
        int prn=0,sys=satsys(record->sat,&prn);
        int excluded,dual_obs,above_mask=0,orbit_present,clock_present;
        int orbit_effective,clock_effective,strict_orbit,strict_clock;
        int iodssr_match,iodcorr_match,numeric_valid,urai_current,urai_icd;
        int cnv1_match,cbias_f1,cbias_f2,cbias_ready,satpos_ok;
        int product_ready,current_usable,icd_reliable_usable;
        const char *product_class,*reason;
        char id[16];

        if (sys!=SYS_CMP) continue;
        b2b=nav->B2bssr+record->sat;
        satno2id(record->sat,id);
        excluded=prn<=18;
        dual_obs=record->code[0]>CODE_NONE&&record->code[1]>CODE_NONE&&
                 record->P[0]!=0.0&&record->P[1]!=0.0&&
                 record->L[0]!=0.0&&record->L[1]!=0.0;
        satpos_ok=norm(rs+i*6,3)>0.0&&svh[i]==0;
        if (receiver_ok&&norm(rs_brdc+i*6,3)>0.0&&
            geodist(rs_brdc+i*6,receiver_pos,e)>0.0) {
            elevation=satazel(pos,e,NULL);
            above_mask=elevation+1E-12>=STAGE5A_ELEVATION_MASK;
        }
        orbit_present=b2b->t0[0].time!=0;
        clock_present=b2b->t0[2].time!=0;
        orbit_effective=b2b_orbit_age_valid(data[0].time,b2b,&orbit_age);
        clock_effective=b2b_clock_age_valid(data[0].time,b2b,&clock_age);
        strict_orbit=orbit_present&&orbit_age>=-DTTOL&&
                     orbit_age<=STAGE5A_ORBIT_STRICT_AGE+DTTOL;
        strict_clock=clock_present&&clock_age>=-DTTOL&&
                     clock_age<=STAGE5A_CLOCK_STRICT_AGE+DTTOL;
        iodssr_match=orbit_present&&clock_present&&
                     b2b->iodssr[0]==b2b->iodssr[2];
        iodcorr_match=orbit_present&&clock_present&&
                      b2b->iodcorr[0]==b2b->iodcorr[1];
        numeric_valid=finite_b2b_products(b2b);
        urai_current=b2b_urai_variance(b2b->ura,&variance);
        urai_icd=b2b->ura>0&&b2b->ura<63;
        cnv1_match=orbit_present&&
                   exact_cnv1_match(nav,data[0].time,record->sat,b2b->iodn);
        cbias_f1=dual_obs&&b2b_cbias_ready(data[0].time,b2b,
                                           record->code[0],&bias,&bias_age);
        cbias_f2=dual_obs&&b2b_cbias_ready(data[0].time,b2b,
                                           record->code[1],&bias,&bias_age);
        cbias_ready=cbias_f1&&cbias_f2;
        product_ready=!excluded&&dual_obs&&orbit_effective&&clock_effective&&
                      iodssr_match&&iodcorr_match&&numeric_valid&&
                      urai_current&&cnv1_match&&cbias_ready&&satpos_ok;
        current_usable=product_ready&&above_mask;
        icd_reliable_usable=current_usable&&urai_icd;
        product_class=product_ready?
                      (strict_orbit&&strict_clock?"STRICT":"DEGRADED"):
                      "INVALID";
        reason=invalid_reason(excluded,dual_obs,above_mask,orbit_present,
                              clock_present,orbit_age,clock_age,
                              orbit_effective,clock_effective,iodssr_match,
                              iodcorr_match,numeric_valid,urai_current,
                              cnv1_match,cbias_ready,satpos_ok);

        fprintf(output,
                "%d,%s,%s,%d,%d,%d,%d,%d,%.3f,%d,%d,%d,%.3f,%.3f,"
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d\n",
                epoch_index,epoch,id,prn,record->code[0],record->code[1],
                excluded,dual_obs,isfinite(elevation)?elevation*R2D:NAN,
                above_mask,orbit_present,clock_present,orbit_age,clock_age,
                orbit_effective,
                clock_effective,strict_orbit,strict_clock,iodssr_match,
                iodcorr_match,numeric_valid,urai_current,urai_icd,cnv1_match,
                cbias_f1,cbias_f2,satpos_ok,product_ready,current_usable,
                product_class,reason,icd_reliable_usable);
        (void)replay_updates;
    }
    free(rs); free(dts); free(var); free(svh);
    free(rs_brdc); free(dts_brdc); free(var_brdc); free(svh_brdc);
    return 1;
}

int main(int argc, char **argv)
{
    obs_t *obs=NULL;
    nav_t *nav=NULL;
    sta_t *sta=NULL;
    replay_t replay;
    FILE *output=NULL;
    int i,epoch_index=0,ok=0;

    if (argc!=5) {
        fprintf(stderr,"Usage: %s <obs_mo> <nav_mn> <sino_b2b_raw> "
                "<satellite_status_csv>\n",argv[0]);
        return 1;
    }
    if (!(obs=(obs_t *)calloc(1,sizeof(*obs)))||
        !(nav=(nav_t *)calloc(1,sizeof(*nav)))||
        !(sta=(sta_t *)calloc(1,sizeof(*sta)))) {
        fprintf(stderr,"memory allocation failed\n");
        goto cleanup;
    }
    if (!read_inputs(argv[1],argv[2],obs,nav,sta)) goto cleanup;
    if (!replay_open(&replay,argv[3])) {
        fprintf(stderr,"failed to open Sino B2b raw: %s\n",argv[3]);
        goto cleanup;
    }
    if (!(output=fopen(argv[4],"wb"))) {
        fprintf(stderr,"failed to create diagnostic CSV: %s\n",argv[4]);
        replay_close(&replay);
        goto cleanup;
    }
    fprintf(output,"epoch_index,time_gpst,satellite,prn,code_f1,code_f2,"
            "excluded_config,dual_observation,elevation_deg,above_elevation_mask,"
            "orbit_present,clock_present,orbit_age_s,clock_age_s,orbit_effective_age,"
            "clock_effective_age,orbit_strict_age,clock_strict_age,"
            "iodssr_match,iodcorr_match,numeric_valid,urai_current_usable,"
            "urai_icd_reliable,cnv1_iodn_match,cbias_f1_ready,cbias_f2_ready,"
            "satpos_b2b_ok,product_ready,current_usable,product_class,invalid_reason,"
            "icd_reliable_usable\n");

    for (i=0;i<obs->n;) {
        int begin=i,n=0,updates=0;
        gtime_t time=obs->data[i].time;

        while (i<obs->n&&fabs(timediff(obs->data[i].time,time))<=DTTOL) {
            i++;
            n++;
        }
        epoch_index++;
        if (!replay_update(&replay,nav,time,&updates)) {
            fprintf(stderr,"B2b replay failed at epoch %d\n",epoch_index);
            fclose(output);
            output=NULL;
            replay_close(&replay);
            goto cleanup;
        }
        if (!write_epoch_rows(output,epoch_index,obs->data+begin,n,nav,sta,
                              updates)) {
            fprintf(stderr,"diagnostic row allocation failed\n");
            fclose(output);
            output=NULL;
            replay_close(&replay);
            goto cleanup;
        }
    }
    fclose(output);
    output=NULL;
    fprintf(stderr,"STAGE5A_EPOCH_AUDIT epochs=%d obs_records=%d eph=%d "
            "decoded_events=%ld nav_updates=%ld errors=%ld pending=%d eof=%d\n",
            epoch_index,obs->n,nav->n,replay.decoded_events,
            replay.nav_updates,replay.errors,replay.pending,replay.eof);
    ok=epoch_index==2880&&replay.errors==0;
    replay_close(&replay);

cleanup:
    if (output) fclose(output);
    if (obs) freeobs(obs);
    if (nav) freenav(nav,0x7F);
    free(obs); free(nav); free(sta);
    return ok?0:1;
}
