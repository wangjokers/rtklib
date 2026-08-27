#include "rtklib.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308
#define SINO_HEADER_LEN 28
#define SINO_TYPE_POS   (SINO_HEADER_LEN*8+44)

/* Keep BDS code selection at the production default. Frequency slots are
 * validated explicitly below instead of being hidden by -CLxx overrides. */
static const char *obs_opt="-SYS=GC -GL1C -GL2W";
static const char *nav_opt="-SYS=GC";
static const int bds_prns[]={6,7,19,23,32,41};
static const int gps_prns[]={1,7,19,30};
static int nav_eph_preuniq=0;
static int nav_cnv1_preuniq=0;
static int nav_cnv1_postuniq=0;

typedef struct {
    long frames[4];
    long nav_updates;
    long errors;
    gtime_t last_time;
    gtime_t first_future_time;
} b2b_replay_stats_t;

typedef struct {
    int obs_sats;
    int product_sats;
    int orbit_ready;
    int clock_ready;
    int orbit_clock_ready;
    int ura_ready;
    int eph_match;
    int satpos_ok;
    int cbias_f1_ready;
    int cbias_f2_ready;
    int generic_non_cnv1;
} gate_system_stats_t;

typedef struct {
    int observed;
    int clock_pairs;
    int final_pairs;
    int precise_pairs;
    double sum_abs_delta_dt_ns;
    double max_abs_delta_dt_ns;
    double sum_abs_tx_delta_ns;
    double max_abs_tx_delta_ns;
    double sum_final_pos_delta_mm;
    double max_final_pos_delta_mm;
    double sum_abs_residual_delta_mm;
    double max_abs_residual_delta_mm;
    double sum_precise_pos_sq;
    double max_precise_pos_m;
    double precise_clock_m[MAXOBS];
    int precise_clock_n;
} clock_seed_ab_stats_t;

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

static int diagnostic_time(const char *date, const char *clock, gtime_t *time)
{
    double ep[6]={0};

    if (!date||!clock||!time||
        sscanf(date,"%lf/%lf/%lf",ep,ep+1,ep+2)!=3||
        sscanf(clock,"%lf:%lf:%lf",ep+3,ep+4,ep+5)!=3) return 0;
    *time=epoch2time(ep);
    return time->time!=0;
}

static int diagnostic_format(const char *name)
{
    if (!name) return -1;
    if (!strcmp(name,"sino")) return STRFMT_SINO;
    if (!strcmp(name,"unicore")) return STRFMT_UNICORE;
    return -1;
}

static const char *diagnostic_format_name(int format)
{
    return format==STRFMT_SINO?"sino":
           format==STRFMT_UNICORE?"unicore":"unknown";
}

static void time_text(gtime_t time, char *buff, int n)
{
    if (!time.time) {
        snprintf(buff,n,"none");
        return;
    }
    time2str(time,buff,0);
}

static const char *code_name(int code, char *buff, int n)
{
    const char *obs=code2obs((uint8_t)code);

    if (obs&&*obs) snprintf(buff,n,"%s",obs);
    else snprintf(buff,n,"code%d",code);
    return buff;
}

/* Stage 4E signal gate: code2idx() is a slot index, not a priority value.
 * Keep every BDS index inside MAXFREQ and pin the intended reference mapping
 * before real RINEX data is allowed to reach the PPP diagnostic. */
static int test_bds_frequency_mapping(void)
{
    static const struct {
        const char *obs;
        int expected_index;
        double expected_frequency;
    } cases[]={
        {"2I",0,FREQ1_CMP}, /* B1I: first PPP frequency */
        {"6I",1,FREQ3_CMP}, /* B3I: second PPP frequency */
        {"7I",2,FREQ2_CMP}, /* B2I/B2b: third frequency */
        {"5X",3,FREQ5},     /* B2a */
        {"8X",4,FREQ8},     /* B2ab */
        {"1X",5,FREQ1}      /* B1C */
    };
    int i,ok=1;

    for (i=0;i<(int)(sizeof(cases)/sizeof(cases[0]));i++) {
        uint8_t code=obs2code(cases[i].obs);
        int index=code2idx(SYS_CMP,code);
        double frequency=code2freq(SYS_CMP,code,0);
        int current_ok=code!=CODE_NONE&&index==cases[i].expected_index&&
                       0<=index&&index<MAXFREQ&&
                       fabs(frequency-cases[i].expected_frequency)<1E-3;

        printf("BDS_FREQ_INDEX code=%s index=%d expected=%d freq=%.3f ok=%d\n",
               cases[i].obs,index,cases[i].expected_index,frequency,current_ok);
        ok&=current_ok;
    }
    printf("BDS_FREQ_MAPPING %d\n",ok);
    return ok;
}

static int frame_index(const raw_t *raw, int format)
{
    int type;

    if (format==STRFMT_SINO) {
        if (raw->len<SINO_HEADER_LEN||SINO_TYPE_POS+6>raw->len*8) return -1;
        type=(int)getbitu(raw->buff,SINO_TYPE_POS,6);
        return 1<=type&&type<=4?type-1:-1;
    }
    type=raw->buff[4]|(raw->buff[5]<<8);

    if (type==UNICORE_MSG_B2B_INFO1) return 0;
    if (type==UNICORE_MSG_B2B_INFO2) return 1;
    if (type==UNICORE_MSG_B2B_INFO3) return 2;
    if (type==UNICORE_MSG_B2B_INFO4) return 3;
    return -1;
}

static int replay_b2b_until(const char *path, int format, gtime_t target,
                            nav_t *nav,
                            b2b_replay_stats_t *stats)
{
    raw_t *raw;
    FILE *fp;
    int ret,index;

    memset(stats,0,sizeof(*stats));
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))) {
        fprintf(stderr,"failed to allocate raw_t\n");
        return 0;
    }
    if (!(fp=fopen(path,"rb"))) {
        fprintf(stderr,"failed to open B2b raw: %s\n",path);
        free(raw);
        return 0;
    }
    if (!init_raw(raw,format)) {
        fprintf(stderr,"failed to initialize %s B2b raw context\n",
                diagnostic_format_name(format));
        fclose(fp);
        free(raw);
        return 0;
    }
    for (;;) {
        ret=input_rawf(raw,format,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;
        if (timediff(raw->time,target)>DTTOL) {
            stats->first_future_time=raw->time;
            break;
        }
        if ((index=frame_index(raw,format))>=0) stats->frames[index]++;
        stats->last_time=raw->time;
        stats->nav_updates+=b2b_update_nav_from_raw(nav,raw);
    }
    free_raw(raw);
    fclose(fp);
    free(raw);
    return 1;
}

static int count_cnv1_eph(const nav_t *nav)
{
    int i,count=0;

    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].code==EPHCODE_BDS_CNV1) count++;
    }
    return count;
}

static int read_inputs(const char *obs_path, int nnav, char **nav_paths,
                       gtime_t target, obs_t *obs, nav_t *nav, sta_t *sta)
{
    gtime_t zero={0};
    int i;

    memset(obs,0,sizeof(*obs));
    memset(nav,0,sizeof(*nav));
    memset(sta,0,sizeof(*sta));

    if (readrnxt(obs_path,1,target,target,0.0,obs_opt,obs,nav,sta)<0) {
        fprintf(stderr,"failed to read observation RINEX: %s\n",obs_path);
        return 0;
    }
    fprintf(stderr,"INPUT_READ kind=obs path=%s obs=%d eph=%d\n",
            obs_path,obs->n,nav->n);
    for (i=0;i<nnav;i++) {
        int before=nav->n;

        if (readrnxt(nav_paths[i],1,zero,zero,0.0,nav_opt,obs,nav,NULL)<0) {
            fprintf(stderr,"failed to read navigation RINEX: %s\n",
                    nav_paths[i]);
            return 0;
        }
        fprintf(stderr,"INPUT_READ kind=nav path=%s added_eph=%d total_eph=%d\n",
                nav_paths[i],nav->n-before,nav->n);
    }
    nav_eph_preuniq=nav->n;
    nav_cnv1_preuniq=count_cnv1_eph(nav);
    sortobs(obs);
    uniqnav(nav);
    nav_cnv1_postuniq=count_cnv1_eph(nav);
    fprintf(stderr,"INPUT_READ kind=uniq obs=%d eph=%d cnv1=%d\n",obs->n,
            nav->n,nav_cnv1_postuniq);
    return obs->n>0&&nav->n>0;
}

static const obsd_t *find_obs_record(const obs_t *obs, gtime_t target, int sat)
{
    int i;

    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat&&fabs(timediff(obs->data[i].time,target))<=DTTOL) {
            return obs->data+i;
        }
    }
    return NULL;
}

static int eph_is_cnv1(const eph_t *eph)
{
    return eph&&eph->code==EPHCODE_BDS_CNV1;
}

static const char *eph_msg_type(const eph_t *eph)
{
    int sys;

    if (!eph) return "NONE";
    sys=satsys(eph->sat,NULL);
    if (sys==SYS_GPS) return "LNAV";
    if (sys==SYS_CMP) return eph_is_cnv1(eph)?"CNV1":"D1/D2";
    return "OTHER";
}

static double diagnostic_eph_tmax(int sys)
{
    return sys==SYS_CMP?MAXDTOE_CMP+1.0:
           sys==SYS_GPS?MAXDTOE+1.0:0.0;
}

/* Mirror the current generic SYS_CMP seleph() rule without changing it. */
static const eph_t *current_generic_eph(const nav_t *nav, gtime_t target,
                                        int sat)
{
    const eph_t *selected=NULL;
    double age,tmax;
    double best_age;
    int i,sys=satsys(sat,NULL),cur_cnv1,best_cnv1=0;

    if ((tmax=diagnostic_eph_tmax(sys))<=0.0) return NULL;
    best_age=tmax+1.0;

    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;

        if (eph->sat!=sat) continue;
        age=fabs(timediff(eph->toe,target));
        if (age>tmax) continue;
        cur_cnv1=sys==SYS_CMP&&eph_is_cnv1(eph);
        if (!selected||(best_cnv1&&!cur_cnv1)||
            (best_cnv1==cur_cnv1&&age<=best_age)) {
            selected=eph;
            best_age=age;
            best_cnv1=cur_cnv1;
        }
    }
    return selected;
}

/* Mirror seleph_B2b() and return the exact healthy IODN/CNV1 ephemeris. */
static const eph_t *diagnostic_b2b_eph(const nav_t *nav, gtime_t target,
                                       int sat, int iodn)
{
    const eph_t *selected=NULL;
    double age,tmax,best_age;
    int i,sys=satsys(sat,NULL);

    if ((tmax=diagnostic_eph_tmax(sys))<=0.0) return NULL;
    best_age=tmax+1.0;
    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;
        int iod_match;

        if (eph->sat!=sat||eph->svh) continue;
        if (sys==SYS_CMP&&!eph_is_cnv1(eph)) continue;
        iod_match=sys==SYS_CMP?eph->iodc==iodn:
                  sys==SYS_GPS?eph->iode==iodn:0;
        if (!iod_match) continue;
        age=fabs(timediff(eph->toe,target));
        if (age>tmax) continue;
        if (!selected||age<best_age-DTTOL) {
            selected=eph;
            best_age=age;
        }
    }
    return selected;
}

/* Mirror the current pntpos.c::gettgd() rule without changing it. */
static const eph_t *current_gettgd_eph(const nav_t *nav, gtime_t target,
                                       int sat, int type)
{
    const eph_t *selected=NULL;
    double age,best_age=MAXDTOE_CMP+2.0;
    int i,want_cnv1=type>=2;

    if (satsys(sat,NULL)!=SYS_CMP) return NULL;

    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;

        if (eph->sat!=sat||eph_is_cnv1(eph)!=want_cnv1) continue;
        age=fabs(timediff(eph->toe,target));
        if (age>MAXDTOE_CMP+1.0) continue;
        if (age<=best_age) {
            selected=eph;
            best_age=age;
        }
    }
    return selected;
}

static void print_model_sources(const obsd_t *obs, const nav_t *nav,
                                gtime_t target, int sat)
{
    const eph_t *satpos_eph=current_generic_eph(nav,target,sat);
    int sys=satsys(sat,NULL),type=obs&&obs->code[0]!=CODE_L2I?2:0;
    const eph_t *tgd_eph=sys==SYS_CMP?
                         current_gettgd_eph(nav,target,sat,type):NULL;
    char toe[32],toc[32];

    if (satpos_eph) {
        time_text(satpos_eph->toe,toe,sizeof(toe));
        time_text(satpos_eph->toc,toc,sizeof(toc));
        printf("  GENERIC_SATPOS_EPH type=%s toe=%s toc=%s iode=%d iodc=%d "
               "svh=%d\n",eph_msg_type(satpos_eph),toe,toc,
               satpos_eph->iode,satpos_eph->iodc,satpos_eph->svh);
    }
    else printf("  GENERIC_SATPOS_EPH none\n");

    if (tgd_eph) {
        time_text(tgd_eph->toe,toe,sizeof(toe));
        time_text(tgd_eph->toc,toc,sizeof(toc));
        printf("  PRANGE_TGD_EPH type=%s toe=%s toc=%s iode=%d iodc=%d "
               "svh=%d tgd=%.12e/%.12e/%.12e/%.12e/%.12e/%.12e\n",
               eph_msg_type(tgd_eph),toe,toc,tgd_eph->iode,tgd_eph->iodc,
               tgd_eph->svh,tgd_eph->tgd[0],tgd_eph->tgd[1],
               tgd_eph->tgd[2],tgd_eph->tgd[3],tgd_eph->tgd[4],
               tgd_eph->tgd[5]);
    }
    else printf("  PRANGE_TGD_EPH %s\n",sys==SYS_CMP?"none":"n/a");
}

static void print_obs_codes(const obsd_t *obs, const nav_t *nav)
{
    char name[16];
    int i;

    if (!obs) {
        printf("  OBS_PRESENT 0\n");
        return;
    }
    printf("  OBS_PRESENT 1\n");
    printf("  OBS_SELECTED_CODES");
    for (i=0;i<NFREQ;i++) {
        if (obs->code[i]<=CODE_NONE) {
            printf(" f%d=none",i+1);
            continue;
        }
        printf(" f%d=%s(%d):P=%.3f:freq=%.3f",i+1,
               code_name(obs->code[i],name,sizeof(name)),obs->code[i],
               obs->P[i],sat2freq(obs->sat,obs->code[i],nav));
    }
    printf("\n");
    printf("  OBS_AVAILABLE_CODES");
    for (i=0;i<NFREQ+NEXOBS;i++) {
        if (obs->code[i]<=CODE_NONE) continue;
        printf(" slot%d=%s(%d):P=%.3f",i+1,
               code_name(obs->code[i],name,sizeof(name)),obs->code[i],
               obs->P[i]);
    }
    printf("\n");
}

static int print_cbias_valid(const B2bssr_t *b2b)
{
    char name[16];
    int code,count=0;

    printf("  B2B_CBIAS_VALID");
    for (code=1;code<=MAXCODE;code++) {
        if (!b2b->cbias_valid[code]) continue;
        printf(" %s(%d)=%.3f",code_name(code,name,sizeof(name)),code,
               b2b->cbias[code]);
        count++;
    }
    if (!count) printf(" none");
    printf("\n");
    return count;
}

static int print_eph_candidates(const nav_t *nav, gtime_t target, int sat,
                                int iodn, int *selected)
{
    char toe[32],toc[32];
    double tmax=diagnostic_eph_tmax(satsys(sat,NULL));
    double best_age=tmax+1.0;
    int i,sys=satsys(sat,NULL),total=0,healthy=0,iod_match=0,age_match=0;
    int best_cnv1=0;

    *selected=-1;
    if (tmax<=0.0) return 0;
    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;
        double age;
        int match,age_ok,cur_cnv1;

        if (eph->sat!=sat) continue;
        age=fabs(timediff(eph->toe,target));
        match=sys==SYS_CMP?eph_is_cnv1(eph)&&eph->iodc==iodn:
              sys==SYS_GPS?eph->iode==iodn:0;
        age_ok=age<=tmax;
        if (!match&&!age_ok) continue;

        total++;
        if (!eph->svh) healthy++;
        if (match) iod_match++;
        if (match&&age_ok&&!eph->svh) {
            age_match++;
            cur_cnv1=sys==SYS_CMP&&eph_is_cnv1(eph);
            if (*selected<0||age<best_age-DTTOL||
                (fabs(age-best_age)<=DTTOL&&cur_cnv1&&!best_cnv1)) {
                *selected=i;
                best_age=age;
                best_cnv1=cur_cnv1;
            }
        }
        time_text(eph->toe,toe,sizeof(toe));
        time_text(eph->toc,toc,sizeof(toc));
        printf("  EPH_CAND type=%s iode=%d iodc=%d toe=%s toc=%s "
               "svh=%d age=%.0f iod_match=%d age_ok=%d Adot=%.6g ndot=%.6g\n",
               eph_msg_type(eph),eph->iode,eph->iodc,toe,toc,eph->svh,age,
               match,age_ok,eph->Adot,eph->ndot);
    }
    printf("  EPH_SUMMARY candidates=%d healthy=%d iod_match=%d usable=%d\n",
           total,healthy,iod_match,age_match);
    return age_match;
}

/* Mirror seleph_B2b() without exposing a production-only selector. */
static int diagnostic_b2b_eph_match(const nav_t *nav, gtime_t target,
                                    int sat, int iodn)
{
    return diagnostic_b2b_eph(nav,target,sat,iodn)!=NULL;
}

static void collect_gate_system_stats(const obs_t *obs, const nav_t *nav,
                                      gtime_t target, int sys,
                                      gate_system_stats_t *stats)
{
    int i;

    memset(stats,0,sizeof(*stats));
    for (i=0;i<obs->n;i++) {
        const obsd_t *record=obs->data+i;
        const B2bssr_t *b2b;
        const eph_t *generic;
        double age=0.0,bias=0.0,variance=0.0;
        double rs[6]={0},dts[2]={0},var=0.0;
        int svh=-1;

        if (satsys(record->sat,NULL)!=sys) continue;
        stats->obs_sats++;
        b2b=nav->B2bssr+record->sat; /* WARNING: PPP-B2b uses [sat]. */
        if (b2b->t0[0].time||b2b->t0[1].time||b2b->t0[2].time) {
            stats->product_sats++;
        }
        if (b2b_orbit_age_valid(target,b2b,&age)) stats->orbit_ready++;
        if (b2b_clock_age_valid(target,b2b,&age)) stats->clock_ready++;
        if (b2b_orbit_clock_ready(target,b2b,NULL,NULL)) {
            stats->orbit_clock_ready++;
            if (b2b_urai_variance(b2b->ura,&variance)) stats->ura_ready++;
        }
        if (diagnostic_b2b_eph_match(nav,target,record->sat,b2b->iodn)) {
            stats->eph_match++;
        }
        if (satpos(target,target,record->sat,EPHOPT_B2b,nav,rs,dts,&var,&svh)) {
            stats->satpos_ok++;
        }
        if (record->code[0]>CODE_NONE&&record->P[0]!=0.0&&
            b2b_cbias_ready(target,b2b,record->code[0],&bias,&age)) {
            stats->cbias_f1_ready++;
        }
        if (NFREQ>1&&record->code[1]>CODE_NONE&&record->P[1]!=0.0&&
            b2b_cbias_ready(target,b2b,record->code[1],&bias,&age)) {
            stats->cbias_f2_ready++;
        }
        generic=current_generic_eph(nav,target,record->sat);
        if (sys==SYS_CMP&&generic&&!eph_is_cnv1(generic)) {
            stats->generic_non_cnv1++;
        }
    }
}

static void print_gate_system_stats(const char *name,
                                    const gate_system_stats_t *stats)
{
    printf("GATE_SYSTEM sys=%s obs=%d products=%d orbit=%d clock=%d "
           "orbit_clock=%d ura=%d eph_match=%d satpos=%d cbias_f1=%d "
           "cbias_f2=%d generic_non_cnv1=%d\n",name,stats->obs_sats,
           stats->product_sats,stats->orbit_ready,stats->clock_ready,
           stats->orbit_clock_ready,stats->ura_ready,stats->eph_match,
           stats->satpos_ok,stats->cbias_f1_ready,stats->cbias_f2_ready,
           stats->generic_non_cnv1);
}

static int first_pseudorange(const obsd_t *obs, double *pr, int *slot)
{
    int i;

    if (!obs||!pr||!slot) return 0;
    for (i=0;i<NFREQ;i++) {
        if (obs->P[i]==0.0) continue;
        *pr=obs->P[i];
        *slot=i;
        return 1;
    }
    return 0;
}

/* Use the same R/A/C basis as b2b_rac_to_ecef(). */
static int ecef_delta_to_b2b_rac(const double *rs, const double *delta,
                                 double *rac)
{
    double ea[3],ec[3],er[3],cross[3],nv,nc;
    int i;

    if (!rs||!delta||!rac) return 0;
    nv=norm(rs+3,3);
    cross[0]=rs[1]*rs[5]-rs[2]*rs[4];
    cross[1]=rs[2]*rs[3]-rs[0]*rs[5];
    cross[2]=rs[0]*rs[4]-rs[1]*rs[3];
    nc=norm(cross,3);
    if (nv<=0.0||nc<=0.0) return 0;
    for (i=0;i<3;i++) {
        ea[i]=rs[i+3]/nv;
        ec[i]=cross[i]/nc;
    }
    er[0]=ea[1]*ec[2]-ea[2]*ec[1];
    er[1]=ea[2]*ec[0]-ea[0]*ec[2];
    er[2]=ea[0]*ec[1]-ea[1]*ec[0];
    rac[0]=dot(delta,er,3);
    rac[1]=dot(delta,ea,3);
    rac[2]=dot(delta,ec,3);
    return 1;
}

static void update_max(double value, double *maximum)
{
    if (value>*maximum) *maximum=value;
}

/* Compare the current generic clock seed with the exact IODN/CNV1 seed.
 * Only the seed changes: both branches then call the same final satpos_B2b(). */
static void print_clock_seed_ab(const obs_t *obs, const nav_t *nav,
                                gtime_t target, const double *rr)
{
    clock_seed_ab_stats_t stats={0};
    double rr_norm=rr?norm(rr,3):0.0;
    int i,rr_ok=rr_norm>RE_WGS84/2.0;

    printf("CLOCK_SEED_AB_REFERENCE_RR ok=%d xyz=%.4f/%.4f/%.4f\n",
           rr_ok,rr?rr[0]:0.0,rr?rr[1]:0.0,rr?rr[2]:0.0);
    printf("PRECISE_COUNTS ne=%d nc=%d frame=COM\n",nav->ne,nav->nc);

    for (i=0;i<obs->n;i++) {
        const obsd_t *record=obs->data+i;
        const B2bssr_t *b2b;
        const eph_t *generic,*exact;
        double pr,dt_generic,dt_cnv1,delta_dt_ns,tx_delta_ns;
        double rs_generic[6]={0},rs_cnv1[6]={0};
        double dts_generic[2]={0},dts_cnv1[2]={0};
        double var_generic=0.0,var_cnv1=0.0;
        double broadcast_generic[3]={0},broadcast_cnv1[3]={0};
        double broadcast_dt_generic=0.0,broadcast_dt_cnv1=0.0;
        double family_delta[3],family_pos_delta_m,family_clock_delta_m;
        double final_delta[3],final_pos_delta_mm=NAN;
        double final_clock_delta_mm=NAN,range_delta_mm=NAN;
        double model_delta_mm=NAN,residual_delta_mm=NAN;
        double precise_rs[6]={0},precise_dts[2]={0},precise_var=0.0;
        double precise_delta[3],precise_rac[3]={NAN,NAN,NAN};
        double broadcast_precise_delta[3];
        double broadcast_precise_rac[3]={NAN,NAN,NAN};
        double applied_delta[3],applied_rac[3]={NAN,NAN,NAN};
        double broadcast_precise_pos_m=NAN,broadcast_precise_clock_m=NAN;
        double precise_pos_m=NAN,precise_clock_m=NAN;
        double los_generic[3],los_cnv1[3];
        double rho_generic,rho_cnv1;
        gtime_t pseudorange_tx,tx_generic,tx_cnv1;
        char id[16],code[16];
        int slot,svh_generic=-1,svh_cnv1=-1,final_ok,precise_ok=0,j;

        if (satsys(record->sat,NULL)!=SYS_CMP) continue;
        stats.observed++;
        satno2id(record->sat,id);
        b2b=nav->B2bssr+record->sat;
        if (!first_pseudorange(record,&pr,&slot)) {
            printf("CLOCK_SEED_AB_SKIP sat=%s reason=no_pseudorange\n",id);
            continue;
        }
        generic=current_generic_eph(nav,target,record->sat);
        exact=diagnostic_b2b_eph(nav,target,record->sat,b2b->iodn);
        if (!generic||!exact) {
            printf("CLOCK_SEED_AB_SKIP sat=%s reason=%s generic=%s iodn=%d\n",
                   id,!generic?"no_generic_eph":"no_exact_iodn_cnv1",
                   eph_msg_type(generic),b2b->iodn);
            continue;
        }

        pseudorange_tx=timeadd(record->time,-pr/CLIGHT);
        dt_generic=eph2clk(pseudorange_tx,generic);
        dt_cnv1=eph2clk(pseudorange_tx,exact);
        tx_generic=timeadd(pseudorange_tx,-dt_generic);
        tx_cnv1=timeadd(pseudorange_tx,-dt_cnv1);
        delta_dt_ns=(dt_cnv1-dt_generic)*1E9;
        tx_delta_ns=timediff(tx_cnv1,tx_generic)*1E9;
        stats.clock_pairs++;
        stats.sum_abs_delta_dt_ns+=fabs(delta_dt_ns);
        stats.sum_abs_tx_delta_ns+=fabs(tx_delta_ns);
        update_max(fabs(delta_dt_ns),&stats.max_abs_delta_dt_ns);
        update_max(fabs(tx_delta_ns),&stats.max_abs_tx_delta_ns);

        eph2pos(tx_generic,generic,broadcast_generic,&broadcast_dt_generic,
                &var_generic);
        eph2pos(tx_cnv1,exact,broadcast_cnv1,&broadcast_dt_cnv1,&var_cnv1);
        for (j=0;j<3;j++) {
            family_delta[j]=broadcast_cnv1[j]-broadcast_generic[j];
        }
        family_pos_delta_m=norm(family_delta,3);
        family_clock_delta_m=(broadcast_dt_cnv1-broadcast_dt_generic)*CLIGHT;

        final_ok=satpos(tx_generic,target,record->sat,EPHOPT_B2b,nav,
                        rs_generic,dts_generic,&var_generic,&svh_generic)&&
                 satpos(tx_cnv1,target,record->sat,EPHOPT_B2b,nav,
                        rs_cnv1,dts_cnv1,&var_cnv1,&svh_cnv1);
        if (final_ok) {
            for (j=0;j<3;j++) final_delta[j]=rs_cnv1[j]-rs_generic[j];
            final_pos_delta_mm=norm(final_delta,3)*1E3;
            final_clock_delta_mm=(dts_cnv1[0]-dts_generic[0])*CLIGHT*1E3;
            if (rr_ok&&
                (rho_generic=geodist(rs_generic,rr,los_generic))>0.0&&
                (rho_cnv1=geodist(rs_cnv1,rr,los_cnv1))>0.0) {
                range_delta_mm=(rho_cnv1-rho_generic)*1E3;
                model_delta_mm=range_delta_mm-final_clock_delta_mm;
                residual_delta_mm=-model_delta_mm;
            }
            stats.final_pairs++;
            stats.sum_final_pos_delta_mm+=final_pos_delta_mm;
            stats.sum_abs_residual_delta_mm+=fabs(residual_delta_mm);
            update_max(final_pos_delta_mm,&stats.max_final_pos_delta_mm);
            update_max(fabs(residual_delta_mm),
                       &stats.max_abs_residual_delta_mm);

            if (nav->ne>0&&peph2pos(tx_cnv1,record->sat,nav,0,precise_rs,
                                    precise_dts,&precise_var)) {
                for (j=0;j<3;j++) {
                    precise_delta[j]=rs_cnv1[j]-precise_rs[j];
                    broadcast_precise_delta[j]=broadcast_cnv1[j]-precise_rs[j];
                    applied_delta[j]=rs_cnv1[j]-broadcast_cnv1[j];
                }
                precise_pos_m=norm(precise_delta,3);
                precise_clock_m=(dts_cnv1[0]-precise_dts[0])*CLIGHT;
                broadcast_precise_pos_m=norm(broadcast_precise_delta,3);
                broadcast_precise_clock_m=
                    (broadcast_dt_cnv1-precise_dts[0])*CLIGHT;
                (void)ecef_delta_to_b2b_rac(rs_cnv1,precise_delta,
                                             precise_rac);
                (void)ecef_delta_to_b2b_rac(rs_cnv1,
                                             broadcast_precise_delta,
                                             broadcast_precise_rac);
                (void)ecef_delta_to_b2b_rac(rs_cnv1,applied_delta,
                                             applied_rac);
                precise_ok=1;
                stats.precise_pairs++;
                stats.sum_precise_pos_sq+=precise_pos_m*precise_pos_m;
                update_max(precise_pos_m,&stats.max_precise_pos_m);
                if (stats.precise_clock_n<MAXOBS) {
                    stats.precise_clock_m[stats.precise_clock_n++]=
                        precise_clock_m;
                }
            }
        }

        printf("CLOCK_SEED_AB sat=%s code=%s(%d) generic=%s exact=%s "
               "iodn=%d dt_generic_s=%.12e dt_cnv1_s=%.12e "
               "delta_dt_ns=%.6f tx_delta_ns=%.6f "
               "family_pos_delta_m=%.4f family_clock_delta_m=%.4f\n",
               id,code_name(record->code[slot],code,sizeof(code)),
               record->code[slot],eph_msg_type(generic),eph_msg_type(exact),
               b2b->iodn,dt_generic,dt_cnv1,delta_dt_ns,tx_delta_ns,
               family_pos_delta_m,family_clock_delta_m);
        printf("CLOCK_SEED_FINAL sat=%s ok=%d pos_delta_mm=%.6f "
               "clock_delta_mm=%.6f range_delta_mm=%.6f "
               "model_delta_mm=%.6f residual_delta_mm=%.6f\n",
               id,final_ok,final_pos_delta_mm,final_clock_delta_mm,
               range_delta_mm,model_delta_mm,residual_delta_mm);
        if (nav->ne>0) {
            printf("B2B_PRECISE sat=%s ok=%d "
                   "broadcast_pos_3d_m=%.4f broadcast_rac_m=%.4f/%.4f/%.4f "
                   "applied_rac_m=%.4f/%.4f/%.4f corrected_pos_3d_m=%.4f "
                   "corrected_rac_m=%.4f/%.4f/%.4f "
                   "broadcast_clock_delta_m=%.4f corrected_clock_delta_m=%.4f\n",
                   id,precise_ok,broadcast_precise_pos_m,
                   broadcast_precise_rac[0],broadcast_precise_rac[1],
                   broadcast_precise_rac[2],applied_rac[0],applied_rac[1],
                   applied_rac[2],precise_pos_m,precise_rac[0],precise_rac[1],
                   precise_rac[2],broadcast_precise_clock_m,precise_clock_m);
        }
    }
    printf("CLOCK_SEED_AB_SUMMARY observed=%d clock_pairs=%d final_pairs=%d "
           "mean_abs_delta_dt_ns=%.6f max_abs_delta_dt_ns=%.6f "
           "mean_abs_tx_delta_ns=%.6f max_abs_tx_delta_ns=%.6f "
           "mean_final_pos_delta_mm=%.6f max_final_pos_delta_mm=%.6f "
           "mean_abs_residual_delta_mm=%.6f "
           "max_abs_residual_delta_mm=%.6f\n",stats.observed,
           stats.clock_pairs,stats.final_pairs,
           stats.clock_pairs?stats.sum_abs_delta_dt_ns/stats.clock_pairs:0.0,
           stats.max_abs_delta_dt_ns,
           stats.clock_pairs?stats.sum_abs_tx_delta_ns/stats.clock_pairs:0.0,
           stats.max_abs_tx_delta_ns,
           stats.final_pairs?stats.sum_final_pos_delta_mm/stats.final_pairs:0.0,
           stats.max_final_pos_delta_mm,
           stats.final_pairs?stats.sum_abs_residual_delta_mm/stats.final_pairs:0.0,
           stats.max_abs_residual_delta_mm);
    if (stats.precise_pairs>0) {
        double mean=0.0,rms_centered=0.0,min_clock=0.0,max_clock=0.0;

        for (i=0;i<stats.precise_clock_n;i++) mean+=stats.precise_clock_m[i];
        mean/=stats.precise_clock_n;
        min_clock=max_clock=stats.precise_clock_m[0];
        for (i=0;i<stats.precise_clock_n;i++) {
            double centered=stats.precise_clock_m[i]-mean;

            rms_centered+=centered*centered;
            if (stats.precise_clock_m[i]<min_clock) {
                min_clock=stats.precise_clock_m[i];
            }
            if (stats.precise_clock_m[i]>max_clock) {
                max_clock=stats.precise_clock_m[i];
            }
        }
        rms_centered=sqrt(rms_centered/stats.precise_clock_n);
        printf("B2B_PRECISE_SUMMARY pairs=%d pos_3d_rms_m=%.4f "
               "pos_3d_max_m=%.4f clock_mean_m=%.4f "
               "clock_centered_rms_m=%.4f clock_peak_to_peak_m=%.4f\n",
               stats.precise_pairs,
               sqrt(stats.sum_precise_pos_sq/stats.precise_pairs),
               stats.max_precise_pos_m,mean,rms_centered,max_clock-min_clock);
    }
}

static void print_sat_diag(const obs_t *obs, const nav_t *nav,
                           gtime_t target, int sat)
{
    const obsd_t *obs_record=find_obs_record(obs,target,sat);
    const B2bssr_t *b2b=nav->B2bssr+sat;
    double orbit_age=0.0,clock_age=0.0,rs[6],dts[2],var=0.0;
    int orbit_ok,clock_ok,combined_ok,selected,svh=-1,satpos_ok;
    char id[16],t0[32],t1[32],t2[32];

    satno2id(sat,id);
    time_text(b2b->t0[0],t0,sizeof(t0));
    time_text(b2b->t0[1],t1,sizeof(t1));
    time_text(b2b->t0[2],t2,sizeof(t2));

    printf("SAT %s sat=%d\n",id,sat);
    print_obs_codes(obs_record,nav);
    print_model_sources(obs_record,nav,target,sat);

    orbit_ok=b2b_orbit_age_valid(target,b2b,&orbit_age);
    clock_ok=b2b_clock_age_valid(target,b2b,&clock_age);
    combined_ok=b2b_orbit_clock_ready(target,b2b,&orbit_age,&clock_age);
    printf("  B2B_READY orbit=%d clock=%d combined=%d orbit_age=%.0f "
           "clock_age=%.0f iodn=%d iodcorr=%u/%u iodssr=%d/%d ura=%d "
           "t0=%s/%s/%s\n",
           orbit_ok,clock_ok,combined_ok,orbit_age,clock_age,b2b->iodn,
           (unsigned int)b2b->iodcorr[0],(unsigned int)b2b->iodcorr[1],
           b2b->iodssr[0],b2b->iodssr[2],b2b->ura,t0,t1,t2);

    print_cbias_valid(b2b);
    (void)print_eph_candidates(nav,target,sat,b2b->iodn,&selected);
    if (selected>=0) {
        const eph_t *eph=nav->eph+selected;
        char toe[32];

        time_text(eph->toe,toe,sizeof(toe));
        printf("  LOCAL_B2B_EPH_MATCH type=%s iode=%d iodc=%d toe=%s\n",
               eph_msg_type(eph),eph->iode,eph->iodc,toe);
    }
    else {
        printf("  LOCAL_B2B_EPH_MATCH none\n");
    }

    memset(rs,0,sizeof(rs));
    memset(dts,0,sizeof(dts));
    satpos_ok=satpos(target,target,sat,EPHOPT_B2b,nav,rs,dts,&var,&svh);
    printf("  SATPOS_B2B ok=%d svh=%d var=%.3f dts=%.12e %.12e\n",
           satpos_ok,svh,var,dts[0],dts[1]);
}

static void print_spp_diag(const obs_t *obs, const nav_t *nav, gtime_t target)
{
    prcopt_t opt=prcopt_default;
    sol_t sol={0};
    ssat_t *ssat;
    double *azel;
    char msg[128],id[16],name[16];
    int i,stat;

    if (!(azel=(double *)calloc((size_t)obs->n*2,sizeof(*azel)))||
        !(ssat=(ssat_t *)calloc(MAXSAT,sizeof(*ssat)))) {
        fprintf(stderr,"failed to allocate SPP diagnostic buffers\n");
        free(azel);
        free(ssat);
        return;
    }
    opt.mode=PMODE_SINGLE;
    opt.navsys=SYS_GPS|SYS_CMP;
    opt.nf=2;
    opt.elmin=10.0*D2R;
    opt.posopt[4]=0;
    stat=pntpos(obs->data,obs->n,nav,&opt,&sol,azel,ssat,msg);
    printf("SPP_RESULT time=%s ok=%d stat=%d ns=%d msg=%s "
           "xyz=%.4f/%.4f/%.4f\n",time_str(target,0),stat,sol.stat,sol.ns,
           msg,sol.rr[0],sol.rr[1],sol.rr[2]);
    for (i=0;i<obs->n;i++) {
        const obsd_t *record=obs->data+i;
        const eph_t *satpos_eph;
        const eph_t *tgd_eph;

        if (!ssat[record->sat-1].vs) continue;
        satno2id(record->sat,id);
        satpos_eph=current_generic_eph(nav,target,record->sat);
        tgd_eph=current_gettgd_eph(nav,target,record->sat,
                                   record->code[0]==CODE_L2I?0:2);
        printf("SPP_RESIDUAL sat=%s code=%s(%d) P=%.3f satpos=%s tgd=%s "
               "res=%.3f azel=%.3f/%.3f\n",id,
               code_name(record->code[0],name,sizeof(name)),record->code[0],
               record->P[0],eph_msg_type(satpos_eph),eph_msg_type(tgd_eph),
               ssat[record->sat-1].resp[0],azel[i*2]*R2D,
               azel[i*2+1]*R2D);
    }
    free(azel);
    free(ssat);
}

/* Some analysis-center CLK files found in the real-data set have the first
 * header fields shifted right by one column, so fixed-column readrnxh() does
 * not recognize type C. The AS records themselves are standard fixed-width
 * records. Keep this tolerance local to the diagnostic tool and never alter
 * the production RINEX reader. */
static int read_precise_clock_records(const char *path, nav_t *nav)
{
    pclk_t *pclk;
    gtime_t time;
    double ep[6],clock,std;
    FILE *fp;
    char buff[1024],record_type[8],satid[8];
    int sat,before,nvalue;

    if (!path||!nav||(fp=fopen(path,"r"))==NULL) return 0;
    before=nav->nc;
    while (fgets(buff,sizeof(buff),fp)) {
        std=0.0;
        if (sscanf(buff,"%7s %7s %lf %lf %lf %lf %lf %lf %d %lf %lf",
                   record_type,satid,ep,ep+1,ep+2,ep+3,ep+4,ep+5,&nvalue,
                   &clock,&std)<10||strcmp(record_type,"AS")||nvalue<1) {
            continue;
        }
        if (!(sat=satid2no(satid))) continue;
        time=epoch2time(ep);
        if (nav->nc<=0||
            fabs(timediff(time,nav->pclk[nav->nc-1].time))>1E-9) {
            if (nav->nc>=nav->ncmax) {
                nav->ncmax+=1024;
                pclk=(pclk_t *)realloc(nav->pclk,
                                      sizeof(*nav->pclk)*(size_t)nav->ncmax);
                if (!pclk) {
                    fclose(fp);
                    return 0;
                }
                nav->pclk=pclk;
            }
            memset(nav->pclk+nav->nc,0,sizeof(*nav->pclk));
            nav->pclk[nav->nc].time=time;
            nav->nc++;
        }
        nav->pclk[nav->nc-1].clk[sat-1][0]=clock;
        nav->pclk[nav->nc-1].std[sat-1][0]=(float)std;
    }
    fclose(fp);
    return nav->nc>before;
}

int main(int argc, char **argv)
{
    obs_t *obs=NULL;
    nav_t *nav=NULL;
    sta_t *sta=NULL;
    char **nav_paths=NULL;
    const char *sp3_path=NULL,*clk_path=NULL;
    b2b_replay_stats_t stats;
    gate_system_stats_t gps_stats,bds_stats;
    B2bssr_t zero_b2b={0};
    gtime_t target={0};
    char target_text[32],last_text[32],future_text[32];
    int i,nnav=0,format,index_zero_unused,future_held,frequency_mapping_ok;
    int gate_pass;

    if (argc<7) {
        fprintf(stderr,"Usage: %s <sino|unicore> <yyyy/mm/dd> <hh:mm:ss> "
                "<obs_rnx> <b2b_raw> <nav_rnx> [nav_rnx...] "
                "[--sp3 precise.sp3] [--clk precise.clk]\n",argv[0]);
        return 1;
    }
    if ((format=diagnostic_format(argv[1]))<0) {
        fprintf(stderr,"unsupported B2b format: %s\n",argv[1]);
        return 1;
    }
    if (!diagnostic_time(argv[2],argv[3],&target)) {
        fprintf(stderr,"invalid diagnostic time: %s %s\n",argv[2],argv[3]);
        return 1;
    }
    if (!(nav_paths=(char **)calloc((size_t)argc,sizeof(*nav_paths)))) {
        fprintf(stderr,"navigation path allocation failed\n");
        return 1;
    }
    for (i=6;i<argc;i++) {
        if (!strcmp(argv[i],"--sp3")||!strcmp(argv[i],"--clk")) {
            const char **destination=!strcmp(argv[i],"--sp3")?
                                     &sp3_path:&clk_path;

            if (++i>=argc) {
                fprintf(stderr,"missing path after %s\n",argv[i-1]);
                free(nav_paths);
                return 1;
            }
            *destination=argv[i];
            continue;
        }
        nav_paths[nnav++]=argv[i];
    }
    if (nnav<=0) {
        fprintf(stderr,"at least one navigation RINEX is required\n");
        free(nav_paths);
        return 1;
    }
    if (!(obs=(obs_t *)calloc(1,sizeof(*obs)))||
        !(nav=(nav_t *)calloc(1,sizeof(*nav)))||
        !(sta=(sta_t *)calloc(1,sizeof(*sta)))) {
        fprintf(stderr,"memory allocation failed\n");
        free(nav_paths);
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }
    if (!read_inputs(argv[4],nnav,nav_paths,target,obs,nav,sta)) {
        fprintf(stderr,"input read failed or empty\n");
        free(nav_paths);
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }
    free(nav_paths);
    nav_paths=NULL;
    if (sp3_path) {
        readsp3(sp3_path,nav,0);
        if (nav->ne<=0) {
            fprintf(stderr,"failed to read precise orbit SP3: %s\n",sp3_path);
            freeobs(obs);
            freenav(nav,0x7F);
            free(obs);
            free(nav);
            free(sta);
            return 1;
        }
    }
    if (clk_path&&(!readrnxc(clk_path,nav)||nav->nc<=0)) {
        fprintf(stderr,"PREC_CLK_HEADER_FALLBACK path=%s\n",clk_path);
        free(nav->pclk);
        nav->pclk=NULL;
        nav->nc=nav->ncmax=0;
        if (!read_precise_clock_records(clk_path,nav)) {
            fprintf(stderr,"failed to read precise clock records: %s\n",
                    clk_path);
            freeobs(obs);
            freenav(nav,0x7F);
            free(obs);
            free(nav);
            free(sta);
            return 1;
        }
    }
    if (!replay_b2b_until(argv[5],format,target,nav,&stats)) {
        freeobs(obs);
        freenav(nav,0x7F);
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }

    time_text(target,target_text,sizeof(target_text));
    time_text(stats.last_time,last_text,sizeof(last_text));
    time_text(stats.first_future_time,future_text,sizeof(future_text));

    printf("TARGET_TIME %s GPST\n",target_text);
    printf("B2B_FORMAT %s (%d)\n",diagnostic_format_name(format),format);
    printf("OBS_OPT %s\n",obs_opt);
    printf("NAV_OPT %s\n",nav_opt);
    printf("OBS_RECORDS_AT_TARGET %d\n",obs->n);
    printf("NAV_COUNTS eph=%d geph=%d seph=%d\n",nav->n,nav->ng,nav->ns);
    printf("NAV_CNV1_COUNTS preuniq_eph=%d preuniq_cnv1=%d postuniq_cnv1=%d\n",
           nav_eph_preuniq,nav_cnv1_preuniq,nav_cnv1_postuniq);
    printf("B2B_REPLAY last=%s first_future=%s updates=%ld errors=%ld\n",
           last_text,future_text,stats.nav_updates,stats.errors);
    printf("B2B_FRAMES mask=%ld orbit=%ld code_bias=%ld clock=%ld\n",
           stats.frames[0],stats.frames[1],stats.frames[2],stats.frames[3]);
    frequency_mapping_ok=test_bds_frequency_mapping();

    collect_gate_system_stats(obs,nav,target,SYS_GPS,&gps_stats);
    collect_gate_system_stats(obs,nav,target,SYS_CMP,&bds_stats);
    print_gate_system_stats("GPS",&gps_stats);
    print_gate_system_stats("BDS",&bds_stats);
    print_clock_seed_ab(obs,nav,target,sta->pos);

    for (i=0;i<(int)(sizeof(bds_prns)/sizeof(bds_prns[0]));i++) {
        int sat=satno(SYS_CMP,bds_prns[i]);

        if (sat<=0) continue;
        print_sat_diag(obs,nav,target,sat);
    }
    for (i=0;i<(int)(sizeof(gps_prns)/sizeof(gps_prns[0]));i++) {
        int sat=satno(SYS_GPS,gps_prns[i]);

        if (sat<=0) continue;
        print_sat_diag(obs,nav,target,sat);
    }
    print_spp_diag(obs,nav,target);

    index_zero_unused=!memcmp(nav->B2bssr,&zero_b2b,sizeof(zero_b2b));
    future_held=stats.first_future_time.time&&
                timediff(stats.first_future_time,target)>DTTOL;
    gate_pass=frequency_mapping_ok&&stats.errors==0&&index_zero_unused&&
              future_held&&
              bds_stats.satpos_ok>0&&
              bds_stats.cbias_f1_ready>0&&bds_stats.cbias_f2_ready>0;
    printf("GATE_PROFILE BDS_ONLY\n");
    printf("GATE_INDEX_ZERO_UNUSED %d\n",index_zero_unused);
    printf("GATE_FUTURE_FRAME_HELD %d\n",future_held);
    printf("GATE_RESULT %s\n",gate_pass?"PASS":"FAIL");

    freeobs(obs);
    freenav(nav,0x7F);
    free(obs);
    free(nav);
    free(sta);
    return gate_pass?0:2;
}
