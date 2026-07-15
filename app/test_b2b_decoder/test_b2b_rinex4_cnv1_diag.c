#include "rtklib.h"
#include "rcv/unicore.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308

static const char *obs_opt="-SYS=C -CL2I -CL7I";
static const char *nav_opt="-SYS=C";
static const int bds_prns[]={23,24,25,32,33,38,39,41};
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

static gtime_t diagnostic_time(void)
{
    double ep[]={2024,12,25,0,30,0.0};

    return epoch2time(ep);
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

static int frame_index(const raw_t *raw)
{
    int type=raw->buff[4]|(raw->buff[5]<<8);

    if (type==UNICORE_MSG_B2B_INFO1) return 0;
    if (type==UNICORE_MSG_B2B_INFO2) return 1;
    if (type==UNICORE_MSG_B2B_INFO3) return 2;
    if (type==UNICORE_MSG_B2B_INFO4) return 3;
    return -1;
}

static int replay_b2b_until(const char *path, gtime_t target, nav_t *nav,
                            b2b_replay_stats_t *stats)
{
    raw_t raw={0};
    FILE *fp;
    int ret,index;

    memset(stats,0,sizeof(*stats));
    raw.format=STRFMT_UNICORE;
    if (!(fp=fopen(path,"rb"))) {
        fprintf(stderr,"failed to open B2bBin: %s\n",path);
        return 0;
    }
    if (!init_unicore_b2b(&raw)) {
        fprintf(stderr,"failed to initialize Unicore B2b raw context\n");
        fclose(fp);
        return 0;
    }
    for (;;) {
        ret=input_unicoref(&raw,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;
        if (timediff(raw.time,target)>DTTOL) {
            stats->first_future_time=raw.time;
            break;
        }
        if ((index=frame_index(&raw))>=0) stats->frames[index]++;
        stats->last_time=raw.time;
        stats->nav_updates+=b2b_update_nav_from_raw(nav,&raw);
    }
    free_unicore_b2b(&raw);
    fclose(fp);
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
    for (i=0;i<nnav;i++) {
        if (readrnxt(nav_paths[i],1,zero,zero,0.0,nav_opt,obs,nav,NULL)<0) {
            fprintf(stderr,"failed to read navigation RINEX: %s\n",
                    nav_paths[i]);
            return 0;
        }
    }
    nav_eph_preuniq=nav->n;
    nav_cnv1_preuniq=count_cnv1_eph(nav);
    sortobs(obs);
    uniqnav(nav);
    nav_cnv1_postuniq=count_cnv1_eph(nav);
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
    return eph_is_cnv1(eph)?"CNV1":"D1/D2";
}

/* Mirror the current generic SYS_CMP seleph() rule without changing it. */
static const eph_t *current_generic_eph(const nav_t *nav, gtime_t target,
                                        int sat)
{
    const eph_t *selected=NULL;
    double age,best_age=MAXDTOE_CMP+2.0;
    int i,cur_cnv1,best_cnv1=0;

    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;

        if (eph->sat!=sat) continue;
        age=fabs(timediff(eph->toe,target));
        if (age>MAXDTOE_CMP+1.0) continue;
        cur_cnv1=eph_is_cnv1(eph);
        if (!selected||(best_cnv1&&!cur_cnv1)||
            (best_cnv1==cur_cnv1&&age<=best_age)) {
            selected=eph;
            best_age=age;
            best_cnv1=cur_cnv1;
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
    int type=obs&&obs->code[0]!=CODE_L2I?2:0;
    const eph_t *tgd_eph=current_gettgd_eph(nav,target,sat,type);
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
    else printf("  PRANGE_TGD_EPH none\n");
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
    double best_age=MAXDTOE_CMP+2.0;
    int i,total=0,healthy=0,iod_match=0,age_match=0,best_cnv1=0;

    *selected=-1;
    for (i=0;i<nav->n;i++) {
        const eph_t *eph=nav->eph+i;
        double age;
        int match,age_ok,cur_cnv1;

        if (eph->sat!=sat) continue;
        age=fabs(timediff(eph->toe,target));
        match=eph->iodc==iodn;
        age_ok=age<=MAXDTOE_CMP+1.0;
        if (!match&&!age_ok) continue;

        total++;
        if (!eph->svh) healthy++;
        if (match) iod_match++;
        if (match&&age_ok&&!eph->svh) {
            age_match++;
            cur_cnv1=eph_is_cnv1(eph);
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
    int i,prn,stat;

    if (!(azel=(double *)calloc((size_t)obs->n*2,sizeof(*azel)))||
        !(ssat=(ssat_t *)calloc(MAXSAT,sizeof(*ssat)))) {
        fprintf(stderr,"failed to allocate SPP diagnostic buffers\n");
        free(azel);
        free(ssat);
        return;
    }
    opt.mode=PMODE_SINGLE;
    opt.navsys=SYS_CMP;
    opt.nf=1;
    opt.elmin=10.0*D2R;
    opt.posopt[4]=0;
    for (prn=1;prn<=18;prn++) {
        int sat=satno(SYS_CMP,prn);
        if (sat>0) opt.exsats[sat-1]=1;
    }
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

int main(int argc, char **argv)
{
    obs_t *obs;
    nav_t *nav;
    sta_t *sta;
    b2b_replay_stats_t stats;
    gtime_t target=diagnostic_time();
    char target_text[32],last_text[32],future_text[32];
    int i;

    if (argc<4) {
        fprintf(stderr,"Usage: %s <obs_rnx> <b2b_bin> <nav_rnx> [nav_rnx...]\n",
                argv[0]);
        return 1;
    }
    if (!(obs=(obs_t *)calloc(1,sizeof(*obs)))||
        !(nav=(nav_t *)calloc(1,sizeof(*nav)))||
        !(sta=(sta_t *)calloc(1,sizeof(*sta)))) {
        fprintf(stderr,"memory allocation failed\n");
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }
    if (!read_inputs(argv[1],argc-3,argv+3,target,obs,nav,sta)) {
        fprintf(stderr,"input read failed or empty\n");
        free(obs);
        free(nav);
        free(sta);
        return 1;
    }
    if (!replay_b2b_until(argv[2],target,nav,&stats)) {
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

    for (i=0;i<(int)(sizeof(bds_prns)/sizeof(bds_prns[0]));i++) {
        int sat=satno(SYS_CMP,bds_prns[i]);

        if (sat<=0) continue;
        print_sat_diag(obs,nav,target,sat);
    }
    print_spp_diag(obs,nav,target);

    freeobs(obs);
    freenav(nav,0x7F);
    free(obs);
    free(nav);
    free(sta);
    return 0;
}
