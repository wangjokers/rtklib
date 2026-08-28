/* Validate the frozen realtime BDS B1Cp/B2aP mapping with the real fixtures. */
#include "../../src/rtklib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OBSERVED_SATS 9
#define MAX_SAMPLES 1200

extern const char *msm_sig_cmp[32];

static const int observed_prns[OBSERVED_SATS]={
    23,24,25,31,32,33,34,38,41
};

typedef struct {
    gtime_t time;
    double p[2];
    double l[2];
    uint8_t code[2];
} sample_t;

typedef struct {
    long frames;
    long type1127;
    long id23_frames;
    long id31_frames;
    long crc_errors;
    long length_errors;
} frame_stats_t;

typedef struct {
    long seen_epochs;
    long slot0_l1p_codes;
    long slot1_l5p_codes;
    long l1p_epochs;
    long l5p_epochs;
    long dual_epochs;
    long matched_epochs;
    int cbias_l1p_valid;
    int cbias_l5p_valid;
    double first_l1p_bias;
    double first_l5p_bias;
    double first_match_tow;
    sample_t samples[MAX_SAMPLES];
    int nsamples;
} sat_stats_t;

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

static int observed_index(int sat)
{
    int i;

    for (i=0;i<OBSERVED_SATS;i++) {
        if (sat==satno(SYS_CMP,observed_prns[i])) return i;
    }
    return -1;
}

static uint32_t get_u3be(const uint8_t *p)
{
    return ((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|p[2];
}

static int scan_rtcm(const char *path, frame_stats_t *stats)
{
    uint8_t frame[1200];
    FILE *fp;
    int data,len,total,type,id;

    memset(stats,0,sizeof(*stats));
    if (!(fp=fopen(path,"rb"))) return 0;

    while ((data=fgetc(fp))!=EOF) {
        if (data!=0xD3) continue;
        frame[0]=(uint8_t)data;
        if (fread(frame+1,1,2,fp)!=2) {
            stats->length_errors++;
            break;
        }
        len=((frame[1]&0x03)<<8)|frame[2];
        total=3+len+3;
        if (total>(int)sizeof(frame)||
            fread(frame+3,1,(size_t)len+3,fp)!=(size_t)len+3) {
            stats->length_errors++;
            break;
        }
        stats->frames++;
        if (rtk_crc24q(frame,3+len)!=get_u3be(frame+3+len)) {
            stats->crc_errors++;
            continue;
        }
        type=(int)getbitu(frame,24,12);
        if (type!=1127) continue;
        stats->type1127++;
        /* MSM common header is 73 bits after the 24-bit transport header;
         * its 64-bit satellite mask is followed by the 32-bit signal mask. */
        for (id=1;id<=32;id++) {
            if (!getbitu(frame,24+73+64+id-1,1)) continue;
            if (id==23) stats->id23_frames++;
            if (id==31) stats->id31_frames++;
        }
    }
    fclose(fp);
    return 1;
}

static int decode_sino(const char *path, raw_t *raw, long *ret20)
{
    FILE *fp;
    int data,ret;

    *ret20=0;
    if (!init_raw(raw,STRFMT_SINO)||!(fp=fopen(path,"rb"))) return 0;
    while ((data=fgetc(fp))!=EOF) {
        ret=input_raw(raw,STRFMT_SINO,(uint8_t)data);
        if (ret==20) (*ret20)++;
    }
    fclose(fp);
    return 1;
}

static void collect_epoch(const rtcm_t *rtcm, sat_stats_t *stats,
                          long *slot2_writes)
{
    const obsd_t *obs;
    sat_stats_t *sat_stats;
    int i,k;

    for (i=0;i<rtcm->obs.n;i++) {
        obs=rtcm->obs.data+i;
        if ((k=observed_index(obs->sat))<0) continue;
        sat_stats=stats+k;

        sat_stats->seen_epochs++;
        if (obs->code[0]==CODE_L1P) sat_stats->slot0_l1p_codes++;
        if (obs->code[1]==CODE_L5P) sat_stats->slot1_l5p_codes++;

        if (obs->code[0]==CODE_L1P&&obs->P[0]!=0.0&&obs->L[0]!=0.0) {
            sat_stats->l1p_epochs++;
        }
        if (obs->code[1]==CODE_L5P&&obs->P[1]!=0.0&&obs->L[1]!=0.0) {
            sat_stats->l5p_epochs++;
        }
        if (obs->code[0]==CODE_L1P&&obs->code[1]==CODE_L5P&&
            obs->P[0]!=0.0&&obs->L[0]!=0.0&&
            obs->P[1]!=0.0&&obs->L[1]!=0.0) {
            sample_t *sample;

            sat_stats->dual_epochs++;
            if (sat_stats->nsamples>=MAX_SAMPLES) continue;
            sample=sat_stats->samples+sat_stats->nsamples++;
            sample->time=obs->time;
            sample->p[0]=obs->P[0]; sample->p[1]=obs->P[1];
            sample->l[0]=obs->L[0]; sample->l[1]=obs->L[1];
            sample->code[0]=obs->code[0]; sample->code[1]=obs->code[1];
        }
        if (obs->code[2]!=CODE_NONE||obs->P[2]!=0.0||obs->L[2]!=0.0||
            obs->D[2]!=0.0||obs->SNR[2]!=0||obs->LLI[2]!=0) {
            (*slot2_writes)++;
        }
    }
}

static int decode_rtcm(const char *path, gtime_t seed, sat_stats_t *stats,
                       uint32_t *frames, uint32_t *type1127,
                       long *epochs, long *slot2_writes)
{
    rtcm_t *rtcm;
    FILE *fp;
    int data,ret,type,i;

    *frames=*type1127=0;
    *epochs=*slot2_writes=0;
    if (!(rtcm=(rtcm_t *)calloc(1,sizeof(*rtcm)))) return 0;
    if (!init_rtcm(rtcm)||!(fp=fopen(path,"rb"))) {
        free(rtcm);
        return 0;
    }
    rtcm->time=seed;
    while ((data=fgetc(fp))!=EOF) {
        ret=input_rtcm3(rtcm,(uint8_t)data);
        if (ret<=0) continue;
        type=(int)getbitu(rtcm->buff,24,12);
        if (ret==1&&type==1127) {
            (*epochs)++;
            collect_epoch(rtcm,stats,slot2_writes);
        }
    }
    fclose(fp);
    for (i=0;i<400;i++) *frames+=rtcm->nmsg3[i];
    *type1127=rtcm->nmsg3[127];
    free_rtcm(rtcm);
    free(rtcm);
    return 1;
}

static long count_trace(const char *path, const char *needle)
{
    FILE *fp;
    char line[4096];
    long count=0;

    if (!(fp=fopen(path,"r"))) return -1;
    while (fgets(line,sizeof(line),fp)) {
        if (strstr(line,needle)) count++;
    }
    fclose(fp);
    return count;
}

static int match_cbias(const raw_t *raw, sat_stats_t *stats)
{
    int i,j,matched_sats=0;

    for (i=0;i<OBSERVED_SATS;i++) {
        const int sat=satno(SYS_CMP,observed_prns[i]);
        const B2bssr_t *b2b=raw->nav.B2bssr+sat;
        sat_stats_t *s=stats+i;

        s->cbias_l1p_valid=b2b->cbias_valid[CODE_L1P]&&
                           isfinite(b2b->cbias[CODE_L1P]);
        s->cbias_l5p_valid=b2b->cbias_valid[CODE_L5P]&&
                           isfinite(b2b->cbias[CODE_L5P]);
        for (j=0;j<s->nsamples;j++) {
            double b1=0.0,b2=0.0,age1=0.0,age2=0.0;
            int mode1=-1,mode2=-1;

            if (!b2b_resolve_cbias(s->samples[j].time,b2b,SYS_CMP,
                                   s->samples[j].code[0],B2BXBIAS_OFF,
                                   &b1,&age1,&mode1)||
                !b2b_resolve_cbias(s->samples[j].time,b2b,SYS_CMP,
                                   s->samples[j].code[1],B2BXBIAS_OFF,
                                   &b2,&age2,&mode2)) continue;
            if (mode1!=B2BXBIAS_OFF||mode2!=B2BXBIAS_OFF) continue;
            if (s->matched_epochs++==0) {
                int week;
                s->first_l1p_bias=b1;
                s->first_l5p_bias=b2;
                s->first_match_tow=time2gpst(s->samples[j].time,&week);
            }
        }
        if (s->matched_epochs>0) matched_sats++;
    }
    return matched_sats;
}

static int write_summary(const char *path, const frame_stats_t *source,
                         uint32_t decoded_frames, uint32_t decoded1127,
                         long epochs, long slot2_writes, long unknown23,
                         long unknown31, long sino_ret20,
                         const sat_stats_t *stats, int mapping_ok,
                         int priority_ok, int matched_sats)
{
    FILE *fp;
    int i,all_dual=1,all_bias=1,pass;

    for (i=0;i<OBSERVED_SATS;i++) {
        all_dual&=stats[i].dual_epochs>0;
        all_bias&=stats[i].cbias_l1p_valid&&stats[i].cbias_l5p_valid;
    }
    pass=source->frames==4834&&source->type1127==1200&&
         source->id23_frames==1200&&source->id31_frames==1200&&
         source->crc_errors==0&&source->length_errors==0&&
         decoded_frames==4834&&decoded1127==1200&&epochs==600&&
         unknown23==0&&unknown31==0&&mapping_ok&&priority_ok&&
         all_dual&&all_bias&&matched_sats==OBSERVED_SATS&&slot2_writes==0;

    if (!(fp=fopen(path,"w"))) return 0;
    fprintf(fp,"{\n  \"status\": \"%s\",\n",pass?"PASS":"FAIL");
    fprintf(fp,"  \"source\": {\"frames\": %ld, \"type1127\": %ld, "
               "\"id23_frames\": %ld, \"id31_frames\": %ld, "
               "\"crc24q_errors\": %ld, \"length_errors\": %ld},\n",
            source->frames,source->type1127,source->id23_frames,
            source->id31_frames,source->crc_errors,source->length_errors);
    fprintf(fp,"  \"decoder\": {\"frames\": %u, \"type1127\": %u, "
               "\"epochs\": %ld, \"unknown_id23\": %ld, "
               "\"unknown_id31\": %ld, \"slot2_writes\": %ld},\n",
            decoded_frames,decoded1127,epochs,unknown23,unknown31,slot2_writes);
    fprintf(fp,"  \"mapping\": {\"id31_code\": %d, \"id31_slot\": %d, "
               "\"id23_code\": %d, \"id23_slot\": %d, "
               "\"priority_ok\": %s},\n",obs2code(msm_sig_cmp[30]),
            code2idx(SYS_CMP,obs2code(msm_sig_cmp[30])),
            obs2code(msm_sig_cmp[22]),
            code2idx(SYS_CMP,obs2code(msm_sig_cmp[22])),
            priority_ok?"true":"false");
    fprintf(fp,"  \"sino_ret20\": %ld,\n  \"satellites\": {",sino_ret20);
    for (i=0;i<OBSERVED_SATS;i++) {
        fprintf(fp,"%s\"C%02d\": {\"seen_epochs\": %ld, "
                   "\"slot0_l1p_codes\": %ld, \"slot1_l5p_codes\": %ld, "
                   "\"l1p_epochs\": %ld, "
                   "\"l5p_epochs\": %ld, \"dual_epochs\": %ld, "
                   "\"cbias_l1p_valid\": %s, \"cbias_l5p_valid\": %s, "
                   "\"matched_epochs\": %ld, \"sample_tow\": %.3f, "
                   "\"l1p_bias_m\": %.3f, \"l5p_bias_m\": %.3f}",
                i?", ":"",observed_prns[i],stats[i].seen_epochs,
                stats[i].slot0_l1p_codes,stats[i].slot1_l5p_codes,
                stats[i].l1p_epochs,
                stats[i].l5p_epochs,stats[i].dual_epochs,
                stats[i].cbias_l1p_valid?"true":"false",
                stats[i].cbias_l5p_valid?"true":"false",
                stats[i].matched_epochs,stats[i].first_match_tow,
                stats[i].first_l1p_bias,stats[i].first_l5p_bias);
    }
    fprintf(fp,"}\n}\n");
    fclose(fp);
    return pass;
}

int main(int argc, char **argv)
{
    frame_stats_t source;
    sat_stats_t *stats;
    raw_t *raw;
    uint32_t decoded_frames,decoded1127;
    long epochs,slot2_writes,unknown23,unknown31,sino_ret20;
    int mapping_ok,priority_ok,matched_sats,pass;

    if (argc!=5) {
        fprintf(stderr,"usage: %s OBS_RTCM3 SINO SUMMARY TRACE\n",argv[0]);
        return 2;
    }
    if (!(stats=(sat_stats_t *)calloc(OBSERVED_SATS,sizeof(*stats)))||
        !(raw=(raw_t *)calloc(1,sizeof(*raw)))) {
        free(stats);
        free(raw);
        return 2;
    }
    if (!scan_rtcm(argv[1],&source)||!decode_sino(argv[2],raw,&sino_ret20)) {
        fprintf(stderr,"fixture open/decode failed\n");
        free(stats);
        free(raw);
        return 2;
    }

    traceopen(argv[4]);
    tracelevel(2);
    if (!decode_rtcm(argv[1],raw->time,stats,&decoded_frames,&decoded1127,
                     &epochs,&slot2_writes)) {
        traceclose();
        free_raw(raw);
        free(stats);
        free(raw);
        return 2;
    }
    traceclose();

    unknown23=count_trace(argv[4],"rtcm3 1127: unknown signal id=23");
    unknown31=count_trace(argv[4],"rtcm3 1127: unknown signal id=31");
    mapping_ok=!strcmp(msm_sig_cmp[30],"1P")&&
               !strcmp(msm_sig_cmp[22],"5P")&&
               obs2code(msm_sig_cmp[30])==CODE_L1P&&
               obs2code(msm_sig_cmp[22])==CODE_L5P&&
               code2idx(SYS_CMP,CODE_L1P)==0&&
               code2idx(SYS_CMP,CODE_L5P)==1;
    priority_ok=getcodepri(SYS_CMP,CODE_L1P,NULL)>0&&
                getcodepri(SYS_CMP,CODE_L5P,NULL)>0;
    matched_sats=match_cbias(raw,stats);
    pass=write_summary(argv[3],&source,decoded_frames,decoded1127,epochs,
                       slot2_writes,unknown23,unknown31,sino_ret20,stats,
                       mapping_ok,priority_ok,matched_sats);
    free_raw(raw);
    free(stats);
    free(raw);

    printf("test_rtcm1127_pcode: %s frames=%u type1127=%u epochs=%ld "
           "unknown23=%ld unknown31=%ld slot2=%ld matched_sats=%d\n",
           pass?"PASS":"FAIL",decoded_frames,decoded1127,epochs,unknown23,
           unknown31,slot2_writes,matched_sats);
    return pass?0:1;
}
