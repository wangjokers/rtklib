#include "rtklib.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SINO_HEADER_LEN       28
#define SINO_MESSAGE72_LEN   216
#define SINO_MSG_BD3EPH       72
#define SINO_MSG_B2B        1697
#define SINO_TYPE_POS (SINO_HEADER_LEN*8+44)
#define SINO_P2_34 5.820766091346740E-11

static const int sample_prns[]={23,31,41};
static const double sample_mn_tgd[][4]={
    { 2.299202606082E-08,-2.270098775625E-09,
     -9.313225746155E-10,-2.735760062933E-09},
    {-1.571606844664E-09,-5.180481821299E-09,
     -2.328306436539E-10,-2.561137080193E-09},
    {-2.008164301515E-08, 5.820766091347E-09,
     -1.746229827404E-10,-2.735760062933E-09}
};

typedef struct {
    long frames;
    long message72;
    long message1697;
    long crc_errors;
    long length_errors;
} frame_stats_t;

typedef struct {
    long ret2;
    long ret20_type2;
    long decode_errors;
    long wrong_sat_writes;
    long field_errors;
    long time_backwards;
    long b1cp_tgd_missing;
    long finite_message72;
    long layout_errors;
    long mn_sample_matches;
    int prn_count;
    int observed_prn_count;
    int iod_match_prns;
    int iod_match_pairs;
    uint8_t seen_prn[MAXPRNCMP+1];
    uint8_t seen_eph_iod[MAXPRNCMP+1][256];
    uint8_t seen_b2b_iod[MAXPRNCMP+1][1024];
    double sample_tgd[3][4];
} decode_stats_t;

extern const B2bssr_t *sino_b2b_source_product(const raw_t *raw, int prn6,
                                                int sat);

static uint16_t get_u2le(const uint8_t *p)
{
    return (uint16_t)p[0]|((uint16_t)p[1]<<8);
}

static uint32_t get_u4le(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|
           ((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}

static int32_t get_i4le(const uint8_t *p)
{
    return (int32_t)get_u4le(p);
}

static uint64_t get_u8le(const uint8_t *p)
{
    return (uint64_t)p[0]|((uint64_t)p[1]<<8)|
           ((uint64_t)p[2]<<16)|((uint64_t)p[3]<<24)|
           ((uint64_t)p[4]<<32)|((uint64_t)p[5]<<40)|
           ((uint64_t)p[6]<<48)|((uint64_t)p[7]<<56);
}

static double get_r8le(const uint8_t *p)
{
    uint64_t bits=get_u8le(p);
    double value;

    memcpy(&value,&bits,sizeof(value));
    return value;
}

static int sample_index(int prn)
{
    int i;

    for (i=0;i<(int)(sizeof(sample_prns)/sizeof(sample_prns[0]));i++) {
        if (prn==sample_prns[i]) return i;
    }
    return -1;
}

static int scan_frames(const char *path, frame_stats_t *stats)
{
    uint8_t frame[MAXRAWLEN];
    FILE *fp;
    size_t n;
    int payload_len,total;

    if (!stats||(fp=fopen(path,"rb"))==NULL) return 0;
    memset(stats,0,sizeof(*stats));

    for (;;) {
        n=fread(frame,1,SINO_HEADER_LEN,fp);
        if (n==0&&feof(fp)) break;
        if (n!=SINO_HEADER_LEN) {
            stats->length_errors++;
            break;
        }
        if (frame[0]!=0xAA||frame[1]!=0x44||frame[2]!=0x12||
            frame[3]!=SINO_HEADER_LEN) {
            stats->length_errors++;
            break;
        }
        payload_len=(int)get_u2le(frame+8);
        total=SINO_HEADER_LEN+payload_len+4;
        if (payload_len<0||total>MAXRAWLEN||
            fread(frame+SINO_HEADER_LEN,1,(size_t)payload_len+4,fp)!=
            (size_t)payload_len+4) {
            stats->length_errors++;
            break;
        }
        if (rtk_crc32(frame,total-4)!=get_u4le(frame+total-4)) {
            stats->crc_errors++;
        }
        stats->frames++;
        if (get_u2le(frame+4)==SINO_MSG_BD3EPH) {
            stats->message72++;
            if (payload_len!=SINO_MESSAGE72_LEN) stats->length_errors++;
        }
        else if (get_u2le(frame+4)==SINO_MSG_B2B) {
            stats->message1697++;
        }
    }
    fclose(fp);
    return 1;
}

static int b2b_type(const raw_t *raw)
{
    if (!raw||SINO_TYPE_POS+6>raw->len*8) return -1;
    return (int)getbitu(raw->buff,SINO_TYPE_POS,6);
}

static void collect_type2_iod(const raw_t *raw, decode_stats_t *stats)
{
    const B2bssr_t *ssr;
    int prn,prn6,sat;

    for (prn6=0;prn6<64;prn6++) {
        for (prn=MINPRNCMP;prn<=MAXPRNCMP;prn++) {
            sat=satno(SYS_CMP,prn);
            if (!sat||(ssr=sino_b2b_source_product(raw,prn6,sat))==NULL||
                !ssr->t0[0].time||ssr->iodn<0||ssr->iodn>1023) continue;
            stats->seen_b2b_iod[prn][ssr->iodn]=1;
        }
    }
}

static int valid_message72_eph(const raw_t *raw, int sat, int prn)
{
    const eph_t *eph=raw->nav.eph+sat-1;
    double toe_age,toc_age;

    toe_age=fabs(timediff(raw->time,eph->toe));
    toc_age=fabs(timediff(raw->time,eph->toc));
    return eph->sat==sat&&eph->code==EPHCODE_BDS_CNV1&&raw->ephset==0&&
           eph->week>0&&eph->toes>=0.0&&eph->toes<604800.0&&
           toe_age<172800.0&&toc_age<172800.0&&
           eph->iode>=0&&eph->iode<=255&&eph->iodc>=0&&eph->iodc<=255&&
           eph->svh==0&&eph->A>2.0E7&&eph->A<5.0E7&&
           eph->e>=0.0&&eph->e<1.0&&isfinite(eph->Adot)&&
           isfinite(eph->ndot)&&isfinite(eph->tgd[1])&&
           isfinite(eph->tgd[2])&&isfinite(eph->tgd[3])&&
           isfinite(eph->tgd[4])&&isfinite(eph->tgd[5])&&
           prn>=MINPRNCMP&&prn<=MAXPRNCMP;
}

static int replay_decoder(const char *path, decode_stats_t *stats)
{
    static const int observed_prns[]={23,24,25,31,32,33,34,38,41};
    raw_t *raw=NULL;
    eph_t *before=NULL;
    FILE *fp=NULL;
    gtime_t last72={0};
    int i,j,ret,sat,prn,ok=0;

    if (!stats||(raw=(raw_t *)calloc(1,sizeof(*raw)))==NULL||
        (before=(eph_t *)malloc(sizeof(eph_t)*MAXSAT*2))==NULL||
        (fp=fopen(path,"rb"))==NULL||!init_raw(raw,STRFMT_SINO)) goto done;
    memset(stats,0,sizeof(*stats));
    strcpy(raw->opt,"-EPHALL");

    for (;;) {
        memcpy(before,raw->nav.eph,sizeof(eph_t)*MAXSAT*2);
        ret=input_rawf(raw,STRFMT_SINO,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->decode_errors++;
            continue;
        }
        if (ret==20&&b2b_type(raw)==2) {
            stats->ret20_type2++;
            collect_type2_iod(raw,stats);
            continue;
        }
        if (ret!=2) continue;

        stats->ret2++;
        sat=raw->ephsat;
        if (satsys(sat,&prn)!=SYS_CMP||sat<=0||sat>MAXSAT) {
            stats->field_errors++;
            continue;
        }
        if (last72.time&&timediff(raw->time,last72)<-DTTOL) {
            stats->time_backwards++;
        }
        last72=raw->time;
        for (i=0;i<MAXSAT*2;i++) {
            if (i==sat-1) continue;
            if (memcmp(before+i,raw->nav.eph+i,sizeof(eph_t))) {
                stats->wrong_sat_writes++;
                break;
            }
        }
        if (!valid_message72_eph(raw,sat,prn)) stats->field_errors++;
        if (!isfinite(raw->nav.eph[sat-1].tgd[2])) {
            stats->b1cp_tgd_missing++;
        }
        else stats->finite_message72++;
        {
            const uint8_t *p=raw->buff+SINO_HEADER_LEN;
            const eph_t *eph=raw->nav.eph+sat-1;

            if (fabs(eph->tgd[4]-get_i4le(p+184)*SINO_P2_34)>1E-20||
                fabs(eph->tgd[5]-get_i4le(p+188)*SINO_P2_34)>1E-20||
                fabs(eph->tgd[2]-get_r8le(p+192))>1E-20||
                fabs(eph->tgd[3]-get_r8le(p+200))>1E-20||
                fabs(eph->tgd[1]-get_r8le(p+208))>1E-20) {
                stats->layout_errors++;
            }
            if ((j=sample_index(prn))>=0) {
                stats->sample_tgd[j][0]=eph->tgd[2];
                stats->sample_tgd[j][1]=eph->tgd[3];
                stats->sample_tgd[j][2]=eph->tgd[4];
                stats->sample_tgd[j][3]=eph->tgd[5];
                if (fabs(eph->tgd[2]-sample_mn_tgd[j][0])<=1E-20&&
                    fabs(eph->tgd[3]-sample_mn_tgd[j][1])<=1E-20&&
                    fabs(eph->tgd[4]-sample_mn_tgd[j][2])<=1E-20&&
                    fabs(eph->tgd[5]-sample_mn_tgd[j][3])<=1E-20) {
                    stats->mn_sample_matches++;
                }
            }
        }
        stats->seen_prn[prn]=1;
        stats->seen_eph_iod[prn][raw->nav.eph[sat-1].iodc]=1;
    }

    for (prn=MINPRNCMP;prn<=MAXPRNCMP;prn++) {
        int matched=0;
        if (stats->seen_prn[prn]) stats->prn_count++;
        for (i=0;i<256;i++) {
            if (stats->seen_eph_iod[prn][i]&&stats->seen_b2b_iod[prn][i]) {
                stats->iod_match_pairs++;
                matched=1;
            }
        }
        if (matched) stats->iod_match_prns++;
    }
    for (i=0;i<(int)(sizeof(observed_prns)/sizeof(observed_prns[0]));i++) {
        if (stats->seen_prn[observed_prns[i]]) stats->observed_prn_count++;
    }
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(before);
    if (fp) fclose(fp);
    return ok;
}

int main(int argc, char **argv)
{
    frame_stats_t frames;
    decode_stats_t decoded;
    int iod,prn,pass;

    if (argc!=2) {
        fprintf(stderr,"usage: %s SINO_BIN\n",argv[0]);
        return 2;
    }
    if (!scan_frames(argv[1],&frames)||!replay_decoder(argv[1],&decoded)) {
        fprintf(stderr,"message72 test setup failed\n");
        return 2;
    }
    pass=frames.frames==2772&&frames.message72==580&&
         frames.message1697==2192&&frames.crc_errors==0&&
         frames.length_errors==0&&decoded.ret2==580&&
         decoded.ret20_type2==40&&decoded.decode_errors==0&&
         decoded.wrong_sat_writes==0&&decoded.field_errors==0&&
         decoded.time_backwards==0&&decoded.prn_count==29&&
         decoded.observed_prn_count==9&&decoded.iod_match_pairs>0&&
         decoded.finite_message72==580&&decoded.b1cp_tgd_missing==0&&
         decoded.layout_errors==0&&decoded.mn_sample_matches==60;

    printf("SINO_MESSAGE72_TEST %s\n",pass?"PASS":"FAIL");
    printf("frames=%ld message72=%ld message1697=%ld crc_errors=%ld "
           "length_errors=%ld\n",frames.frames,frames.message72,
           frames.message1697,frames.crc_errors,frames.length_errors);
    printf("ret2=%ld type2=%ld decode_errors=%ld wrong_sat_writes=%ld "
           "field_errors=%ld time_backwards=%ld b1cp_tgd_missing=%ld "
           "finite_message72=%ld layout_errors=%ld mn_sample_matches=%ld\n",
           decoded.ret2,
           decoded.ret20_type2,decoded.decode_errors,
           decoded.wrong_sat_writes,decoded.field_errors,
           decoded.time_backwards,decoded.b1cp_tgd_missing,
           decoded.finite_message72,decoded.layout_errors,
           decoded.mn_sample_matches);
    printf("eph_prns=%d observed_prns=%d iod_match_prns=%d "
           "iod_match_pairs=%d\n",decoded.prn_count,
           decoded.observed_prn_count,decoded.iod_match_prns,
           decoded.iod_match_pairs);
    printf("iod_matches=");
    for (prn=MINPRNCMP;prn<=MAXPRNCMP;prn++) {
        for (iod=0;iod<256;iod++) {
            if (decoded.seen_eph_iod[prn][iod]&&
                decoded.seen_b2b_iod[prn][iod]) {
                printf(" C%02d:%d",prn,iod);
            }
        }
    }
    printf("\n");
    for (prn=0;prn<3;prn++) {
        printf("C%02d tgd2=%.15e tgd3=%.15e tgd4=%.15e tgd5=%.15e\n",
               sample_prns[prn],decoded.sample_tgd[prn][0],
               decoded.sample_tgd[prn][1],decoded.sample_tgd[prn][2],
               decoded.sample_tgd[prn][3]);
    }
    return pass?0:1;
}
