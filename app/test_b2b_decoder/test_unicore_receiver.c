#include "unicore.h"

#include <stdio.h>
#include <stdlib.h>

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308

typedef struct {
    long frames[4];
    long sat_updates[4];
    int errors;
    int have_mask;
    B2bmask_t first_mask;
    int sample_sat[3];
    B2bssr_t sample_ssr[3];
} test_stats_t;

static int frame_type(const raw_t *raw)
{
    return raw->buff[4]|(raw->buff[5]<<8);
}

static int type_index(int type)
{
    if (type==UNICORE_MSG_B2B_INFO1) return 0;
    if (type==UNICORE_MSG_B2B_INFO2) return 1;
    if (type==UNICORE_MSG_B2B_INFO3) return 2;
    if (type==UNICORE_MSG_B2B_INFO4) return 3;
    return -1;
}

static void collect_result(raw_t *raw, int type, test_stats_t *stats)
{
    const B2bmask_t *mask;
    int i,index=type_index(type);

    if (index<0) return;
    stats->frames[index]++;

    if (type==UNICORE_MSG_B2B_INFO1) {
        if (!stats->have_mask&&(mask=unicore_b2b_mask(raw))) {
            stats->first_mask=*mask;
            stats->have_mask=1;
        }
        return;
    }
    for (i=1;i<=MAXSAT;i++) {
        B2bssr_t *ssr=&raw->nav.B2bssr[i];

        if (!ssr->update) continue;
        stats->sat_updates[index]++;
        if (!stats->sample_sat[index-1]) {
            stats->sample_sat[index-1]=i;
            stats->sample_ssr[index-1]=*ssr;
        }
        ssr->update=0;
    }
}

static void print_mask(const test_stats_t *stats)
{
    int i,n;

    printf("MASK_COUNT %ld\n",stats->frames[0]);
    if (!stats->have_mask) return;
    printf("MASK_FIRST_SATNUM %d\n",stats->first_mask.satnum);
    printf("MASK_FIRST_SATNO");
    n=stats->first_mask.satnum<8?stats->first_mask.satnum:8;
    for (i=0;i<n;i++) printf(" %d",stats->first_mask.satno[i]);
    printf("\n");
}

static void print_orbit_sample(const test_stats_t *stats)
{
    const B2bssr_t *ssr=&stats->sample_ssr[0];

    printf("ORBIT_COUNT %ld\n",stats->frames[1]);
    printf("ORBIT_UPDATE_SATS %ld\n",stats->sat_updates[1]);
    if (!stats->sample_sat[0]) return;
    printf("ORBIT_SAMPLE sat=%d iodssr=%d iodn=%d iodcorr=%u "
           "deph=%.4f,%.4f,%.4f ura=%d\n",
           stats->sample_sat[0],ssr->iodssr[0],ssr->iodn,
           (unsigned int)ssr->iodcorr[0],ssr->deph[0],ssr->deph[1],
           ssr->deph[2],ssr->ura);
}

static void print_code_sample(const test_stats_t *stats)
{
    const B2bssr_t *ssr=&stats->sample_ssr[1];
    int code,n=0;

    printf("CODE_BIAS_COUNT %ld\n",stats->frames[2]);
    printf("CODE_BIAS_UPDATE_SATS %ld\n",stats->sat_updates[2]);
    if (!stats->sample_sat[1]) return;
    printf("CODE_BIAS_SAMPLE sat=%d iodssr=%d",stats->sample_sat[1],
           ssr->iodssr[1]);
    for (code=1;code<=MAXCODE&&n<4;code++) {
        if (ssr->cbias[code]==0.0f) continue;
        printf(" code%d=%.3f",code,ssr->cbias[code]);
        n++;
    }
    printf("\n");
}

static void print_clock_sample(const test_stats_t *stats)
{
    const B2bssr_t *ssr=&stats->sample_ssr[2];

    printf("CLOCK_COUNT %ld\n",stats->frames[3]);
    printf("CLOCK_UPDATE_SATS %ld\n",stats->sat_updates[3]);
    if (!stats->sample_sat[2]) return;
    printf("CLOCK_SAMPLE sat=%d iodssr=%d iodp=%d iodcorr=%u dclk0=%.4f\n",
           stats->sample_sat[2],ssr->iodssr[2],ssr->iodp[0],
           (unsigned int)ssr->iodcorr[1],ssr->dclk[0]);
}

int main(int argc, char **argv)
{
    test_stats_t stats={0};
    raw_t *raw,*other;
    const B2bmask_t *other_mask;
    FILE *fp;
    int ret,context_isolated;

    if (argc!=2) {
        fprintf(stderr,"Usage: %s <Unicore_B2bBin>\n",argv[0]);
        return 1;
    }
    if (!(fp=fopen(argv[1],"rb"))) {
        fprintf(stderr,"failed to open input: %s\n",argv[1]);
        return 1;
    }
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))) {
        fclose(fp);
        return 1;
    }
    if (!(other=(raw_t *)calloc(1,sizeof(*other)))) {
        free(raw);
        fclose(fp);
        return 1;
    }
    /*
     * Stage 3C test entry:
     *   init_raw(..., STRFMT_UNICORE)
     *     -> input_rawf(..., STRFMT_UNICORE, fp)
     *       -> input_unicoref()
     *         -> input_unicore()
     * The test deliberately avoids calling decoder static functions or the
     * byte decoder directly, so message counts come from the raw dispatch path.
     */
    if (!init_raw(raw,STRFMT_UNICORE)||!init_raw(other,STRFMT_UNICORE)) {
        free_raw(raw);
        free_raw(other);
        free(raw);
        free(other);
        fclose(fp);
        return 1;
    }

    for (;;) {
        ret=input_rawf(raw,STRFMT_UNICORE,fp);
        if (ret==-2) break; /* EOF from input_unicoref() */
        if (ret<0) {
            stats.errors++;
        }
        else if (ret==20) {
            collect_result(raw,frame_type(raw),&stats);
        }
    }

    print_mask(&stats);
    print_orbit_sample(&stats);
    print_code_sample(&stats);
    print_clock_sample(&stats);
    printf("ERROR_COUNT %d\n",stats.errors);
    other_mask=unicore_b2b_mask(other);
    context_isolated=other_mask&&other_mask->IOD_SSR==-1&&
                     other_mask->IODP==-1&&other_mask->satnum==0;
    printf("CONTEXT_ISOLATION %d\n",context_isolated);

    free_raw(raw);
    free_raw(other);
    free(raw);
    free(other);
    fclose(fp);
    return stats.errors||!context_isolated?1:0;
}
