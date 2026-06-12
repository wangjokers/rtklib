#include "unicore.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    long bridge_updates;
    long bridge_second_updates;
    int bridge_raw_consumed;
    int bridge_fields_match;
    int bridge_nav_update_visible;
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

static int same_time(gtime_t a, gtime_t b)
{
    return a.time==b.time&&a.sec==b.sec;
}

static int same_products(const B2bssr_t *a, const B2bssr_t *b)
{
    int i;

    if (a->sow!=b->sow||a->verify_sow!=b->verify_sow||
        a->iodn!=b->iodn||a->ura!=b->ura) return 0;

    for (i=0;i<6;i++) {
        if (!same_time(a->t0[i],b->t0[i])||
            a->udi[i]!=b->udi[i]||
            a->iodssr[i]!=b->iodssr[i]) return 0;
    }
    for (i=0;i<2;i++) {
        if (a->iodp[i]!=b->iodp[i]) return 0;
    }
    for (i=0;i<4;i++) {
        if (a->iodcorr[i]!=b->iodcorr[i]) return 0;
    }
    for (i=0;i<3;i++) {
        if (a->deph[i]!=b->deph[i]||
            a->ddeph[i]!=b->ddeph[i]||
            a->dclk[i]!=b->dclk[i]) return 0;
    }
    return !memcmp(a->cbias,b->cbias,sizeof(a->cbias));
}

static void collect_result(raw_t *raw, nav_t *nav, int type,
                           test_stats_t *stats)
{
    const B2bmask_t *mask;
    uint8_t pending[MAXSAT+1]={0};
    int i,n,pending_count=0,index=type_index(type);

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
        pending[i]=1;
        pending_count++;
        stats->sat_updates[index]++;
        if (!stats->sample_sat[index-1]) {
            stats->sample_sat[index-1]=i;
            stats->sample_ssr[index-1]=*ssr;
        }
    }
    n=b2b_update_nav_from_raw(nav,raw);
    stats->bridge_updates+=n;
    if (n!=pending_count) stats->bridge_fields_match=0;

    for (i=1;i<=MAXSAT;i++) {
        if (!pending[i]) continue;
        if (raw->nav.B2bssr[i].update) stats->bridge_raw_consumed=0;
        if (!nav->B2bssr[i].update) stats->bridge_nav_update_visible=0;
        if (!same_products(raw->nav.B2bssr+i,nav->B2bssr+i)) {
            stats->bridge_fields_match=0;
        }
    }
    stats->bridge_second_updates+=b2b_update_nav_from_raw(nav,raw);
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

static int test_partial_product_updates(void)
{
    raw_t *raw=(raw_t *)calloc(1,sizeof(*raw));
    nav_t *nav=(nav_t *)calloc(1,sizeof(*nav));
    B2bssr_t *src,*dst;
    int ok=1,sat=1;

    if (!raw||!nav) {
        free(raw);
        free(nav);
        return 0;
    }
    src=raw->nav.B2bssr+sat;
    dst=nav->B2bssr+sat;

    src->t0[0].time=100;
    src->iodssr[0]=3;
    src->iodn=21;
    src->iodcorr[0]=4;
    src->deph[0]=1.25;
    src->deph[1]=-2.5;
    src->deph[2]=3.75;
    src->ura=6;
    src->update=1;
    ok&=b2b_update_nav_from_raw(nav,raw)==1;

    src->t0[1].time=200;
    src->iodssr[1]=3;
    src->cbias[1]=0.85f;
    src->update=1;
    ok&=b2b_update_nav_from_raw(nav,raw)==1;
    ok&=dst->deph[0]==1.25&&dst->deph[1]==-2.5&&
        dst->deph[2]==3.75&&dst->ura==6;

    src->t0[2].time=300;
    src->iodssr[2]=3;
    src->iodp[0]=2;
    src->iodcorr[1]=4;
    src->dclk[0]=-0.64;
    src->update=1;
    ok&=b2b_update_nav_from_raw(nav,raw)==1;
    ok&=dst->deph[0]==1.25&&dst->cbias[1]==0.85f&&
        dst->dclk[0]==-0.64&&dst->update==1;
    ok&=b2b_update_nav_from_raw(nav,raw)==0;

    free(raw);
    free(nav);
    return ok;
}

static int test_sat_index_bounds(void)
{
    raw_t *raw=(raw_t *)calloc(1,sizeof(*raw));
    nav_t *nav=(nav_t *)calloc(1,sizeof(*nav));
    int ok=1;

    if (!raw||!nav) {
        free(raw);
        free(nav);
        return 0;
    }
    nav->B2bssr[0].iodn=77;
    raw->nav.B2bssr[0].iodn=88;
    raw->nav.B2bssr[0].update=1;

    raw->nav.B2bssr[MAXSAT].iodn=99;
    raw->nav.B2bssr[MAXSAT].update=1;

    ok&=b2b_update_nav_from_raw(nav,raw)==1;
    ok&=nav->B2bssr[0].iodn==77;
    ok&=raw->nav.B2bssr[0].update==1;
    ok&=nav->B2bssr[MAXSAT].iodn==99;
    ok&=nav->B2bssr[MAXSAT].update==1;
    ok&=raw->nav.B2bssr[MAXSAT].update==0;

    free(raw);
    free(nav);
    return ok;
}

int main(int argc, char **argv)
{
    test_stats_t stats={0};
    nav_t *nav;
    raw_t *raw,*other;
    const B2bmask_t *other_mask;
    FILE *fp;
    int ret,context_isolated,partial_products,index_bounds;

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
    if (!(nav=(nav_t *)calloc(1,sizeof(*nav)))) {
        free(raw);
        free(other);
        fclose(fp);
        return 1;
    }
    stats.bridge_raw_consumed=1;
    stats.bridge_fields_match=1;
    stats.bridge_nav_update_visible=1;
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
        free(nav);
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
            collect_result(raw,nav,frame_type(raw),&stats);
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
    partial_products=test_partial_product_updates();
    index_bounds=test_sat_index_bounds();
    printf("BRIDGE_UPDATED_SATS %ld\n",stats.bridge_updates);
    printf("BRIDGE_SECOND_CALL_UPDATES %ld\n",stats.bridge_second_updates);
    printf("BRIDGE_RAW_CONSUMED %d\n",stats.bridge_raw_consumed);
    printf("BRIDGE_FIELDS_MATCH %d\n",stats.bridge_fields_match);
    printf("BRIDGE_NAV_UPDATE_VISIBLE %d\n",stats.bridge_nav_update_visible);
    printf("BRIDGE_PARTIAL_PRODUCTS %d\n",partial_products);
    printf("BRIDGE_INDEX_BOUNDS %d\n",index_bounds);

    free_raw(raw);
    free_raw(other);
    free(raw);
    free(other);
    free(nav);
    fclose(fp);
    return stats.errors||!context_isolated||stats.bridge_second_updates||
           !stats.bridge_raw_consumed||!stats.bridge_fields_match||
           !stats.bridge_nav_update_visible||!partial_products||!index_bounds?
           1:0;
}
