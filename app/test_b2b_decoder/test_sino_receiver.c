#include "rtklib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SINO_HEADER_LEN 28
#define SINO_PRN6_POS   (SINO_HEADER_LEN*8+32)
#define SINO_TYPE_POS   (SINO_HEADER_LEN*8+44)
#define SINO_DATA_POS   (SINO_HEADER_LEN*8+50)
#define SINO_IODSSR_POS (SINO_DATA_POS+21)
#define SINO_IODP_POS   (SINO_DATA_POS+23)
#define SINO_SUBTYPE_POS (SINO_DATA_POS+27)
#define SINO_CLOCK_RECORD_POS (SINO_DATA_POS+32)
#define SINO_ORBIT_RECORD_POS (SINO_DATA_POS+23)
#define SINO_ORBIT_RECORD_BITS 69
#define SINO_CLOCK_RECORD_BITS 18

typedef struct {
    long accepted[4];
    long update_sats[4];
    long coherent_publications;
    long invalid_publications;
    long missing_source_publications;
    long source_publications[64];
    long file_errors;
    long valid_biases;
    int valid_zero_bias;
    int sample_code;
    float sample_bias;
    int have_mask;
    B2bmask_t first_mask;
    int sample_sat[3];
    B2bssr_t sample_ssr[3];
} product_stats_t;

typedef struct {
    long frames;
    long types[64];
    long crc_errors;
    long frame_errors;
    long other_product_errors;
} raw_stats_t;

typedef struct {
    long events[4];
    long pending_sats[4];
    long transferred_sats[4];
    long second_call_updates;
    long cbias_valid_raw;
    long cbias_valid_nav;
    int errors;
    int raw_consumed;
    int fields_match;
    int cbias_valid_match;
    int nav_update_visible;
    int nav_update_survives_second_call;
    int valid_zero_copied;
    int index_zero_unchanged;
} bridge_stats_t;

typedef struct {
    uint8_t type1[MAXRAWLEN];
    uint8_t type4[MAXRAWLEN];
    int type1_len;
    int type4_len;
} prn_context_sample_t;

typedef struct {
    uint8_t type1[MAXRAWLEN];
    uint8_t type2[MAXRAWLEN];
    uint8_t type4[MAXRAWLEN];
    int type1_len;
    int type2_len;
    int type4_len;
    int source_prn6;
    int target_sat;
    int orbit_record;
    int clock_record;
    int iodssr;
    int iodp;
    int iodn;
    int iodcorr;
} coherent_fixture_t;

typedef struct {
    const char *name;
    long raw[4];
    long products[4];
    long frames;
    long unknown;
    int orbit_iodn;
    int orbit_iodcorr;
    double deph[3];
    double dclk;
} receiver_baseline_t;

extern const B2bssr_t *sino_b2b_source_product(const raw_t *raw, int prn6,
                                                int sat);
extern int sino_b2b_selected_source(const raw_t *raw, int sat);

static int feed_frame(raw_t *raw, const uint8_t *frame, int n);

static int b2b_type(const raw_t *raw)
{
    if (!raw||raw->len<SINO_HEADER_LEN||
        SINO_TYPE_POS+6>raw->len*8) return -1;
    return (int)getbitu(raw->buff,SINO_TYPE_POS,6);
}

static int b2b_prn6(const raw_t *raw)
{
    if (!raw||raw->len<SINO_HEADER_LEN||
        SINO_PRN6_POS+6>raw->len*8) return -1;
    return (int)getbitu(raw->buff,SINO_PRN6_POS,6);
}

static void clear_updates(raw_t *raw)
{
    int sat;

    for (sat=1;sat<=MAXSAT;sat++) raw->nav.B2bssr[sat].update=0;
}

static int any_update(const raw_t *raw)
{
    int sat;

    for (sat=1;sat<=MAXSAT;sat++) {
        if (raw->nav.B2bssr[sat].update) return 1;
    }
    return 0;
}

/* Compare every B2b product field while deliberately excluding update, whose
 * value differs by design after raw-to-nav consumption. */
static int same_b2b_products(const B2bssr_t *a, const B2bssr_t *b)
{
    int i;

    if (!a||!b||a->sow!=b->sow||a->verify_sow!=b->verify_sow||
        a->iodn!=b->iodn||a->ura!=b->ura||
        a->source_prn6!=b->source_prn6||
        a->source_valid!=b->source_valid) return 0;
    for (i=0;i<6;i++) {
        if (a->t0[i].time!=b->t0[i].time||a->t0[i].sec!=b->t0[i].sec||
            a->udi[i]!=b->udi[i]||a->iodssr[i]!=b->iodssr[i]) return 0;
    }
    for (i=0;i<2;i++) {
        if (a->iodp[i]!=b->iodp[i]) return 0;
    }
    for (i=0;i<4;i++) {
        if (a->iodcorr[i]!=b->iodcorr[i]) return 0;
    }
    for (i=0;i<3;i++) {
        if (a->deph[i]!=b->deph[i]||a->ddeph[i]!=b->ddeph[i]||
            a->dclk[i]!=b->dclk[i]) return 0;
    }
    return !memcmp(a->cbias,b->cbias,sizeof(a->cbias))&&
           !memcmp(a->cbias_valid,b->cbias_valid,sizeof(a->cbias_valid));
}

static int publication_matches_source(const raw_t *raw, int sat,
                                      const B2bssr_t *published)
{
    const B2bssr_t *source;
    B2bssr_t expected;
    int prn6=sino_b2b_selected_source(raw,sat);

    if (!published||!published->source_valid||prn6<0||
        prn6!=published->source_prn6||
        !(source=sino_b2b_source_product(raw,prn6,sat))) return 0;
    expected=*source;
    if (!expected.t0[1].time||expected.iodssr[1]!=expected.iodssr[0]) {
        memset(expected.cbias_valid,0,sizeof(expected.cbias_valid));
    }
    expected.source_prn6=(uint8_t)prn6;
    expected.source_valid=1;
    return same_b2b_products(&expected,published);
}

static void set_unsigned_bits(uint8_t *buff, int pos, int len, uint32_t data)
{
    uint8_t mask;
    int i,bit;

    for (i=0;i<len;i++) {
        bit=pos+i;
        mask=(uint8_t)(1u<<(7-bit%8));
        if ((data>>(len-i-1))&1u) buff[bit/8]|=mask;
        else buff[bit/8]&=(uint8_t)~mask;
    }
}

static void put_u4(uint8_t *p, uint32_t value)
{
    p[0]=(uint8_t)value;
    p[1]=(uint8_t)(value>>8);
    p[2]=(uint8_t)(value>>16);
    p[3]=(uint8_t)(value>>24);
}

static void update_frame_crc(uint8_t *frame, int total)
{
    int payload_len=total-4;

    put_u4(frame+payload_len,rtk_crc32(frame,payload_len));
}

static int mask_from_type1_frame(const uint8_t *frame, int total,
                                 B2bmask_t *mask)
{
    int i,pos=SINO_DATA_POS;

    if (!frame||!mask||total<4||SINO_DATA_POS+27+B2B_MAXSAT>
        (total-4)*8||getbitu(frame,SINO_TYPE_POS,6)!=1) return 0;
    memset(mask,0,sizeof(*mask));
    pos+=17+4;
    mask->IOD_SSR=(int)getbitu(frame,pos,2); pos+=2;
    mask->IODP=(int)getbitu(frame,pos,4); pos+=4;
    for (i=0;i<63;i++,pos++) mask->MASK_BD[i]=(int)getbitu(frame,pos,1);
    for (i=0;i<37;i++,pos++) mask->MASK_GPS[i]=(int)getbitu(frame,pos,1);
    for (i=0;i<37;i++,pos++) mask->MASK_GAL[i]=(int)getbitu(frame,pos,1);
    for (i=0;i<37;i++,pos++) mask->MASK_GLO[i]=(int)getbitu(frame,pos,1);
    b2b_mask2satno(mask);
    return mask->satnum>0;
}

static int mask_sat_index(const B2bmask_t *mask, int sat)
{
    int i;

    if (!mask) return -1;
    for (i=0;i<mask->satnum&&i<B2B_MAXSAT;i++) {
        if (mask->satno[i]==sat) return i;
    }
    return -1;
}

static int orbit_record_for_sat(const uint8_t *frame, int total, int sat)
{
    int i,pos,slot;

    if (!frame||total<4||SINO_ORBIT_RECORD_POS+6*SINO_ORBIT_RECORD_BITS>
        (total-4)*8||getbitu(frame,SINO_TYPE_POS,6)!=2) return -1;
    for (i=0;i<6;i++) {
        pos=SINO_ORBIT_RECORD_POS+i*SINO_ORBIT_RECORD_BITS;
        slot=(int)getbitu(frame,pos,9);
        if (b2b_slot2satno(slot)==sat) return i;
    }
    return -1;
}

static int clock_record_for_sat(const uint8_t *frame, int total,
                                const B2bmask_t *mask, int sat)
{
    int index,subtype,record;

    if (!frame||!mask||total<4||
        SINO_CLOCK_RECORD_POS+23*SINO_CLOCK_RECORD_BITS>(total-4)*8||
        getbitu(frame,SINO_TYPE_POS,6)!=4) return -1;
    index=mask_sat_index(mask,sat);
    subtype=(int)getbitu(frame,SINO_SUBTYPE_POS,5);
    record=index-subtype*23;
    return record>=0&&record<23?record:-1;
}

static int fixture_identity_matches(const B2bssr_t *ssr,
                                    const coherent_fixture_t *fixture,
                                    int variant)
{
    int iodssr=(fixture->iodssr+variant)&3;
    int iodp=(fixture->iodp+variant)&15;
    int iodn=(fixture->iodn+variant)&1023;
    int iodcorr=(fixture->iodcorr+variant)&7;

    return ssr&&ssr->iodssr[0]==iodssr&&ssr->iodssr[2]==iodssr&&
           ssr->iodp[0]==iodp&&ssr->iodn==iodn&&
           ssr->iodcorr[0]==iodcorr&&ssr->iodcorr[1]==iodcorr;
}

static int make_fixture_frame(const coherent_fixture_t *fixture, int type,
                              int prn6, int variant, uint8_t *frame,
                              int *total)
{
    const uint8_t *source=NULL;
    int len=0,pos;

    if (!fixture||!frame||!total||prn6<0||prn6>=64||variant<0||variant>1) {
        return 0;
    }
    if (type==1) {source=fixture->type1; len=fixture->type1_len;}
    else if (type==2) {source=fixture->type2; len=fixture->type2_len;}
    else if (type==4) {source=fixture->type4; len=fixture->type4_len;}
    if (!source||len<=4||len>MAXRAWLEN) return 0;

    memcpy(frame,source,(size_t)len);
    set_unsigned_bits(frame,SINO_PRN6_POS,6,(uint32_t)prn6);
    set_unsigned_bits(frame,SINO_IODSSR_POS,2,
                      (uint32_t)((fixture->iodssr+variant)&3));
    if (type==1||type==4) {
        set_unsigned_bits(frame,SINO_IODP_POS,4,
                          (uint32_t)((fixture->iodp+variant)&15));
    }
    if (type==2) {
        pos=SINO_ORBIT_RECORD_POS+
            fixture->orbit_record*SINO_ORBIT_RECORD_BITS;
        set_unsigned_bits(frame,pos+9,10,
                          (uint32_t)((fixture->iodn+variant)&1023));
        set_unsigned_bits(frame,pos+19,3,
                          (uint32_t)((fixture->iodcorr+variant)&7));
    }
    else if (type==4) {
        pos=SINO_CLOCK_RECORD_POS+
            fixture->clock_record*SINO_CLOCK_RECORD_BITS;
        set_unsigned_bits(frame,pos,3,
                          (uint32_t)((fixture->iodcorr+variant)&7));
    }
    update_frame_crc(frame,len);
    *total=len;
    return 1;
}

static int feed_fixture_frame(raw_t *raw, const coherent_fixture_t *fixture,
                              int type, int prn6, int variant)
{
    uint8_t frame[MAXRAWLEN];
    int total;

    if (!make_fixture_frame(fixture,type,prn6,variant,frame,&total)) return -1;
    return feed_frame(raw,frame,total);
}

static int feed_complete_source(raw_t *raw,
                                const coherent_fixture_t *fixture,
                                int prn6, int variant)
{
    return feed_fixture_frame(raw,fixture,1,prn6,variant)==20&&
           feed_fixture_frame(raw,fixture,2,prn6,variant)==20&&
           feed_fixture_frame(raw,fixture,4,prn6,variant)==20;
}

static void collect_product(raw_t *raw, product_stats_t *stats)
{
    const B2bmask_t *mask;
    int type=b2b_type(raw);
    int sat,code,index=type-1;

    if (type<1||type>4) return;
    stats->accepted[index]++;

    if (type==1) {
        if (!stats->have_mask&&(mask=sino_b2b_mask(raw))) {
            stats->first_mask=*mask;
            stats->have_mask=1;
        }
        return;
    }

    for (sat=1;sat<=MAXSAT;sat++) {
        const B2bssr_t *ssr=&raw->nav.B2bssr[sat];

        if (!ssr->update) continue;
        stats->update_sats[index]++;
        if (!ssr->source_valid||ssr->source_prn6>=64) {
            stats->missing_source_publications++;
        }
        else {
            stats->source_publications[ssr->source_prn6]++;
        }
        if (b2b_orbit_clock_ready(raw->time,ssr,NULL,NULL)&&
            publication_matches_source(raw,sat,ssr)) {
            stats->coherent_publications++;
        }
        else {
            stats->invalid_publications++;
        }
        if (type>=2&&type<=4&&!stats->sample_sat[type-2]) {
            stats->sample_sat[type-2]=sat;
            stats->sample_ssr[type-2]=*ssr;
        }
        for (code=1;code<=MAXCODE;code++) {
            if (!ssr->cbias_valid[code]) continue;
            stats->valid_biases++;
            if (!stats->sample_code) {
                stats->sample_code=code;
                stats->sample_bias=ssr->cbias[code];
            }
            if (ssr->cbias[code]==0.0f) stats->valid_zero_bias=1;
        }
    }
}

/* Exercise the required file-level public raw path without using the nav bridge. */
static int replay_products(const char *path, product_stats_t *stats,
                           int *context_isolated, int *index_zero_unchanged)
{
    raw_t *raw=NULL,*other=NULL;
    B2bssr_t index_zero_before;
    const B2bmask_t *other_mask;
    FILE *fp=NULL;
    int ret,ok=0;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(other=(raw_t *)calloc(1,sizeof(*other)))||
        !(fp=fopen(path,"rb"))) goto done;
    if (!init_raw(raw,STRFMT_SINO)||!init_raw(other,STRFMT_SINO)) goto done;

    raw->nav.B2bssr[0].iodn=12345;
    raw->nav.B2bssr[0].deph[0]=9.25;
    raw->nav.B2bssr[0].update=7;
    index_zero_before=raw->nav.B2bssr[0];

    for (;;) {
        clear_updates(raw);
        ret=input_rawf(raw,STRFMT_SINO,fp);
        if (ret==-2) break;
        if (ret<0) stats->file_errors++;
        else if (ret==20) collect_product(raw,stats);
    }

    other_mask=sino_b2b_mask(other);
    *context_isolated=raw->rcv_data&&other->rcv_data&&
                      raw->rcv_data!=other->rcv_data&&other_mask&&
                      other_mask->IOD_SSR==-1&&other_mask->IODP==-1&&
                      other_mask->satnum==0;
    *index_zero_unchanged=!memcmp(raw->nav.B2bssr,
                                 &index_zero_before,sizeof(index_zero_before));
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    if (other) {
        if (other->format==STRFMT_SINO) free_raw(other);
        free(other);
    }
    if (fp) fclose(fp);
    return ok;
}

/* Stage 3D: decode real Sino data through the public raw path, then consume
 * each pending satellite with the common raw-to-main-nav bridge. */
static int replay_nav_bridge(const char *path, bridge_stats_t *stats)
{
    raw_t *raw=NULL;
    nav_t *nav=NULL;
    B2bssr_t raw_zero_before,nav_zero_before;
    uint8_t pending[MAXSAT+1];
    FILE *fp=NULL;
    int ret,type,index,sat,code,n,pending_count,ok=0;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(nav=(nav_t *)calloc(1,sizeof(*nav)))||
        !(fp=fopen(path,"rb"))||!init_raw(raw,STRFMT_SINO)) goto done;

    stats->raw_consumed=1;
    stats->fields_match=1;
    stats->cbias_valid_match=1;
    stats->nav_update_visible=1;
    stats->nav_update_survives_second_call=1;

    raw->nav.B2bssr[0].iodn=8101;
    raw->nav.B2bssr[0].deph[0]=8.125;
    raw->nav.B2bssr[0].cbias_valid[CODE_L1P]=1;
    raw->nav.B2bssr[0].update=3;
    nav->B2bssr[0].iodn=9101;
    nav->B2bssr[0].dclk[0]=9.25;
    nav->B2bssr[0].cbias_valid[CODE_L2I]=1;
    nav->B2bssr[0].update=5;
    raw_zero_before=raw->nav.B2bssr[0];
    nav_zero_before=nav->B2bssr[0];

    for (;;) {
        ret=input_rawf(raw,STRFMT_SINO,fp);
        if (ret==-2) break;
        if (ret<0) {
            stats->errors++;
            continue;
        }
        if (ret!=20) continue;

        type=b2b_type(raw);
        index=type-1;
        if (index<0||index>=4) {
            stats->errors++;
            continue;
        }
        stats->events[index]++;
        memset(pending,0,sizeof(pending));
        pending_count=0;
        for (sat=1;sat<=MAXSAT;sat++) {
            if (!raw->nav.B2bssr[sat].update) continue;
            pending[sat]=1;
            pending_count++;
        }
        stats->pending_sats[index]+=pending_count;

        n=b2b_update_nav_from_raw(nav,raw);
        stats->transferred_sats[index]+=n;
        if (n!=pending_count) stats->fields_match=0;
        if (type==1&&n!=0) stats->fields_match=0;

        for (sat=1;sat<=MAXSAT;sat++) {
            const B2bssr_t *src,*dst;

            if (!pending[sat]) continue;
            src=&raw->nav.B2bssr[sat];
            dst=&nav->B2bssr[sat];
            if (src->update) stats->raw_consumed=0;
            if (!dst->update) stats->nav_update_visible=0;
            if (!same_b2b_products(src,dst)) stats->fields_match=0;

            for (code=1;code<=MAXCODE;code++) {
                if (src->cbias_valid[code]) {
                    stats->cbias_valid_raw++;
                    if (src->cbias[code]==0.0f&&dst->cbias_valid[code]&&
                        dst->cbias[code]==0.0f) {
                        stats->valid_zero_copied=1;
                    }
                }
                if (dst->cbias_valid[code]) stats->cbias_valid_nav++;
                if (src->cbias_valid[code]!=dst->cbias_valid[code]) {
                    stats->cbias_valid_match=0;
                }
            }
        }

        stats->second_call_updates+=b2b_update_nav_from_raw(nav,raw);
        for (sat=1;sat<=MAXSAT;sat++) {
            if (pending[sat]&&!nav->B2bssr[sat].update) {
                stats->nav_update_survives_second_call=0;
            }
        }
        b2b_clear_nav_updates(nav);
    }

    stats->index_zero_unchanged=
        !memcmp(raw->nav.B2bssr,&raw_zero_before,sizeof(raw_zero_before))&&
        !memcmp(nav->B2bssr,&nav_zero_before,sizeof(nav_zero_before));
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(nav);
    if (fp) fclose(fp);
    return ok;
}

/* Count every complete frame, including Type 63 pages that correctly return 0. */
static int replay_raw_bytes(const char *path, raw_stats_t *stats)
{
    raw_t *raw=NULL;
    FILE *fp=NULL;
    int data,ret,before,complete,type,ok=0;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(fp=fopen(path,"rb"))) goto done;
    if (!init_raw(raw,STRFMT_SINO)) goto done;

    while ((data=fgetc(fp))!=EOF) {
        before=raw->nbyte;
        ret=input_raw(raw,STRFMT_SINO,(uint8_t)data);
        complete=before>0&&raw->nbyte==0&&raw->len>=SINO_HEADER_LEN&&
                 before+1==raw->len+4;
        if (complete) {
            stats->frames++;
            type=b2b_type(raw);
            if (type>=0&&type<64) stats->types[type]++;
            if (ret<0) stats->crc_errors++;
            if ((type<1||type>4)&&(ret!=0||any_update(raw))) {
                stats->other_product_errors++;
            }
            clear_updates(raw);
        }
        else if (ret<0) {
            stats->frame_errors++;
        }
    }
    ok=raw->nbyte==0;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    if (fp) fclose(fp);
    return ok;
}

static int read_first_frame(const char *path, uint8_t *frame, int *frame_len)
{
    FILE *fp;
    int payload_len,total;

    if (!(fp=fopen(path,"rb"))) return 0;
    if (fread(frame,1,10,fp)!=10) {
        fclose(fp);
        return 0;
    }
    payload_len=frame[8]|(frame[9]<<8);
    total=SINO_HEADER_LEN+payload_len+4;
    if (total>MAXRAWLEN||fread(frame+10,1,total-10,fp)!=(size_t)(total-10)) {
        fclose(fp);
        return 0;
    }
    fclose(fp);
    *frame_len=total;
    return 1;
}

static int feed_frame(raw_t *raw, const uint8_t *frame, int n)
{
    int i,ret=0;

    for (i=0;i<n;i++) ret=input_raw(raw,STRFMT_SINO,frame[i]);
    return ret;
}

/* Capture one real, complete source-local orbit/clock identity. The captured
 * frames are later cloned to controlled PRN6 sources with a recomputed CRC. */
static int capture_coherent_fixture(const char *path,
                                    coherent_fixture_t *fixture)
{
    raw_t *raw=NULL;
    B2bmask_t mask,next_mask;
    FILE *fp=NULL;
    int data,before,complete,type,prn6,ret,total,record,pos;
    int radial,in_track,cross,ura,c0,clock_corr=-1,ok=0;

    if (!fixture||(raw=(raw_t *)calloc(1,sizeof(*raw)))==NULL||
        (fp=fopen(path,"rb"))==NULL||!init_raw(raw,STRFMT_SINO)) goto done;
    memset(fixture,0,sizeof(*fixture));
    memset(&mask,0,sizeof(mask));
    fixture->source_prn6=1;

    while ((data=fgetc(fp))!=EOF) {
        before=raw->nbyte;
        ret=input_raw(raw,STRFMT_SINO,(uint8_t)data);
        complete=before>0&&raw->nbyte==0&&raw->len>=SINO_HEADER_LEN&&
                 before+1==raw->len+4;
        if (!complete||ret!=20) continue;

        type=b2b_type(raw);
        prn6=b2b_prn6(raw);
        total=raw->len+4;
        if (prn6!=fixture->source_prn6) continue;

        if (type==1&&mask_from_type1_frame(raw->buff,total,&next_mask)) {
            if (fixture->type1_len&&
                (next_mask.IOD_SSR!=fixture->iodssr||
                 next_mask.IODP!=fixture->iodp||
                 next_mask.satno[0]!=fixture->target_sat)) {
                fixture->type2_len=fixture->type4_len=0;
                clock_corr=-1;
            }
            mask=next_mask;
            fixture->iodssr=mask.IOD_SSR;
            fixture->iodp=mask.IODP;
            fixture->target_sat=mask.satno[0];
            memcpy(fixture->type1,raw->buff,(size_t)total);
            fixture->type1_len=total;
        }
        else if (type==2&&fixture->type1_len&&
                 (int)getbitu(raw->buff,SINO_IODSSR_POS,2)==
                 fixture->iodssr&&
                 (record=orbit_record_for_sat(raw->buff,total,
                                               fixture->target_sat))>=0) {
            pos=SINO_ORBIT_RECORD_POS+record*SINO_ORBIT_RECORD_BITS;
            radial=(int)getbits(raw->buff,pos+22,15);
            in_track=(int)getbits(raw->buff,pos+37,13);
            cross=(int)getbits(raw->buff,pos+50,13);
            ura=(int)getbitu(raw->buff,pos+63,6);
            if (abs(radial)<16383&&abs(in_track)<4095&&abs(cross)<4095&&
                ura<63) {
                fixture->orbit_record=record;
                fixture->iodn=(int)getbitu(raw->buff,pos+9,10);
                fixture->iodcorr=(int)getbitu(raw->buff,pos+19,3);
                memcpy(fixture->type2,raw->buff,(size_t)total);
                fixture->type2_len=total;
            }
        }
        else if (type==4&&fixture->type1_len&&
                 (int)getbitu(raw->buff,SINO_IODSSR_POS,2)==
                 fixture->iodssr&&
                 (int)getbitu(raw->buff,SINO_IODP_POS,4)==fixture->iodp&&
                 (record=clock_record_for_sat(raw->buff,total,&mask,
                                               fixture->target_sat))>=0) {
            pos=SINO_CLOCK_RECORD_POS+record*SINO_CLOCK_RECORD_BITS;
            c0=(int)getbits(raw->buff,pos+3,15);
            if (abs(c0)<16383) {
                fixture->clock_record=record;
                clock_corr=(int)getbitu(raw->buff,pos,3);
                memcpy(fixture->type4,raw->buff,(size_t)total);
                fixture->type4_len=total;
            }
        }
        if (fixture->type1_len&&fixture->type2_len&&fixture->type4_len&&
            fixture->iodcorr==clock_corr) {
            ok=1;
            break;
        }
    }

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    if (fp) fclose(fp);
    return ok;
}

/* Re-feed real Type 2/3/4 frames into a fresh raw_t that has never received
 * Type 1. None may publish products without receiver-local MASK context. */
static int test_mask_gate(const char *path, int *mask_gate_reject)
{
    raw_t *parser=NULL,*empty=NULL;
    uint8_t (*frames)[MAXRAWLEN]=NULL;
    FILE *fp=NULL;
    const B2bmask_t *mask;
    int lengths[3]={0};
    int data,before,complete,type,ret,i,found=0,ok=0;

    if (!(parser=(raw_t *)calloc(1,sizeof(*parser)))||
        !(empty=(raw_t *)calloc(1,sizeof(*empty)))||
        !(frames=(uint8_t (*)[MAXRAWLEN])calloc(3,MAXRAWLEN))||
        !(fp=fopen(path,"rb"))||!init_raw(parser,STRFMT_SINO)||
        !init_raw(empty,STRFMT_SINO)) goto done;

    while (found<3&&(data=fgetc(fp))!=EOF) {
        before=parser->nbyte;
        ret=input_raw(parser,STRFMT_SINO,(uint8_t)data);
        complete=before>0&&parser->nbyte==0&&
                 parser->len>=SINO_HEADER_LEN&&before+1==parser->len+4;
        if (!complete) continue;
        type=b2b_type(parser);
        if (type>=2&&type<=4&&!lengths[type-2]) {
            lengths[type-2]=parser->len+4;
            memcpy(frames[type-2],parser->buff,(size_t)lengths[type-2]);
            found++;
        }
        clear_updates(parser);
    }
    if (found!=3) goto done;

    *mask_gate_reject=1;
    for (i=0;i<3;i++) {
        clear_updates(empty);
        ret=feed_frame(empty,frames[i],lengths[i]);
        if (ret!=0||any_update(empty)) *mask_gate_reject=0;
    }
    mask=sino_b2b_mask(empty);
    if (!mask||mask->IOD_SSR!=-1||mask->IODP!=-1||mask->satnum!=0) {
        *mask_gate_reject=0;
    }
    ok=1;

done:
    if (parser) {
        if (parser->format==STRFMT_SINO) free_raw(parser);
        free(parser);
    }
    if (empty) {
        if (empty->format==STRFMT_SINO) free_raw(empty);
        free(empty);
    }
    free(frames);
    if (fp) fclose(fp);
    return ok;
}

/* Mutate a known-good Type 4 page while keeping CRC valid. Both IOD gates
 * must suppress raw->nav.B2bssr[] updates rather than reporting CRC failure. */
static int test_iod_gates(const char *path, int *iodssr_reject,
                          int *iodp_reject)
{
    raw_t *raw=NULL;
    uint8_t *original=NULL,*work=NULL;
    FILE *fp=NULL;
    uint32_t crc;
    int data,before,complete,type,ret,total=0,payload_len=0,ok=0;
    int iodssr,iodp;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(original=(uint8_t *)malloc(MAXRAWLEN))||
        !(work=(uint8_t *)malloc(MAXRAWLEN))||
        !(fp=fopen(path,"rb"))||!init_raw(raw,STRFMT_SINO)) goto done;

    while ((data=fgetc(fp))!=EOF) {
        before=raw->nbyte;
        ret=input_raw(raw,STRFMT_SINO,(uint8_t)data);
        complete=before>0&&raw->nbyte==0&&raw->len>=SINO_HEADER_LEN&&
                 before+1==raw->len+4;
        type=complete?b2b_type(raw):-1;
        if (type==4&&ret==20) {
            payload_len=raw->len;
            total=payload_len+4;
            memcpy(original,raw->buff,(size_t)total);
            break;
        }
        if (complete) clear_updates(raw);
    }
    if (!total) goto done;

    iodssr=(int)getbitu(original,SINO_IODSSR_POS,2);
    iodp=(int)getbitu(original,SINO_IODP_POS,4);

    memcpy(work,original,(size_t)total);
    set_unsigned_bits(work,SINO_IODSSR_POS,2,(uint32_t)((iodssr+1)&3));
    crc=rtk_crc32(work,payload_len);
    put_u4(work+payload_len,crc);
    clear_updates(raw);
    ret=feed_frame(raw,work,total);
    *iodssr_reject=ret==0&&!any_update(raw);

    memcpy(work,original,(size_t)total);
    set_unsigned_bits(work,SINO_IODP_POS,4,(uint32_t)((iodp+1)&15));
    crc=rtk_crc32(work,payload_len);
    put_u4(work+payload_len,crc);
    clear_updates(raw);
    ret=feed_frame(raw,work,total);
    *iodp_reject=ret==0&&!any_update(raw);
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(original);
    free(work);
    if (fp) fclose(fp);
    return ok;
}

/* Capture accepted Type 4 pages together with the most recent same-source
 * Type 1 page. The real stream provides deterministic PRN6=1/IODSSR=1 and
 * PRN6=4/IODSSR=2 fixtures without inventing MASK contents. */
static int capture_prn_context_samples(const char *path,
                                       prn_context_sample_t samples[5])
{
    raw_t *raw=NULL;
    uint8_t (*latest_type1)[MAXRAWLEN]=NULL;
    int latest_len[64]={0};
    FILE *fp=NULL;
    int data,before,complete,type,prn6,ret,total,found=0,ok=0;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(latest_type1=(uint8_t (*)[MAXRAWLEN])calloc(64,MAXRAWLEN))||
        !(fp=fopen(path,"rb"))||!init_raw(raw,STRFMT_SINO)) goto done;

    while (found<2&&(data=fgetc(fp))!=EOF) {
        before=raw->nbyte;
        ret=input_raw(raw,STRFMT_SINO,(uint8_t)data);
        complete=before>0&&raw->nbyte==0&&raw->len>=SINO_HEADER_LEN&&
                 before+1==raw->len+4;
        if (!complete) continue;

        type=b2b_type(raw);
        prn6=b2b_prn6(raw);
        total=raw->len+4;
        if (type==1&&prn6>=0&&prn6<64) {
            memcpy(latest_type1[prn6],raw->buff,(size_t)total);
            latest_len[prn6]=total;
        }
        else if (type==4&&ret==20&&(prn6==1||prn6==4)&&
                 latest_len[prn6]&&!samples[prn6].type4_len) {
            memcpy(samples[prn6].type1,latest_type1[prn6],
                   (size_t)latest_len[prn6]);
            samples[prn6].type1_len=latest_len[prn6];
            memcpy(samples[prn6].type4,raw->buff,(size_t)total);
            samples[prn6].type4_len=total;
            found++;
        }
        clear_updates(raw);
    }
    ok=found==2;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(latest_type1);
    if (fp) fclose(fp);
    return ok;
}

/* After both source MASKs have arrived, each Type 4 page must use the MASK
 * carried by its own PRN6 rather than whichever source sent Type 1 last. */
static int test_prn6_context_bank(const prn_context_sample_t samples[5],
                                  int *prn6_context_bank)
{
    raw_t *raw=NULL;
    const B2bssr_t *source1,*source4;
    int sat,sat1=0,sat4=0;
    int ret,ok=0;

    *prn6_context_bank=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !init_raw(raw,STRFMT_SINO)) goto done;

    if (feed_frame(raw,samples[1].type1,samples[1].type1_len)!=20||
        feed_frame(raw,samples[4].type1,samples[4].type1_len)!=20) goto done;

    clear_updates(raw);
    ret=feed_frame(raw,samples[1].type4,samples[1].type4_len);
    if (ret!=20||any_update(raw)) goto done;
    for (sat=1;sat<=MAXSAT;sat++) {
        source1=sino_b2b_source_product(raw,1,sat);
        if (source1&&source1->t0[2].time) {sat1=sat; break;}
    }
    if (!sat1) goto done;

    clear_updates(raw);
    ret=feed_frame(raw,samples[4].type4,samples[4].type4_len);
    if (ret!=20||any_update(raw)) goto done;
    for (sat=1;sat<=MAXSAT;sat++) {
        source4=sino_b2b_source_product(raw,4,sat);
        if (source4&&source4->t0[2].time) {sat4=sat; break;}
    }
    if (!sat4) goto done;

    /* Re-publish PRN6=1's MASK. PRN6=4 must retain its own stored context. */
    clear_updates(raw);
    if (feed_frame(raw,samples[1].type1,samples[1].type1_len)!=20) goto done;
    ret=feed_frame(raw,samples[4].type4,samples[4].type4_len);
    source1=sino_b2b_source_product(raw,1,sat1);
    source4=sino_b2b_source_product(raw,4,sat4);
    if (ret!=20||any_update(raw)||!source1||!source4||
        !source1->t0[2].time||!source4->t0[2].time) goto done;

    *prn6_context_bank=1;
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    return ok;
}

/* Type 4 publication is per-satellite atomic. A reserved raw=-16383 record
 * must leave the prior complete clock tuple untouched. A raw=0 record is a
 * real product: it advances the epoch, publishes update=1 and stores 0.0 m. */
static int test_type4_clock_state(const prn_context_sample_t *sample,
                                  int *clock_atomic, int *clock_zero_valid,
                                  int *clock_missing_distinct)
{
    raw_t *raw=NULL;
    uint8_t *work=NULL;
    const B2bmask_t *mask;
    const B2bssr_t *source;
    B2bssr_t before;
    uint32_t crc,sow;
    int subtype,record=-1,mask_index,sat=0,c0,pos,payload_len,ret,i,ok=0;

    *clock_atomic=*clock_zero_valid=*clock_missing_distinct=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(work=(uint8_t *)malloc(MAXRAWLEN))||
        !init_raw(raw,STRFMT_SINO)) goto done;
    if (feed_frame(raw,sample->type1,sample->type1_len)!=20||
        !(mask=sino_b2b_mask(raw))) goto done;

    subtype=(int)getbitu(sample->type4,SINO_SUBTYPE_POS,5);
    for (i=0;i<23;i++) {
        mask_index=subtype*23+i;
        pos=SINO_CLOCK_RECORD_POS+i*18+3;
        c0=(int)getbits(sample->type4,pos,15);
        if (mask_index>=0&&mask_index<mask->satnum&&
            mask_index<B2B_MAXSAT&&abs(c0)<16383) {
            sat=mask->satno[mask_index];
            record=i;
            break;
        }
    }
    if (record<0||sat<=0||sat>MAXSAT) goto done;

    source=sino_b2b_source_product(raw,1,sat);
    *clock_missing_distinct=!source&&
                            !raw->nav.B2bssr[sat].t0[2].time&&
                            !raw->nav.B2bssr[sat].update&&
                            raw->nav.B2bssr[sat].dclk[0]==0.0;
    if (feed_frame(raw,sample->type4,sample->type4_len)!=20||
        raw->nav.B2bssr[sat].update||
        !(source=sino_b2b_source_product(raw,1,sat))) goto done;
    before=*source;

    payload_len=sample->type4_len-4;
    sow=getbitu(sample->type4,SINO_DATA_POS,17);
    pos=SINO_CLOCK_RECORD_POS+record*18+3;

    memcpy(work,sample->type4,(size_t)sample->type4_len);
    set_unsigned_bits(work,SINO_DATA_POS,17,(sow+1)%86400);
    set_unsigned_bits(work,pos,15,0x4001u); /* signed 15-bit -16383 */
    crc=rtk_crc32(work,payload_len);
    put_u4(work+payload_len,crc);
    ret=feed_frame(raw,work,sample->type4_len);
    source=sino_b2b_source_product(raw,1,sat);
    *clock_atomic=ret==20&&
                  source&&source->t0[2].time==before.t0[2].time&&
                  source->t0[2].sec==before.t0[2].sec&&
                  source->sow==before.sow&&
                  source->verify_sow==before.verify_sow&&
                  source->iodssr[2]==before.iodssr[2]&&
                  source->iodp[0]==before.iodp[0]&&
                  source->iodcorr[1]==before.iodcorr[1]&&
                  source->dclk[0]==before.dclk[0]&&
                  !raw->nav.B2bssr[sat].update;

    memcpy(work,sample->type4,(size_t)sample->type4_len);
    set_unsigned_bits(work,SINO_DATA_POS,17,(sow+2)%86400);
    set_unsigned_bits(work,pos,15,0u);
    crc=rtk_crc32(work,payload_len);
    put_u4(work+payload_len,crc);
    ret=feed_frame(raw,work,sample->type4_len);
    source=sino_b2b_source_product(raw,1,sat);
    *clock_zero_valid=ret==20&&source&&source->t0[2].time&&
                      source->sow==(int)((sow+2)%86400)&&
                      source->dclk[0]==0.0&&
                      !raw->nav.B2bssr[sat].update;
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(work);
    return ok;
}

static int published_fixture_ready(const raw_t *raw,
                                   const coherent_fixture_t *fixture,
                                   int prn6, int variant)
{
    const B2bssr_t *ssr;

    if (!raw||!fixture) return 0;
    ssr=&raw->nav.B2bssr[fixture->target_sat];
    return ssr->source_valid&&ssr->source_prn6==prn6&&
           sino_b2b_selected_source(raw,fixture->target_sat)==prn6&&
           fixture_identity_matches(ssr,fixture,variant)&&
           b2b_orbit_clock_ready(raw->time,ssr,NULL,NULL)&&
           publication_matches_source(raw,fixture->target_sat,ssr);
}

/* An orbit from PRN6=2 and a clock from PRN6=4 must remain private partial
 * products. They may not be assembled in the legacy shared product slot. */
static int test_cross_source_reject(const coherent_fixture_t *fixture,
                                    int *cross_source_reject)
{
    raw_t *raw=NULL;
    const B2bssr_t *source2,*source4;
    int ret2,ret4,ok=0;

    *cross_source_reject=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !init_raw(raw,STRFMT_SINO)) goto done;
    if (feed_fixture_frame(raw,fixture,1,2,0)!=20||
        feed_fixture_frame(raw,fixture,1,4,0)!=20) goto done;

    clear_updates(raw);
    ret2=feed_fixture_frame(raw,fixture,2,2,0);
    ret4=feed_fixture_frame(raw,fixture,4,4,0);
    source2=sino_b2b_source_product(raw,2,fixture->target_sat);
    source4=sino_b2b_source_product(raw,4,fixture->target_sat);
    *cross_source_reject=ret2==20&&ret4==20&&!any_update(raw)&&
        sino_b2b_selected_source(raw,fixture->target_sat)==-1&&
        source2&&source2->t0[0].time&&!source2->t0[2].time&&
        source4&&!source4->t0[0].time&&source4->t0[2].time&&
        !raw->nav.B2bssr[fixture->target_sat].source_valid;
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    return ok;
}

/* A new-IOD orbit temporarily makes its source incomplete. The previously
 * published tuple must remain untouched until the matching clock arrives. */
static int test_atomic_identity_switch(const coherent_fixture_t *fixture,
                                       int *single_source_ready,
                                       int *old_product_retained,
                                       int *atomic_identity_switch)
{
    raw_t *raw=NULL;
    B2bssr_t before;
    int ret,ok=0;

    *single_source_ready=*old_product_retained=*atomic_identity_switch=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !init_raw(raw,STRFMT_SINO)) goto done;
    if (!feed_complete_source(raw,fixture,1,0)) goto done;
    *single_source_ready=raw->nav.B2bssr[fixture->target_sat].update&&
                         published_fixture_ready(raw,fixture,1,0);
    clear_updates(raw);
    before=raw->nav.B2bssr[fixture->target_sat];

    if (feed_fixture_frame(raw,fixture,1,1,1)!=20) goto done;
    ret=feed_fixture_frame(raw,fixture,2,1,1);
    *old_product_retained=ret==20&&
        !raw->nav.B2bssr[fixture->target_sat].update&&
        same_b2b_products(&before,
                          &raw->nav.B2bssr[fixture->target_sat])&&
        fixture_identity_matches(&raw->nav.B2bssr[fixture->target_sat],
                                 fixture,0)&&
        b2b_orbit_clock_ready(raw->time,
                 &raw->nav.B2bssr[fixture->target_sat],NULL,NULL)&&
        sino_b2b_selected_source(raw,fixture->target_sat)==1;

    ret=feed_fixture_frame(raw,fixture,4,1,1);
    *atomic_identity_switch=ret==20&&
        raw->nav.B2bssr[fixture->target_sat].update&&
        published_fixture_ready(raw,fixture,1,1);
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    return ok;
}

/* Start from a conflicting current source. Two agreeing sources must override
 * it, and the third agreeing source must not destabilize the selected source. */
static int test_majority_identity(const coherent_fixture_t *fixture,
                                  int *majority_identity)
{
    raw_t *raw=NULL;
    int tied_current_kept,majority_switched,ok=0;

    *majority_identity=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !init_raw(raw,STRFMT_SINO)) goto done;
    if (!feed_complete_source(raw,fixture,4,1)||
        !published_fixture_ready(raw,fixture,4,1)) goto done;

    clear_updates(raw);
    if (!feed_complete_source(raw,fixture,1,0)) goto done;
    tied_current_kept=sino_b2b_selected_source(raw,fixture->target_sat)==4&&
                      published_fixture_ready(raw,fixture,4,1);

    clear_updates(raw);
    if (!feed_complete_source(raw,fixture,2,0)) goto done;
    majority_switched=raw->nav.B2bssr[fixture->target_sat].update&&
        sino_b2b_selected_source(raw,fixture->target_sat)==1&&
        published_fixture_ready(raw,fixture,1,0);

    clear_updates(raw);
    if (!feed_complete_source(raw,fixture,3,0)) goto done;
    *majority_identity=tied_current_kept&&majority_switched&&
        sino_b2b_selected_source(raw,fixture->target_sat)==1&&
        published_fixture_ready(raw,fixture,1,0);
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    return ok;
}

/* Change every orbit metadata field before placing the reserved -16383 value
 * in the target record. None of those tentative values may reach the bank. */
static int test_type2_invalid_atomic(const coherent_fixture_t *fixture,
                                     int *type2_invalid_atomic)
{
    raw_t *raw=NULL;
    uint8_t frame[MAXRAWLEN];
    const B2bssr_t *source;
    B2bssr_t before;
    uint32_t sow;
    int total,pos,ret,ok=0;

    *type2_invalid_atomic=0;
    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !init_raw(raw,STRFMT_SINO)) goto done;
    if (!feed_complete_source(raw,fixture,1,0)||
        !(source=sino_b2b_source_product(raw,1,fixture->target_sat))) {
        goto done;
    }
    before=*source;
    clear_updates(raw);
    if (!make_fixture_frame(fixture,2,1,0,frame,&total)) goto done;

    sow=getbitu(frame,SINO_DATA_POS,17);
    pos=SINO_ORBIT_RECORD_POS+
        fixture->orbit_record*SINO_ORBIT_RECORD_BITS;
    set_unsigned_bits(frame,SINO_DATA_POS,17,(sow+1)%86400);
    set_unsigned_bits(frame,pos+9,10,(uint32_t)((fixture->iodn+1)&1023));
    set_unsigned_bits(frame,pos+19,3,
                      (uint32_t)((fixture->iodcorr+1)&7));
    set_unsigned_bits(frame,pos+22,15,0x4001u); /* signed 15-bit -16383 */
    update_frame_crc(frame,total);

    ret=feed_frame(raw,frame,total);
    source=sino_b2b_source_product(raw,1,fixture->target_sat);
    *type2_invalid_atomic=ret==20&&source&&
        !memcmp(source,&before,sizeof(before))&&
        !raw->nav.B2bssr[fixture->target_sat].update&&
        published_fixture_ready(raw,fixture,1,0);
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    return ok;
}

/* Targeted negative tests distinguish CRC rejection from header-length guard. */
static int test_frame_guards(const char *path, int *crc_reject,
                             int *length_reject)
{
    raw_t *raw=NULL;
    uint8_t *frame=NULL;
    int frame_len,ret,ok=0;

    if (!(raw=(raw_t *)calloc(1,sizeof(*raw)))||
        !(frame=(uint8_t *)malloc(MAXRAWLEN))) goto done;
    if (!read_first_frame(path,frame,&frame_len)||
        !init_raw(raw,STRFMT_SINO)) goto done;

    frame[SINO_HEADER_LEN]^=0x01; /* payload changes but stored CRC does not */
    ret=feed_frame(raw,frame,frame_len);
    *crc_reject=ret==-1;
    free_raw(raw);
    memset(raw,0,sizeof(*raw));

    if (!init_raw(raw,STRFMT_SINO)) goto done;
    frame[8]=0xFF;
    frame[9]=0xFF;
    ret=feed_frame(raw,frame,10);
    *length_reject=ret==-1&&raw->nbyte==0&&raw->len==0;
    ok=1;

done:
    if (raw) {
        if (raw->format==STRFMT_SINO) free_raw(raw);
        free(raw);
    }
    free(frame);
    return ok;
}

static void print_time_value(const char *label, gtime_t time)
{
    double ep[6];

    time2epoch(time,ep);
    printf("%s %04d-%02d-%02d %02d:%02d:%04.1f\n",label,
           (int)ep[0],(int)ep[1],(int)ep[2],(int)ep[3],(int)ep[4],ep[5]);
}

static void print_results(const product_stats_t *products,
                          const raw_stats_t *raws, int context_isolated,
                          int index_zero_unchanged, int crc_reject,
                          int length_reject, int iodssr_reject,
                           int iodp_reject, int mask_gate_reject,
                           int prn6_context_bank, int clock_atomic,
                           int clock_zero_valid, int clock_missing_distinct,
                           const bridge_stats_t *bridge)
{
    long unknown=raws->frames-raws->types[1]-raws->types[2]-
                 raws->types[3]-raws->types[4];
    long source_total=0;
    const B2bssr_t *orbit=&products->sample_ssr[0];
    const B2bssr_t *code=&products->sample_ssr[1];
    const B2bssr_t *clock=&products->sample_ssr[2];
    int i;

    for (i=0;i<64;i++) source_total+=products->source_publications[i];

    printf("RAW_FRAMES %ld\n",raws->frames);
    printf("RAW_TYPE_1 %ld\n",raws->types[1]);
    printf("RAW_TYPE_2 %ld\n",raws->types[2]);
    printf("RAW_TYPE_3 %ld\n",raws->types[3]);
    printf("RAW_TYPE_4 %ld\n",raws->types[4]);
    printf("RAW_TYPE_OTHER %ld\n",unknown);
    printf("MASK_COUNT %ld\n",products->accepted[0]);
    printf("ORBIT_COUNT %ld\n",products->accepted[1]);
    printf("CODE_BIAS_COUNT %ld\n",products->accepted[2]);
    printf("CLOCK_COUNT %ld\n",products->accepted[3]);
    printf("CRC_ERROR %ld\n",raws->crc_errors);
    printf("FRAME_ERROR %ld\n",raws->frame_errors);
    printf("OTHER_PRODUCT_ERROR %ld\n",raws->other_product_errors);
    printf("FILE_PATH_ERROR %ld\n",products->file_errors);
    printf("COHERENT_PUBLICATIONS %ld\n",products->coherent_publications);
    printf("INVALID_PUBLICATIONS %ld\n",products->invalid_publications);
    printf("MISSING_SOURCE_PUBLICATIONS %ld\n",
           products->missing_source_publications);
    printf("SOURCE_PROVENANCE_PUBLICATIONS %ld\n",source_total);

    if (products->have_mask) {
        printf("MASK_FIRST iodssr=%d iodp=%d satnum=%d\n",
               products->first_mask.IOD_SSR,products->first_mask.IODP,
               products->first_mask.satnum);
        print_time_value("MASK_RECV_TIME",products->first_mask.recv_time);
        print_time_value("MASK_REF_TIME",products->first_mask.ref_time);
    }
    printf("ORBIT_SAMPLE sat=%d iodssr=%d iodn=%d iodcorr=%u ura=%d "
           "deph=%.4f,%.4f,%.4f\n",products->sample_sat[0],
           orbit->iodssr[0],orbit->iodn,(unsigned int)orbit->iodcorr[0],
           orbit->ura,orbit->deph[0],orbit->deph[1],orbit->deph[2]);
    printf("CODE_SAMPLE sat=%d iodssr=%d code=%d bias=%.4f\n",
           products->sample_sat[1],code->iodssr[1],products->sample_code,
           products->sample_bias);
    printf("CLOCK_SAMPLE sat=%d iodssr=%d iodp=%d iodcorr=%u dclk=%.4f\n",
           products->sample_sat[2],clock->iodssr[2],clock->iodp[0],
           (unsigned int)clock->iodcorr[1],clock->dclk[0]);
    printf("CODE_BIAS_VALID_INSTANCES %ld\n",products->valid_biases);
    printf("CODE_BIAS_VALID_ZERO %d\n",products->valid_zero_bias);
    printf("CONTEXT_ISOLATION %d\n",context_isolated);
    printf("INDEX_ZERO_UNCHANGED %d\n",index_zero_unchanged);
    printf("CRC_REJECT %d\n",crc_reject);
    printf("LENGTH_REJECT %d\n",length_reject);
    printf("IODSSR_REJECT %d\n",iodssr_reject);
    printf("IODP_REJECT %d\n",iodp_reject);
    printf("MASK_GATE_REJECT %d\n",mask_gate_reject);
    printf("PRN6_CONTEXT_BANK %d\n",prn6_context_bank);
    printf("TYPE4_CLOCK_ATOMIC %d\n",clock_atomic);
    printf("TYPE4_CLOCK_ZERO_VALID %d\n",clock_zero_valid);
    printf("TYPE4_CLOCK_MISSING_DISTINCT %d\n",clock_missing_distinct);
    printf("BRIDGE_EVENTS %ld,%ld,%ld,%ld\n",bridge->events[0],
           bridge->events[1],bridge->events[2],bridge->events[3]);
    printf("BRIDGE_PENDING_SATS %ld,%ld,%ld,%ld\n",
           bridge->pending_sats[0],bridge->pending_sats[1],
           bridge->pending_sats[2],bridge->pending_sats[3]);
    printf("BRIDGE_TRANSFERRED_SATS %ld,%ld,%ld,%ld\n",
           bridge->transferred_sats[0],bridge->transferred_sats[1],
           bridge->transferred_sats[2],bridge->transferred_sats[3]);
    printf("BRIDGE_SECOND_CALL_UPDATES %ld\n",bridge->second_call_updates);
    printf("BRIDGE_RAW_CONSUMED %d\n",bridge->raw_consumed);
    printf("BRIDGE_FIELDS_MATCH %d\n",bridge->fields_match);
    printf("BRIDGE_CBIAS_VALID_MATCH %d\n",bridge->cbias_valid_match);
    printf("BRIDGE_CBIAS_VALID_COUNTS %ld,%ld\n",
           bridge->cbias_valid_raw,bridge->cbias_valid_nav);
    printf("BRIDGE_VALID_ZERO_COPIED %d\n",bridge->valid_zero_copied);
    printf("BRIDGE_NAV_UPDATE_VISIBLE %d\n",bridge->nav_update_visible);
    printf("BRIDGE_NAV_UPDATE_SURVIVES_SECOND_CALL %d\n",
           bridge->nav_update_survives_second_call);
    printf("BRIDGE_INDEX_ZERO_UNCHANGED %d\n",
           bridge->index_zero_unchanged);
    printf("BRIDGE_ERROR %d\n",bridge->errors);
}

int main(int argc, char **argv)
{
    static const receiver_baseline_t baselines[]={
        {"2026-05-23",{6241,24713,26700,149843},
         {6241,24697,26684,149749},299610,92113,4,2,
         {-0.0224,0.0128,0.0832},0.0528},
        {"2026-06-11",{6427,25461,27019,154044},
         {6427,25442,27002,153959},308044,95093,12,5,
         {0.0016,0.0832,0.0768},0.1216}
    };
    product_stats_t products={0};
    raw_stats_t raws={0};
    bridge_stats_t bridge={0};
    prn_context_sample_t prn_samples[5]={0};
    coherent_fixture_t fixture={0};
    long unknown,publication_total=0;
    int i,ok=1,context_isolated=0,index_zero_unchanged=0;
    int crc_reject=0,length_reject=0;
    int iodssr_reject=0,iodp_reject=0,mask_gate_reject=0;
    int prn6_context_bank=0,clock_atomic=0,clock_zero_valid=0;
    int clock_missing_distinct=0;
    int fixture_ok=0,cross_source_reject=0,single_source_ready=0;
    int old_product_retained=0,atomic_identity_switch=0;
    int majority_identity=0,type2_invalid_atomic=0;
    const receiver_baseline_t *baseline=NULL;

    if (argc!=2) {
        fprintf(stderr,"Usage: %s <Sino_B2b_raw>\n",argv[0]);
        return 1;
    }
    ok&=replay_products(argv[1],&products,&context_isolated,
                        &index_zero_unchanged);
    ok&=replay_raw_bytes(argv[1],&raws);
    ok&=test_frame_guards(argv[1],&crc_reject,&length_reject);
    ok&=test_mask_gate(argv[1],&mask_gate_reject);
    ok&=test_iod_gates(argv[1],&iodssr_reject,&iodp_reject);
    ok&=capture_prn_context_samples(argv[1],prn_samples);
    ok&=test_prn6_context_bank(prn_samples,&prn6_context_bank);
    ok&=test_type4_clock_state(&prn_samples[1],&clock_atomic,
                               &clock_zero_valid,&clock_missing_distinct);
    fixture_ok=capture_coherent_fixture(argv[1],&fixture);
    ok&=fixture_ok;
    if (fixture_ok) {
        ok&=test_cross_source_reject(&fixture,&cross_source_reject);
        ok&=test_atomic_identity_switch(&fixture,&single_source_ready,
                                        &old_product_retained,
                                        &atomic_identity_switch);
        ok&=test_majority_identity(&fixture,&majority_identity);
        ok&=test_type2_invalid_atomic(&fixture,&type2_invalid_atomic);
    }
    ok&=replay_nav_bridge(argv[1],&bridge);

    print_results(&products,&raws,context_isolated,index_zero_unchanged,
                   crc_reject,length_reject,iodssr_reject,iodp_reject,
                   mask_gate_reject,prn6_context_bank,clock_atomic,
                   clock_zero_valid,clock_missing_distinct,&bridge);
    printf("COHERENT_FIXTURE %d source=%d sat=%d iodssr=%d iodp=%d "
           "iodn=%d iodcorr=%d\n",fixture_ok,fixture.source_prn6,
           fixture.target_sat,fixture.iodssr,fixture.iodp,fixture.iodn,
           fixture.iodcorr);
    printf("CROSS_SOURCE_REJECT %d\n",cross_source_reject);
    printf("SINGLE_SOURCE_READY %d\n",single_source_ready);
    printf("OLD_PRODUCT_RETAINED %d\n",old_product_retained);
    printf("ATOMIC_IDENTITY_SWITCH %d\n",atomic_identity_switch);
    printf("MAJORITY_IDENTITY %d\n",majority_identity);
    printf("TYPE2_INVALID_ATOMIC %d\n",type2_invalid_atomic);

    for (i=0;i<(int)(sizeof(baselines)/sizeof(baselines[0]));i++) {
        if (raws.frames==baselines[i].frames) {
            baseline=baselines+i;
            break;
        }
    }
    printf("RECEIVER_BASELINE %s\n",baseline?baseline->name:"UNKNOWN");

    ok&=baseline!=NULL;
    for (i=0;baseline&&i<4;i++) {
        ok&=raws.types[i+1]==baseline->raw[i];
        ok&=products.accepted[i]==baseline->products[i];
    }
    unknown=raws.frames-raws.types[1]-raws.types[2]-
            raws.types[3]-raws.types[4];
    ok&=baseline&&raws.frames==baseline->frames&&unknown==baseline->unknown;
    ok&=raws.crc_errors==0&&raws.frame_errors==0&&products.file_errors==0&&
        raws.other_product_errors==0;
    for (i=0;i<4;i++) publication_total+=products.update_sats[i];
    ok&=products.coherent_publications==publication_total&&
        products.coherent_publications>0&&
        products.invalid_publications==0&&
        products.missing_source_publications==0;
    ok&=products.have_mask&&products.first_mask.IOD_SSR==1&&
        products.first_mask.IODP==14&&products.first_mask.satnum==62&&
        products.first_mask.satno[0]==satno(SYS_CMP,6)&&
        products.first_mask.satno[29]==satno(SYS_CMP,42)&&
        products.first_mask.satno[30]==satno(SYS_GPS,1)&&
        products.first_mask.satno[61]==satno(SYS_GPS,32);
    ok&=products.sample_sat[0]==satno(SYS_CMP,6)&&
        products.sample_ssr[0].iodssr[0]==1&&
        baseline&&products.sample_ssr[0].iodn==baseline->orbit_iodn&&
        products.sample_ssr[0].iodcorr[0]==baseline->orbit_iodcorr&&
        products.sample_ssr[0].ura==31&&
        fabs(products.sample_ssr[0].deph[0]-baseline->deph[0])<1E-8&&
        fabs(products.sample_ssr[0].deph[1]-baseline->deph[1])<1E-8&&
        fabs(products.sample_ssr[0].deph[2]-baseline->deph[2])<1E-8;
    ok&=products.sample_sat[1]==satno(SYS_CMP,6)&&
        products.sample_ssr[1].iodssr[1]==1&&
        products.sample_code==CODE_L1P&&
        fabs(products.sample_bias-1.666)<1E-6;
    ok&=products.sample_sat[2]==satno(SYS_CMP,6)&&
        products.sample_ssr[2].iodssr[2]==1&&
        products.sample_ssr[2].iodp[0]==14&&
        baseline&&products.sample_ssr[2].iodcorr[1]==baseline->orbit_iodcorr&&
        fabs(products.sample_ssr[2].dclk[0]-baseline->dclk)<1E-8;
    ok&=products.valid_biases>0&&products.valid_zero_bias;
    ok&=context_isolated&&index_zero_unchanged&&crc_reject&&length_reject&&
        iodssr_reject&&iodp_reject&&mask_gate_reject&&prn6_context_bank&&
        clock_atomic&&clock_zero_valid&&clock_missing_distinct&&fixture_ok&&
        cross_source_reject&&single_source_ready&&old_product_retained&&
        atomic_identity_switch&&majority_identity&&type2_invalid_atomic;
    for (i=0;i<4;i++) {
        ok&=baseline&&bridge.events[i]==baseline->products[i];
        ok&=bridge.pending_sats[i]==bridge.transferred_sats[i];
        ok&=bridge.pending_sats[i]==products.update_sats[i];
    }
    ok&=bridge.pending_sats[0]==0&&bridge.second_call_updates==0&&
        bridge.cbias_valid_raw>0&&
        bridge.cbias_valid_raw==bridge.cbias_valid_nav&&
        bridge.raw_consumed&&bridge.fields_match&&
        bridge.cbias_valid_match&&bridge.valid_zero_copied&&
        bridge.nav_update_visible&&bridge.nav_update_survives_second_call&&
        bridge.index_zero_unchanged&&bridge.errors==0;

    printf("SINO_RECEIVER_TEST %s\n",ok?"PASS":"FAIL");
    return ok?0:1;
}
