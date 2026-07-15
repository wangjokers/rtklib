#include "rtklib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SINO_HEADER_LEN 28
#define SINO_TYPE_POS   (SINO_HEADER_LEN*8+44)
#define SINO_DATA_POS   (SINO_HEADER_LEN*8+50)
#define SINO_IODSSR_POS (SINO_DATA_POS+21)
#define SINO_IODP_POS   (SINO_DATA_POS+23)

typedef struct {
    long accepted[4];
    long update_sats[4];
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

static int b2b_type(const raw_t *raw)
{
    if (!raw||raw->len<SINO_HEADER_LEN||
        SINO_TYPE_POS+6>raw->len*8) return -1;
    return (int)getbitu(raw->buff,SINO_TYPE_POS,6);
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
        if (!stats->sample_sat[index-1]) {
            stats->sample_sat[index-1]=sat;
            stats->sample_ssr[index-1]=*ssr;
        }
        if (type!=3) continue;
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
                          int iodp_reject, int mask_gate_reject)
{
    long unknown=raws->frames-raws->types[1]-raws->types[2]-
                 raws->types[3]-raws->types[4];
    const B2bssr_t *orbit=&products->sample_ssr[0];
    const B2bssr_t *code=&products->sample_ssr[1];
    const B2bssr_t *clock=&products->sample_ssr[2];

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
}

int main(int argc, char **argv)
{
    static const long expected_raw[4]={6241,24713,26700,149843};
    static const long expected_products[4]={6241,14052,15109,86076};
    product_stats_t products={0};
    raw_stats_t raws={0};
    long unknown;
    int i,ok=1,context_isolated=0,index_zero_unchanged=0;
    int crc_reject=0,length_reject=0;
    int iodssr_reject=0,iodp_reject=0,mask_gate_reject=0;

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

    print_results(&products,&raws,context_isolated,index_zero_unchanged,
                  crc_reject,length_reject,iodssr_reject,iodp_reject,
                  mask_gate_reject);

    for (i=0;i<4;i++) {
        ok&=raws.types[i+1]==expected_raw[i];
        ok&=products.accepted[i]==expected_products[i];
    }
    unknown=raws.frames-raws.types[1]-raws.types[2]-
            raws.types[3]-raws.types[4];
    ok&=raws.frames==299610&&unknown==92113;
    ok&=raws.crc_errors==0&&raws.frame_errors==0&&products.file_errors==0&&
        raws.other_product_errors==0;
    ok&=products.have_mask&&products.first_mask.IOD_SSR==1&&
        products.first_mask.IODP==14&&products.first_mask.satnum==62&&
        products.first_mask.satno[0]==satno(SYS_CMP,6)&&
        products.first_mask.satno[29]==satno(SYS_CMP,42)&&
        products.first_mask.satno[30]==satno(SYS_GPS,1)&&
        products.first_mask.satno[61]==satno(SYS_GPS,32);
    ok&=products.sample_sat[0]==satno(SYS_CMP,6)&&
        products.sample_ssr[0].iodssr[0]==2&&
        products.sample_ssr[0].iodn==4&&
        products.sample_ssr[0].iodcorr[0]==2&&
        products.sample_ssr[0].ura==31&&
        fabs(products.sample_ssr[0].deph[0]+0.0224)<1E-8&&
        fabs(products.sample_ssr[0].deph[1]-0.0128)<1E-8&&
        fabs(products.sample_ssr[0].deph[2]-0.0896)<1E-8;
    ok&=products.sample_sat[1]==satno(SYS_CMP,6)&&
        products.sample_ssr[1].iodssr[1]==2&&
        products.sample_code==CODE_L1P&&
        fabs(products.sample_bias-1.666)<1E-6;
    ok&=products.sample_sat[2]==satno(SYS_CMP,6)&&
        products.sample_ssr[2].iodssr[2]==2&&
        products.sample_ssr[2].iodp[0]==14&&
        products.sample_ssr[2].iodcorr[1]==2&&
        fabs(products.sample_ssr[2].dclk[0]-0.0640)<1E-8;
    ok&=products.valid_biases>0&&products.valid_zero_bias;
    ok&=context_isolated&&index_zero_unchanged&&crc_reject&&length_reject&&
        iodssr_reject&&iodp_reject&&mask_gate_reject;

    printf("SINO_RECEIVER_TEST %s\n",ok?"PASS":"FAIL");
    return ok?0:1;
}
