/*------------------------------------------------------------------------------
* sinan.c : minimal SinoGNSS PPP-B2b receiver decoder
*
* This receiver layer accepts SinoGNSS/K803 AA 44 12 binary frames and writes
* decoded PPP-B2b products to raw->nav.B2bssr[sat]. It deliberately stops at
* raw_t: main-nav replay, realtime-server updates and PPP use belong to later
* stages.
*-----------------------------------------------------------------------------*/
#include "../rtklib.h"

#define SINO_SYNC1       0xAA
#define SINO_SYNC2       0x44
#define SINO_SYNC3       0x12
#define SINO_HEADER_LEN  28
#define SINO_MSG_B2B     1697

#define B2B_CODE_MODE_COUNT 15
#define SINO_CTX_MAGIC      0x53423242u
#define SINO_PRN6_COUNT     64

typedef struct {
    uint32_t magic;
    /* Type 1 and decoded products are source-local. Only a coherent selected
     * source is published to the legacy raw->nav.B2bssr[] interface. */
    B2bmask_t mask[SINO_PRN6_COUNT];
    B2bssr_t *product[SINO_PRN6_COUNT];
    int selected_prn6[MAXSAT+1];
    int latest_mask_prn6;
} sino_b2b_ctx_t;

static const int b2b_bds_codebias_mode[B2B_CODE_MODE_COUNT]={
    CODE_L2I,CODE_L1D,CODE_L1P,CODE_NONE,CODE_L5D,
    CODE_L5P,CODE_NONE,CODE_L7I,CODE_L7Q,CODE_NONE,
    CODE_NONE,CODE_NONE,CODE_L6I,CODE_NONE,CODE_NONE
};
static const int b2b_gps_codebias_mode[B2B_CODE_MODE_COUNT]={
    CODE_L1C,CODE_L1P,CODE_NONE,CODE_NONE,CODE_L1L,
    CODE_L1X,CODE_NONE,CODE_L2L,CODE_L2X,CODE_NONE,
    CODE_NONE,CODE_L5I,CODE_L5Q,CODE_L5X,CODE_NONE
};
static const int b2b_glo_codebias_mode[B2B_CODE_MODE_COUNT]={
    CODE_L1C,CODE_L1P,CODE_L2C,CODE_NONE,CODE_NONE,
    CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,
    CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE
};
static const int b2b_gal_codebias_mode[B2B_CODE_MODE_COUNT]={
    CODE_NONE,CODE_L1B,CODE_L1X,CODE_NONE,CODE_L5Q,
    CODE_L5I,CODE_NONE,CODE_L7I,CODE_L7Q,CODE_NONE,
    CODE_NONE,CODE_L6C,CODE_NONE,CODE_NONE,CODE_NONE
};

static uint16_t get_u2(const uint8_t *p)
{
    return (uint16_t)p[0]|((uint16_t)p[1]<<8);
}

static uint32_t get_u4(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|
           ((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}

static sino_b2b_ctx_t *get_context(const raw_t *raw)
{
    sino_b2b_ctx_t *ctx;

    if (!raw||!raw->rcv_data) return NULL;
    ctx=(sino_b2b_ctx_t *)raw->rcv_data;
    return ctx->magic==SINO_CTX_MAGIC?ctx:NULL;
}

static B2bmask_t *get_prn6_mask(sino_b2b_ctx_t *ctx, int prn6)
{
    if (!ctx||prn6<0||prn6>=SINO_PRN6_COUNT) return NULL;
    return &ctx->mask[prn6];
}

static B2bssr_t *get_prn6_products(sino_b2b_ctx_t *ctx, int prn6,
                                    int allocate)
{
    if (!ctx||prn6<0||prn6>=SINO_PRN6_COUNT) return NULL;
    if (!ctx->product[prn6]&&allocate) {
        ctx->product[prn6]=(B2bssr_t *)calloc(MAXSAT+1,sizeof(B2bssr_t));
    }
    return ctx->product[prn6];
}

static int same_product_identity(const B2bssr_t *a, const B2bssr_t *b)
{
    return a&&b&&a->iodssr[0]==b->iodssr[0]&&
           a->iodcorr[0]==b->iodcorr[0]&&a->iodn==b->iodn&&
           a->iodp[0]==b->iodp[0];
}

static int source_candidate_ready(gtime_t time, const B2bssr_t *ssr)
{
    double variance;

    return b2b_orbit_clock_ready(time,ssr,NULL,NULL)&&
           b2b_urai_variance(ssr->ura,&variance);
}

static int newer_candidate(const B2bssr_t *candidate,
                           const B2bssr_t *current)
{
    double dt;

    if (!current) return 1;
    dt=timediff(candidate->t0[2],current->t0[2]);
    if (dt>DTTOL) return 1;
    if (dt<-DTTOL) return 0;
    return timediff(candidate->t0[0],current->t0[0])>DTTOL;
}

/* Select one complete source without ever merging product components. */
static int select_source(const sino_b2b_ctx_t *ctx, gtime_t time, int sat,
                         int *support)
{
    int ready[SINO_PRN6_COUNT]={0};
    int current,best=-1,best_support=0,prn6,other,count;

    if (support) *support=0;
    if (!ctx||sat<=0||sat>MAXSAT) return -1;
    current=ctx->selected_prn6[sat];

    for (prn6=0;prn6<SINO_PRN6_COUNT;prn6++) {
        if (!ctx->product[prn6]) continue;
        ready[prn6]=source_candidate_ready(time,
                                           ctx->product[prn6]+sat);
    }
    for (prn6=0;prn6<SINO_PRN6_COUNT;prn6++) {
        const B2bssr_t *candidate;

        if (!ready[prn6]) continue;
        candidate=ctx->product[prn6]+sat;
        count=0;
        for (other=0;other<SINO_PRN6_COUNT;other++) {
            if (ready[other]&&same_product_identity(candidate,
                                      ctx->product[other]+sat)) count++;
        }
        if (count>best_support) {
            best=prn6;
            best_support=count;
        }
        else if (count==best_support) {
            if (prn6==current) {
                best=prn6;
            }
            else if (best!=current&&
                     (newer_candidate(candidate,ctx->product[best]+sat)||
                      (!newer_candidate(ctx->product[best]+sat,candidate)&&
                       prn6<best))) {
                best=prn6;
            }
        }
    }
    if (support) *support=best_support;
    return best;
}

/* Publish a complete candidate atomically. Partial source updates stay private
 * until one source has a current coherent orbit/clock tuple. */
static int publish_selected(raw_t *raw, sino_b2b_ctx_t *ctx, int sat,
                            int updated_prn6)
{
    B2bssr_t next;
    int previous,selected,support;

    if (!raw||!ctx||sat<=0||sat>MAXSAT) return 0;
    previous=ctx->selected_prn6[sat];
    selected=select_source(ctx,raw->time,sat,&support);
    if (selected<0) return 0;
    if (selected==previous&&selected!=updated_prn6) return 0;

    next=ctx->product[selected][sat];
    if (!next.t0[1].time||next.iodssr[1]!=next.iodssr[0]) {
        memset(next.cbias_valid,0,sizeof(next.cbias_valid));
    }
    next.source_prn6=(uint8_t)selected;
    next.source_valid=1;
    next.update=1;
    raw->nav.B2bssr[sat]=next;
    ctx->selected_prn6[sat]=selected;

    if (selected!=previous) {
        trace(3,"sino b2b source select: sat=%2d source=%d previous=%d "
              "support=%d iodssr=%d iodcorr=%u iodn=%d iodp=%d\n",
              sat,selected,previous,support,
              next.iodssr[0],(unsigned int)next.iodcorr[0],next.iodn,
              next.iodp[0]);
    }
    return 1;
}

/* Search the byte stream for the SinoGNSS AA 44 12 sync sequence. */
static int sync_sino(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1];
    buff[1]=buff[2];
    buff[2]=data;
    return buff[0]==SINO_SYNC1&&buff[1]==SINO_SYNC2&&
           buff[2]==SINO_SYNC3;
}

/* Guard every B2b bit-field read against the complete header+payload length. */
static int bit_range_valid(const raw_t *raw, int pos, int len)
{
    int total;

    if (!raw||pos<0||len<0||raw->len<0) return 0;
    total=raw->len*8;
    return pos<=total&&len<=total-pos;
}

static int verify_sod(gtime_t time)
{
    double ep[6];

    time2epoch(time,ep);
    return (int)(ep[3]*3600.0+ep[4]*60.0+ep[5]);
}

static const int *codebias_modes(int sat)
{
    switch (satsys(sat,NULL)) {
        case SYS_GPS: return b2b_gps_codebias_mode;
        case SYS_GLO: return b2b_glo_codebias_mode;
        case SYS_GAL: return b2b_gal_codebias_mode;
        case SYS_CMP: return b2b_bds_codebias_mode;
    }
    return NULL;
}

/* Decode B2b Type 1: receiver-local MASK and IOD context. */
static int decode_type1(raw_t *raw, int pos, int prn6)
{
    sino_b2b_ctx_t *ctx=get_context(raw);
    B2bmask_t *source_mask=get_prn6_mask(ctx,prn6);
    B2bmask_t mask;
    uint32_t sow;
    int i;

    if (!source_mask||!bit_range_valid(raw,pos,27+B2B_MAXSAT)) return -1;

    memset(&mask,0,sizeof(mask));
    sow=getbitu(raw->buff,pos,17); pos+=17;
    pos+=4; /* reserved */
    mask.IOD_SSR=(int)getbitu(raw->buff,pos,2); pos+=2;
    mask.IODP=(int)getbitu(raw->buff,pos,4); pos+=4;

    for (i=0;i<63;i++,pos++) mask.MASK_BD[i]=(int)getbitu(raw->buff,pos,1);
    for (i=0;i<37;i++,pos++) mask.MASK_GPS[i]=(int)getbitu(raw->buff,pos,1);
    for (i=0;i<37;i++,pos++) mask.MASK_GAL[i]=(int)getbitu(raw->buff,pos,1);
    for (i=0;i<37;i++,pos++) mask.MASK_GLO[i]=(int)getbitu(raw->buff,pos,1);

    mask.recv_time=raw->time;
    mask.ref_time=b2b_tod2time(raw->time,sow);
    if (!mask.ref_time.time) return -1;
    b2b_mask2satno(&mask);
    *source_mask=mask;
    ctx->latest_mask_prn6=prn6;
    return 20;
}

/* Decode B2b Type 2: six fixed orbit/URAI records. */
static int decode_type2(raw_t *raw, int pos, int prn6)
{
    sino_b2b_ctx_t *ctx=get_context(raw);
    B2bmask_t *mask=get_prn6_mask(ctx,prn6);
    B2bssr_t *products;
    gtime_t ref_time;
    uint32_t sow;
    int i,iod_ssr,updated=0;

    if (!mask||!bit_range_valid(raw,pos,23+6*69)) return -1;

    sow=getbitu(raw->buff,pos,17); pos+=17;
    pos+=4; /* reserved */
    iod_ssr=(int)getbitu(raw->buff,pos,2); pos+=2;
    if (mask->IOD_SSR<0||iod_ssr!=mask->IOD_SSR) return 0;

    ref_time=b2b_tod2time(raw->time,sow);
    if (!ref_time.time) return -1;
    if (!(products=get_prn6_products(ctx,prn6,1))) return -1;

    for (i=0;i<6;i++) {
        int slot=(int)getbitu(raw->buff,pos,9);
        int iodn,iodcorr,radial,in_track,cross,ura,sat;
        B2bssr_t next;

        pos+=9;
        iodn=(int)getbitu(raw->buff,pos,10); pos+=10;
        iodcorr=(int)getbitu(raw->buff,pos,3); pos+=3;
        radial=(int)getbits(raw->buff,pos,15); pos+=15;
        in_track=(int)getbits(raw->buff,pos,13); pos+=13;
        cross=(int)getbits(raw->buff,pos,13); pos+=13;
        ura=(int)getbitu(raw->buff,pos,6); pos+=6;

        sat=b2b_slot2satno(slot);
        if (sat<=0||sat>MAXSAT) continue;
        if (abs(radial)>=16383||abs(in_track)>=4095||abs(cross)>=4095) {
            continue;
        }

        next=products[sat];
        next.t0[0]=ref_time;
        next.sow=(int)sow;
        next.verify_sow=verify_sod(ref_time);
        next.iodssr[0]=iod_ssr;
        next.iodn=iodn;
        next.iodcorr[0]=(uint16_t)iodcorr;
        next.deph[0]=radial*0.0016;
        next.deph[1]=in_track*0.0064;
        next.deph[2]=cross*0.0064;
        next.ura=ura;
        next.source_prn6=(uint8_t)prn6;
        next.source_valid=1;
        next.update=0;
        products[sat]=next;
        publish_selected(raw,ctx,sat,prn6);
        updated++;
    }
    return updated?20:0;
}

/* Decode B2b Type 3: variable satellite/signal code-bias records. */
static int decode_type3(raw_t *raw, int pos, int prn6)
{
    sino_b2b_ctx_t *ctx=get_context(raw);
    B2bmask_t *mask=get_prn6_mask(ctx,prn6);
    B2bssr_t *products;
    gtime_t ref_time;
    uint32_t sow;
    int i,j,iod_ssr,satnum,updated=0;

    if (!mask||!bit_range_valid(raw,pos,28)) return -1;

    sow=getbitu(raw->buff,pos,17); pos+=17; /* seconds of day */
    pos+=4; /* reserved */
    iod_ssr=(int)getbitu(raw->buff,pos,2); pos+=2;//IOD_SSR
    satnum=(int)getbitu(raw->buff,pos,5); pos+=5; /* satellite count */
    if (mask->IOD_SSR<0||iod_ssr!=mask->IOD_SSR) return 0;

    ref_time=b2b_tod2time(raw->time,sow);
    if (!ref_time.time) return -1;
    if (!(products=get_prn6_products(ctx,prn6,1))) return -1;

    for (i=0;i<satnum;i++) {
        const int *modes=NULL;
        B2bssr_t *ssr=NULL;
        int slot,sig_num,sat,new_epoch=0,sat_updated=0;

        if (!bit_range_valid(raw,pos,13)) return -1;
        slot=(int)getbitu(raw->buff,pos,9); pos+=9; /* B2b slot */
        sig_num=(int)getbitu(raw->buff,pos,4); pos+=4; /* signal count */
        if (!bit_range_valid(raw,pos,sig_num*16)) return -1;

        sat=b2b_slot2satno(slot);
        if (sat>0&&sat<=MAXSAT&&(modes=codebias_modes(sat))) {
            ssr=products+sat; /* WARNING: index 0 stays unused. */
            new_epoch=!ssr->t0[1].time||ssr->t0[1].time!=ref_time.time||
                      ssr->t0[1].sec!=ref_time.sec||
                      ssr->iodssr[1]!=iod_ssr;
            if (new_epoch) {
                /* Validity, rather than the numeric bias, distinguishes zero
                 * correction from a product that is not present. */
                memset(ssr->cbias_valid,0,sizeof(ssr->cbias_valid));
            }
            ssr->t0[1]=ref_time;
            ssr->sow=(int)sow;
            ssr->verify_sow=verify_sod(ref_time);
            ssr->iodssr[1]=iod_ssr;
        }

        for (j=0;j<sig_num;j++) {
            int mode=(int)getbitu(raw->buff,pos,4);
            int dcb,code;

            pos+=4;
            dcb=(int)getbits(raw->buff,pos,12); pos+=12;
            if (!ssr||abs(dcb)>=2103||mode>=B2B_CODE_MODE_COUNT) continue;
            code=modes[mode];
            if (code<=CODE_NONE||code>MAXCODE) continue;
            ssr->cbias[code]=(float)(dcb*0.017);
            ssr->cbias_valid[code]=1;
            sat_updated=1;
        }
        if (ssr&&(sat_updated||new_epoch)) {
            ssr->source_prn6=(uint8_t)prn6;
            ssr->source_valid=1;
            ssr->update=0;
            publish_selected(raw,ctx,sat,prn6);
            updated++;
        }
    }
    return updated?20:0;
}

/* Decode B2b Type 4: one 23-satellite clock subtype. */
static int decode_type4(raw_t *raw, int pos, int prn6)
{
    sino_b2b_ctx_t *ctx=get_context(raw);
    B2bmask_t *mask=get_prn6_mask(ctx,prn6);
    B2bssr_t *products;
    gtime_t ref_time;
    uint32_t sow;
    int i,iod_ssr,iodp,subtype,begin,updated=0;

    if (!mask||!bit_range_valid(raw,pos,32+23*18)) return -1;

    sow=getbitu(raw->buff,pos,17); pos+=17;
    pos+=4; /* reserved */
    iod_ssr=(int)getbitu(raw->buff,pos,2); pos+=2;
    iodp=(int)getbitu(raw->buff,pos,4); pos+=4;
    subtype=(int)getbitu(raw->buff,pos,5); pos+=5;
    if (mask->IOD_SSR<0||iod_ssr!=mask->IOD_SSR||
        iodp!=mask->IODP||subtype>31) return 0;

    begin=subtype*23;
    ref_time=b2b_tod2time(raw->time,sow);
    if (!ref_time.time) return -1;
    if (!(products=get_prn6_products(ctx,prn6,1))) return -1;

    for (i=0;i<23;i++) {
        int iodcorr=(int)getbitu(raw->buff,pos,3);
        int c0,mask_index=begin+i,sat=0;
        B2bssr_t *ssr;

        pos+=3;
        c0=(int)getbits(raw->buff,pos,15); pos+=15;
        if (mask_index>=0&&mask_index<mask->satnum&&
            mask_index<B2B_MAXSAT) {
            sat=mask->satno[mask_index];
        }
        if (sat<=0||sat>MAXSAT) continue;
        if (abs(c0)>=16383||iodcorr>7) continue;

        ssr=products+sat; /* WARNING: B2b uses [sat]. */
        {
            B2bssr_t next=*ssr;

            next.t0[2]=ref_time;
            next.sow=(int)sow;
            next.verify_sow=verify_sod(ref_time);
            next.iodssr[2]=iod_ssr;
            next.iodp[0]=iodp;
            next.iodcorr[1]=(uint16_t)iodcorr;
            next.dclk[0]=c0*0.0016;
            next.source_prn6=(uint8_t)prn6;
            next.source_valid=1;
            next.update=0;
            *ssr=next; /* Commit one complete clock tuple atomically. */
        }
        publish_selected(raw,ctx,sat,prn6);
        updated++;
    }
    return updated?20:0;
}

/* Message 1697 contains PRN32, PRN6, reserved bits, type and B2b data. */
static int decode_b2b(raw_t *raw)
{
    int pos=SINO_HEADER_LEN*8;
    int prn6,type;

    if (!bit_range_valid(raw,pos,50)) return -1;
    pos+=32; /* PRN32 is transport metadata; PRN6 selects B2b context. */
    prn6=(int)getbitu(raw->buff,pos,6); pos+=6;
    pos+=6; /* reserved */
    type=(int)getbitu(raw->buff,pos,6); pos+=6;

    if (prn6==62) return 0;
    switch (type) {
        case 1: return decode_type1(raw,pos,prn6);
        case 2: return decode_type2(raw,pos,prn6);
        case 3: return decode_type3(raw,pos,prn6);
        case 4: return decode_type4(raw,pos,prn6);
    }
    return 0; /* Type 63 and other pages do not publish B2b products. */
}

/* Validate a complete Sino frame and dispatch receiver message 1697. */
static int decode_sino(raw_t *raw)
{
    int type,week;
    double tow;

    if (!raw||raw->len<SINO_HEADER_LEN) return -1;
    if (rtk_crc32(raw->buff,raw->len)!=get_u4(raw->buff+raw->len)) {
        return -1;
    }
    if (raw->buff[3]!=SINO_HEADER_LEN) return -1;

    type=get_u2(raw->buff+4);
    week=get_u2(raw->buff+14);
    if (week==0) return 0;
    tow=get_u4(raw->buff+16)*0.001;
    raw->time=gpst2time(week,tow);

    /* The reference decoder reads uninitialized msg/stat variables. This
     * implementation intentionally relies only on the documented week/TOW,
     * CRC, frame length and explicit bit-range checks. */
    if (raw->outtype) {
        sprintf(raw->msgtype,"SINO %4d (%4d)",type,raw->len);
    }
    return type==SINO_MSG_B2B?decode_b2b(raw):0;
}

/* Allocate the per-PRN6 MASK context owned exclusively by this Sino raw_t. */
extern int init_sino_b2b(raw_t *raw)
{
    sino_b2b_ctx_t *ctx;
    int prn6,sat;

    if (!raw) return 0;
    if ((ctx=get_context(raw))) return 1;
    if (raw->rcv_data) return 0;
    if (!(ctx=(sino_b2b_ctx_t *)calloc(1,sizeof(*ctx)))) return 0;

    ctx->magic=SINO_CTX_MAGIC;
    for (prn6=0;prn6<SINO_PRN6_COUNT;prn6++) {
        ctx->mask[prn6].IOD_SSR=-1;
        ctx->mask[prn6].IODP=-1;
    }
    for (sat=0;sat<=MAXSAT;sat++) ctx->selected_prn6[sat]=-1;
    ctx->latest_mask_prn6=0;
    raw->rcv_data=ctx;
    raw->nbyte=raw->len=0;
    memset(raw->buff,0,sizeof(raw->buff));
    memset(raw->nav.B2bssr,0,sizeof(raw->nav.B2bssr));
    return 1;
}

/* Free only the receiver-local Sino context selected by raw->format. */
extern void free_sino_b2b(raw_t *raw)
{
    sino_b2b_ctx_t *ctx=get_context(raw);
    int prn6;

    if (!ctx) return;
    for (prn6=0;prn6<SINO_PRN6_COUNT;prn6++) {
        free(ctx->product[prn6]);
        ctx->product[prn6]=NULL;
    }
    ctx->magic=0;
    free(ctx);
    raw->rcv_data=NULL;
    raw->nbyte=raw->len=0;
}

/* Return the most recently published Type 1 MASK for legacy diagnostics. */
extern const B2bmask_t *sino_b2b_mask(const raw_t *raw)
{
    sino_b2b_ctx_t *ctx=get_context(raw);

    return ctx?&ctx->mask[ctx->latest_mask_prn6]:NULL;
}

/* Read-only source-bank accessors used by focused receiver diagnostics. */
extern const B2bssr_t *sino_b2b_source_product(const raw_t *raw, int prn6,
                                                int sat)
{
    sino_b2b_ctx_t *ctx=get_context(raw);

    if (!ctx||prn6<0||prn6>=SINO_PRN6_COUNT||sat<=0||sat>MAXSAT||
        !ctx->product[prn6]) return NULL;
    return ctx->product[prn6]+sat;
}

extern int sino_b2b_selected_source(const raw_t *raw, int sat)
{
    sino_b2b_ctx_t *ctx=get_context(raw);

    return ctx&&sat>0&&sat<=MAXSAT?ctx->selected_prn6[sat]:-1;
}

/* Feed one byte through sync, length, complete-frame and CRC processing. */
extern int input_sino(raw_t *raw, uint8_t data)
{
    if (!raw||!get_context(raw)) return -1;

    if (raw->nbyte==0) {
        if (sync_sino(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    if (raw->nbyte>=MAXRAWLEN) {
        raw->nbyte=raw->len=0;
        return -1;
    }
    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte==10) {
        raw->len=get_u2(raw->buff+8)+SINO_HEADER_LEN;
        /* raw->len covers header+payload; four trailing CRC bytes follow. */
        if (raw->len<SINO_HEADER_LEN||raw->len>MAXRAWLEN-4) {
            raw->nbyte=raw->len=0;
            return -1;
        }
    }
    if (raw->nbyte<10||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    return decode_sino(raw);
}

/* File replay deliberately reuses the byte decoder used by realtime streams. */
extern int input_sinof(raw_t *raw, FILE *fp)
{
    int i,data,ret;

    if (!raw||!fp) return -1;
    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        ret=input_sino(raw,(uint8_t)data);
        if (ret) return ret;
    }
    return 0;
}
