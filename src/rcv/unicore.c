/*------------------------------------------------------------------------------
* unicore.c : minimal Unicore PPP-B2b receiver decoder
*
* Stage 3B decodes complete UM980 B2bBin frames into raw->nav.B2bssr[sat].
* Stage 3C connects the decoder to RTKLIB raw input dispatch only; it still
* does not update a main nav_t or any PPP processing path.
*-----------------------------------------------------------------------------*/
#include "unicore.h"

#define UNICORE_SYNC1       0xAA
#define UNICORE_SYNC2       0x44
#define UNICORE_SYNC3       0xB5
#define UNICORE_HEADER_LEN  24

#define UNICORE_MSG_B2B_INFO1 2302
#define UNICORE_MSG_B2B_INFO2 2304
#define UNICORE_MSG_B2B_INFO3 2306
#define UNICORE_MSG_B2B_INFO4 2308

#define B2B_CODE_MODE_COUNT 15
#define UNICORE_CTX_MAGIC   0x55423242u

typedef struct {
    uint32_t magic;
    B2bmask_t mask;
} unicore_b2b_ctx_t;

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

static int16_t get_s2(const uint8_t *p)
{
    return (int16_t)get_u2(p);
}

static uint32_t get_u4(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|
           ((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}

static unicore_b2b_ctx_t *get_context(const raw_t *raw)
{
    unicore_b2b_ctx_t *ctx;

    if (!raw||!raw->rcv_data) return NULL;
    ctx=(unicore_b2b_ctx_t *)raw->rcv_data;
    return ctx->magic==UNICORE_CTX_MAGIC?ctx:NULL;
}

static int sync_unicore(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1];
    buff[1]=buff[2];
    buff[2]=data;
    return buff[0]==UNICORE_SYNC1&&buff[1]==UNICORE_SYNC2&&
           buff[2]==UNICORE_SYNC3;
}

static void mask_to_binary(const uint8_t *mask, int *binary)
{
    int i,j;

    for (i=0;i<32;i++) {
        for (j=0;j<8;j++) {
            binary[i*8+j]=(mask[i]>>(7-j))&1;
        }
    }
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
        case SYS_GAL:
            /*
             * NEEDS ICD CONFIRMATION: Stage 1 maps Galileo mode 2 to L1X,
             * while the reference project maps it to L1C. Stage 3B preserves
             * the already validated Stage 1 mapping for regression testing.
             */
            return b2b_gal_codebias_mode;
        case SYS_CMP: return b2b_bds_codebias_mode;
    }
    return NULL;
}

static int decode_PPPPB2BINFO1(raw_t *raw, const uint8_t *payload,
                               int payload_len)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);
    int binary[256]={0};
    int i,geoprn;

    if (!ctx||payload_len<40) return -1;
    geoprn=(int)get_s2(payload)-160;
    if (geoprn==62) return 0;

    memset(&ctx->mask,0,sizeof(ctx->mask));
    ctx->mask.recv_time=raw->time;
    ctx->mask.ref_time=b2b_tod2time(raw->time,get_u4(payload+4));
    ctx->mask.IOD_SSR=payload[2];
    ctx->mask.IODP=payload[3];

    mask_to_binary(payload+8,binary);
    for (i=0;i<63;i++) ctx->mask.MASK_BD[i]=binary[i];
    for (i=63;i<100;i++) ctx->mask.MASK_GPS[i-63]=binary[i];
    for (i=100;i<137;i++) ctx->mask.MASK_GAL[i-100]=binary[i];
    for (i=137;i<174;i++) ctx->mask.MASK_GLO[i-137]=binary[i];
    b2b_mask2satno(&ctx->mask);
    return 20;
}

static int decode_PPPPB2BINFO2(raw_t *raw, const uint8_t *payload,
                               int payload_len)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);
    gtime_t ref_time;
    uint32_t sow;
    int i,updated=0,geoprn;

    if (!ctx||payload_len<80) return -1;
    geoprn=(int)get_s2(payload)-160;
    if (geoprn==62) return 0;
    if ((int)payload[2]!=ctx->mask.IOD_SSR) return 0;

    sow=get_u4(payload+4);
    ref_time=b2b_tod2time(raw->time,sow);

    for (i=0;i<6;i++) {
        const uint8_t *p=payload+8+i*12;
        int sat=b2b_slot2satno(get_u2(p));
        int radial=get_s2(p+4);
        int in_track=get_s2(p+6);
        int cross=get_s2(p+8);
        B2bssr_t *ssr;

        if (sat<=0||sat>MAXSAT) continue;
        ssr=&raw->nav.B2bssr[sat];
        ssr->t0[0]=ref_time;
        ssr->sow=(int)sow;
        ssr->verify_sow=verify_sod(ref_time);
        ssr->iodssr[0]=payload[2];
        ssr->iodn=get_u2(p+2);
        ssr->iodcorr[0]=p[10];

        if (abs(radial)>=16383||abs(in_track)>=4095||abs(cross)>=4095) {
            continue;
        }
        ssr->deph[0]=radial*0.0016;
        ssr->deph[1]=in_track*0.0064;
        ssr->deph[2]=cross*0.0064;
        ssr->ura=p[11];
        ssr->update=1; /* later nav-update stage consumes and clears this */
        updated++;
    }
    return updated?20:0;
}

static int decode_PPPPB2BINFO3(raw_t *raw, const uint8_t *payload,
                               int payload_len)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);
    gtime_t ref_time;
    uint32_t sow;
    int i,j,satnum,updated=0,geoprn;

    if (!ctx||payload_len<8) return -1;
    satnum=payload[3];
    if (satnum>(payload_len-8)/64) return -1;
    geoprn=(int)get_s2(payload)-160;
    if (geoprn==62) return 0;
    if ((int)payload[2]!=ctx->mask.IOD_SSR) return 0;

    sow=get_u4(payload+4);
    ref_time=b2b_tod2time(raw->time,sow);

    for (i=0;i<satnum;i++) {
        const uint8_t *p=payload+8+i*64;
        int sat=b2b_slot2satno(get_u2(p));
        int bias_num=get_u2(p+2);
        const int *modes;
        B2bssr_t *ssr;
        int sat_updated=0;

        if (sat<=0||sat>MAXSAT) continue;
        if (!(modes=codebias_modes(sat))) continue;

        ssr=&raw->nav.B2bssr[sat];
        ssr->t0[1]=ref_time;
        ssr->sow=(int)sow;
        ssr->verify_sow=verify_sod(ref_time);
        ssr->iodssr[1]=payload[2];

        if (bias_num>B2B_CODE_MODE_COUNT) bias_num=B2B_CODE_MODE_COUNT;
        for (j=0;j<bias_num;j++) {
            const uint8_t *q=p+4+j*4;
            int mode=get_u2(q);
            int corr=get_s2(q+2);
            int code;

            if (abs(corr)>=2103||mode>=B2B_CODE_MODE_COUNT) continue;
            code=modes[mode];
            if (code<=CODE_NONE||code>MAXCODE) continue;
            ssr->cbias[code]=(float)(corr*0.017);
            sat_updated=1;
        }
        if (sat_updated) {
            ssr->update=1; /* code-bias product is ready in raw->nav.B2bssr */
            updated++;
        }
    }
    return updated?20:0;
}

static int decode_PPPPB2BINFO4(raw_t *raw, const uint8_t *payload,
                               int payload_len)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);
    gtime_t ref_time;
    uint32_t sow;
    int i,subtype,updated=0,geoprn;

    if (!ctx||payload_len<104) return -1;
    geoprn=(int)get_s2(payload)-160;
    if (geoprn==62) return 0;
    if ((int)payload[2]!=ctx->mask.IOD_SSR||
       (int)payload[3]!=ctx->mask.IODP) return 0;

    subtype=payload[8];
    if (subtype>31) return 0;
    sow=get_u4(payload+4);
    ref_time=b2b_tod2time(raw->time,sow);

    for (i=0;i<23;i++) {
        int mask_index=subtype*23+i;
        const uint8_t *p=payload+12+i*4;
        int iodcorr=get_u2(p);
        int c0=get_s2(p+2);
        int sat;
        B2bssr_t *ssr;

        if (mask_index<0||mask_index>=B2B_MAXSAT||
            mask_index>=ctx->mask.satnum) continue;
        sat=ctx->mask.satno[mask_index];
        if (sat<=0||sat>MAXSAT) continue;

        ssr=&raw->nav.B2bssr[sat];
        ssr->t0[2]=ref_time;
        ssr->sow=(int)sow;
        ssr->verify_sow=verify_sod(ref_time);
        ssr->iodssr[2]=ctx->mask.IOD_SSR;
        ssr->iodp[0]=ctx->mask.IODP;
        ssr->iodcorr[1]=(uint16_t)iodcorr;

        if (abs(c0)>=16383||iodcorr>7) continue;
        ssr->dclk[0]=c0*0.0016;
        ssr->update=1; /* clock product is ready in raw->nav.B2bssr */
        updated++;
    }
    return updated?20:0;
}

static int decode_unicore(raw_t *raw)
{
    const uint8_t *payload=raw->buff+UNICORE_HEADER_LEN;
    int type=get_u2(raw->buff+4);
    int stat=raw->buff[9];
    int week=get_u2(raw->buff+10);
    int payload_len=raw->len-UNICORE_HEADER_LEN;
    double tow;

    if (rtk_crc32(raw->buff,raw->len)!=get_u4(raw->buff+raw->len)) {
        return -1;
    }
    if (stat==201||week==0) return 0;

    tow=get_u4(raw->buff+12)*0.001;
    raw->time=gpst2time(week,tow);

    switch (type) {
        case UNICORE_MSG_B2B_INFO1:
            return decode_PPPPB2BINFO1(raw,payload,payload_len);
        case UNICORE_MSG_B2B_INFO2:
            return decode_PPPPB2BINFO2(raw,payload,payload_len);
        case UNICORE_MSG_B2B_INFO3:
            return decode_PPPPB2BINFO3(raw,payload,payload_len);
        case UNICORE_MSG_B2B_INFO4:
            return decode_PPPPB2BINFO4(raw,payload,payload_len);
    }
    return 0;
}

/*
 * Initialize Unicore PPP-B2b receiver-local state.
 *
 * raw is the RTKLIB receiver control object created by init_raw(). This
 * function allocates raw->rcv_data for the latest MASK and byte-stream state,
 * resets the Unicore frame buffer, and clears raw->nav.B2bssr[] so the decoder
 * starts from a known state. It returns 1 on success and 0 on allocation or
 * ownership errors. The next layer is input_unicore()/input_unicoref().
 */
extern int init_unicore_b2b(raw_t *raw)
{
    unicore_b2b_ctx_t *ctx;

    if (!raw) return 0;
    if ((ctx=get_context(raw))) return 1; /* duplicate init is harmless */
    if (raw->rcv_data) return 0;
    /*
     * The latest MASK and decoder receive state belong to this raw_t instance.
     * Storing them in raw->rcv_data avoids a file-scope global MASK, which
     * would mix state between multiple receiver streams.
     */
    if (!(ctx=(unicore_b2b_ctx_t *)calloc(1,sizeof(*ctx)))) return 0;

    ctx->magic=UNICORE_CTX_MAGIC;
    ctx->mask.IOD_SSR=-1;
    ctx->mask.IODP=-1;
    raw->rcv_data=ctx;
    raw->nbyte=raw->len=0;
    memset(raw->buff,0,sizeof(raw->buff));
    memset(raw->nav.B2bssr,0,sizeof(raw->nav.B2bssr));
    return 1;
}

/*
 * Release Unicore PPP-B2b receiver-local state.
 *
 * free_raw() calls this for STRFMT_UNICORE. The function frees only the
 * Unicore context owned by raw->rcv_data, leaves decoded B2b products in
 * raw->nav under the normal raw_t lifetime rules, and clears the pointer to
 * avoid reuse after free.
 */
extern void free_unicore_b2b(raw_t *raw)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);

    if (!ctx) return; /* repeated free or non-Unicore raw_t */
    ctx->magic=0;
    free(ctx);
    raw->rcv_data=NULL; /* clear receiver-local context after release */
    raw->nbyte=raw->len=0;
}

extern const B2bmask_t *unicore_b2b_mask(const raw_t *raw)
{
    unicore_b2b_ctx_t *ctx=get_context(raw);

    return ctx?&ctx->mask:NULL;
}

/*
 * Input one byte of Unicore binary data.
 *
 * This is the shared byte decoder used by both stream and file input. It syncs
 * AA 44 B5, collects the fixed 24-byte header plus payload and CRC, then calls
 * decode_unicore(). Successful PPP-B2b products are stored in
 * raw->nav.B2bssr[sat] with update=1; this stage does not copy them to a main
 * nav_t and never writes standard SSR storage.
 */
extern int input_unicore(raw_t *raw, uint8_t data)
{
    if (!raw||!get_context(raw)) return -1;

    if (raw->nbyte==0) {
        if (sync_unicore(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    if (raw->nbyte>=MAXRAWLEN) {
        raw->nbyte=raw->len=0;
        return -1;
    }
    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte==8) {
        raw->len=get_u2(raw->buff+6)+UNICORE_HEADER_LEN;
        /* raw->len excludes the trailing CRC; input waits for len+4 bytes. */
        if (raw->len<UNICORE_HEADER_LEN||raw->len>MAXRAWLEN-4) {
            raw->nbyte=raw->len=0;
            return -1;
        }
    }
    if (raw->nbyte<8||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    return decode_unicore(raw);
}

/*
 * File-input wrapper for the Unicore PPP-B2b decoder.
 *
 * input_unicoref() intentionally does not parse frames on its own. It reads a
 * bounded chunk from FILE *fp and feeds every byte into input_unicore(), so file
 * replay and future realtime stream input share the same sync, length, CRC and
 * message dispatch logic. Return values are passed through: 20 for a decoded
 * B2b product, 0 if more bytes are needed, a negative value for EOF or errors.
 */
extern int input_unicoref(raw_t *raw, FILE *fp)
{
    int i,data,ret;

    if (!raw||!fp) return -1;
    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2; /* EOF follows raw file decoders */
        ret=input_unicore(raw,(uint8_t)data);
        if (ret) return ret;
    }
    return 0; /* no complete frame in this chunk yet */
}
