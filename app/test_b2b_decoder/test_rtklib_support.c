/*
 * Minimal RTKLIB utility subset for the independent Stage 3C decoder test.
 *
 * The target rtkcmn.c currently has an unrelated readblq() compile error, so
 * this test adapter supplies only the time, CRC and satellite-number helpers
 * used by b2b.c and rcv/unicore.c. It is not part of the receiver decoder.
 */
#include "rtklib.h"

#define POLYCRC32 0xEDB88320u

static const double gpst0[]={1980,1,6,0,0,0};
static const double gst0 []={1999,8,22,0,0,0};
static const double bdt0 []={2006,1,1,0,0,0};

extern int init_rt17(raw_t *raw) {(void)raw; return 1;}
extern int init_cmr(raw_t *raw) {(void)raw; return 1;}
extern void free_rt17(raw_t *raw) {(void)raw;}
extern void free_cmr(raw_t *raw) {(void)raw;}
extern int update_cmr(raw_t *raw, rtksvr_t *svr, obs_t *obs)
{
    (void)raw; (void)svr; (void)obs;
    return 0;
}

extern void trace(int level, const char *format, ...)
{
    (void)level; (void)format;
}

extern void tracet(int level, const char *format, ...)
{
    (void)level; (void)format;
}

extern int input_oem4(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_oem3(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_ubx(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_ss2(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_cres(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_stq(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_javad(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_nvs(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_bnx(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_rt17(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}
extern int input_sbf(raw_t *raw, uint8_t data) {(void)raw; (void)data; return 0;}

extern int input_oem4f(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_oem3f(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_ubxf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_ss2f(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_cresf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_stqf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_javadf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_nvsf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_bnxf(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_rt17f(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}
extern int input_sbff(raw_t *raw, FILE *fp) {(void)raw; (void)fp; return -2;}

extern int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_CMP:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_IRN:
            if (prn<MINPRNIRN||MAXPRNIRN<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+
                   prn-MINPRNIRN+1;
        case SYS_LEO:
            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
                   prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
                   NSATLEO+prn-MINPRNSBS+1;
    }
    return 0;
}

extern int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;

    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1;
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_CMP; sat+=MINPRNCMP-1;
    }
    else if ((sat-=NSATCMP)<=NSATIRN) {
        sys=SYS_IRN; sat+=MINPRNIRN-1;
    }
    else if ((sat-=NSATIRN)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1;
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1;
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}

extern uint32_t rtk_crc32(const uint8_t *buff, int len)
{
    uint32_t crc=0;
    int i,j;

    for (i=0;i<len;i++) {
        crc^=buff[i];
        for (j=0;j<8;j++) {
            crc=crc&1?(crc>>1)^POLYCRC32:crc>>1;
        }
    }
    return crc;
}

extern uint32_t getbitu(const uint8_t *buff, int pos, int len)
{
    uint32_t bits=0;
    int i;

    for (i=pos;i<pos+len;i++) {
        bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    }
    return bits;
}

extern int32_t getbits(const uint8_t *buff, int pos, int len)
{
    uint32_t bits=getbitu(buff,pos,len);

    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int32_t)bits;
    return (int32_t)(bits|(~0u<<len));
}

extern gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

    if (year<1970||2099<year||mon<1||12<mon) return time;
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+
         (year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

extern void time2epoch(gtime_t t, double *ep)
{
    const int mday[]={
        31,28,31,30,31,30,31,31,30,31,30,31,
        31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,
        31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days=(int)(t.time/86400);
    int sec=(int)(t.time-(time_t)days*86400);
    int mon,day;

    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon];
        else break;
    }
    ep[0]=1970+days/1461*4+mon/12;
    ep[1]=mon%12+1;
    ep[2]=day+1;
    ep[3]=sec/3600;
    ep[4]=sec%3600/60;
    ep[5]=sec%60+t.sec;
}

extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

extern gtime_t gst2time(int week, double sec)
{
    gtime_t t=epoch2time(gst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

extern gtime_t bdt2time(int week, double sec)
{
    gtime_t t=epoch2time(bdt0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

extern double time2gpst(gtime_t t, int *week)
{
    double sec=timediff(t,epoch2time(gpst0));
    int w=(int)floor(sec/(86400.0*7.0));

    if (week) *week=w;
    return sec-w*86400.0*7.0;
}

extern gtime_t gpst2utc(gtime_t t)
{
    return t;
}

extern gtime_t utc2gpst(gtime_t t)
{
    return t;
}

extern int adjgpsweek(int week)
{
    return week;
}

extern gtime_t timeadd(gtime_t t, double sec)
{
    double tt;

    t.sec+=sec;
    tt=floor(t.sec);
    t.time+=(int)tt;
    t.sec-=tt;
    return t;
}

extern gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t,-14.0);
}

extern gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t,14.0);
}
