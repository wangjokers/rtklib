/*------------------------------------------------------------------------------
* ppp.c : precise point positioning
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* options : -DIERS_MODEL  use IERS tide model
*           -DOUTSTAT_AMB output ambiguity parameters to solution status
*
* references :
*    [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*    [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
*        2003, November 2003
*    [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*        Space Technology Library, 2004
*    [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*        May 2009
*    [5] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*        Code Biases, URA
*    [6] MacMillan et al., Atmospheric gradients and the VLBI terrestrial and
*        celestial reference frames, Geophys. Res. Let., 1997
*    [7] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
*         Conventions (2010), 2010
*    [8] J.Kouba, A simplified yaw-attitude model for eclipsing GPS satellites,
*        GPS Solutions, 13:1-12, 2009
*    [9] F.Dilssner, GPS IIF-1 satellite antenna phase center and attitude
*        modeling, InsideGNSS, September, 2010
*    [10] F.Dilssner, The GLONASS-M satellite yaw-attitude model, Advances in
*        Space Research, 2010
*    [11] IGS MGEX (http://igs.org/mgex)
*
* version : $Revision:$ $Date:$
* history : 2010/07/20 1.0  new
*                           added api:
*                               tidedisp()
*           2010/12/11 1.1  enable exclusion of eclipsing satellite
*           2012/02/01 1.2  add gps-glonass h/w bias correction
*                           move windupcorr() to rtkcmn.c
*           2013/03/11 1.3  add otl and pole tides corrections
*                           involve iers model with -DIERS_MODEL
*                           change initial variances
*                           suppress acos domain error
*           2013/09/01 1.4  pole tide model by iers 2010
*                           add mode of ionosphere model off
*           2014/05/23 1.5  add output of trop gradient in solution status
*           2014/10/13 1.6  fix bug on P0(a[3]) computation in tide_oload()
*                           fix bug on m2 computation in tide_pole()
*           2015/03/19 1.7  fix bug on ionosphere correction for GLO and BDS
*           2015/05/10 1.8  add function to detect slip by MW-LC jump
*                           fix ppp solutin problem with large clock variance
*           2015/06/08 1.9  add precise satellite yaw-models
*                           cope with day-boundary problem of satellite clock
*           2015/07/31 1.10 fix bug on nan-solution without glonass nav-data
*                           pppoutsolsat() -> pppoutstat()
*           2015/11/13 1.11 add L5-receiver-dcb estimation
*                           merge post-residual validation by rnx2rtkp_test
*                           support support option opt->pppopt=-GAP_RESION=nnnn
*           2016/01/22 1.12 delete support for yaw-model bug
*                           add support for ura of ephemeris
*           2018/10/10 1.13 support api change of satexclude()
*           2020/11/30 1.14 use sat2freq() to get carrier frequency
*                           use E1-E5b for Galileo iono-free LC
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define MIN(x,y)    ((x)<(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

#define MAX_ITER    8               /* max number of iterations */
#define MAX_STD_FIX 0.15            /* max std-dev (3d) to fix solution */
#define MIN_NSAT_SOL 4              /* min satellite number for solution */
#define THRES_REJECT 4.0            /* reject threshold of posfit-res (sigma) */

#define THRES_MW_JUMP 10.0

#define VAR_POS     SQR(60.0)       /* init variance receiver position (m^2) */
#define VAR_VEL     SQR(10.0)       /* init variance of receiver vel ((m/s)^2) */
#define VAR_ACC     SQR(10.0)       /* init variance of receiver acc ((m/ss)^2) */
#define VAR_CLK     SQR(60.0)       /* init variance receiver clock (m^2) */
#define VAR_ZTD     SQR( 0.6)       /* init variance ztd (m^2) */
#define VAR_GRA     SQR(0.01)       /* init variance gradient (m^2) */
#define VAR_DCB     SQR(30.0)       /* init variance dcb (m^2) */
#define VAR_BIAS    SQR(60.0)       /* init variance phase-bias (m^2) */
#define VAR_IONO    SQR(60.0)       /* init variance iono-delay */
#define VAR_GLO_IFB SQR( 0.6)       /* variance of glonass ifb */

#define ERR_SAAS    0.3             /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5             /* broadcast iono model error factor */
#define ERR_CBIAS   0.3             /* code bias error std (m) */
#define REL_HUMI    0.7             /* relative humidity for saastamoinen model */
#define GAP_RESION  120             /* default gap to reset ionos parameters (ep) */

#define EFACT_GPS_L5 10.0           /* error factor of GPS/QZS L5 */

#define MUDOT_GPS   (0.00836*D2R)   /* average angular velocity GPS (rad/s) */
#define MUDOT_GLO   (0.00888*D2R)   /* average angular velocity GLO (rad/s) */
#define EPS0_GPS    (13.5*D2R)      /* max shadow crossing angle GPS (rad) */
#define EPS0_GLO    (14.2*D2R)      /* max shadow crossing angle GLO (rad) */
#define T_POSTSHADOW 1800.0         /* post-shadow recovery time (s) */
#define QZS_EC_BETA 20.0            /* max beta angle for qzss Ec (deg) */

/* number and index of states *///N开头的就是number of  什么东西
//I开头的就是index在扩展卡尔曼滤波里面x的位置
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)//使用LC组合则=1，否则=载波种类个数
#define NP(opt)     ((opt)->dynamics?9:3)//坐标的未知数个数，静态三个，动态九个
#define NC(opt)     (NSYS)//(多系统)接收机钟差的个数，=预编译器中已开启的系统个数number of clock solution
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))//电离层未知数个数
#define NI(opt)     ((opt)->ionoopt==IONOOPT_EST?MAXSAT:0)//电离层参数个数，使用LC则=0
#define ND(opt)     ((opt)->nf>=3?1:0)//接收机DCB未知数个数，非3频为0
#define NR(opt)     (NP(opt)+NC(opt)+NT(opt)+NI(opt)+ND(opt))//除去模糊度以外，所有需要估计的状态的总共的维度，前面五种的总和
#define NB(opt)     (NF(opt)*MAXSAT)//模糊度个数
#define NX(opt)     (NR(opt)+NB(opt))//待估参数个数
#define IC(s,opt)   (NP(opt)+(s))   //接收机钟差在x阵的idx: s的取值如下，0:GPS 1:GLONASS 2:GAL 3:BDS
#define IT(opt)     (NP(opt)+NC(opt))//电离层未知数在x阵的idx
#define II(s,opt)   (NP(opt)+NC(opt)+NT(opt)+(s)-1)//电离层未知数在x阵的idx
#define ID(opt)     (NP(opt)+NC(opt)+NT(opt)+NI(opt))//接收机DCB未知数在x阵的idx
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)//模糊度在x阵的idx

/* standard deviation of state -----------------------------------------------*/
static double STD(rtk_t *rtk, int i)
{
    if (rtk->sol.stat==SOLQ_FIX) return SQRT(rtk->Pa[i+i*rtk->nx]);
    return SQRT(rtk->P[i+i*rtk->nx]);
}
/* write solution status for PPP ---------------------------------------------*/
extern int pppoutstat(rtk_t *rtk, char *buff)
{
    ssat_t *ssat;
    double tow,pos[3],vel[3],acc[3],*x;
    int i,j,week;
    char id[32],*p=buff;
    
    if (!rtk->sol.stat) return 0;
    
    trace(3,"pppoutstat:\n");
    
    tow=time2gpst(rtk->sol.time,&week);
    
    x=rtk->sol.stat==SOLQ_FIX?rtk->xa:rtk->x;
    
    /* receiver position */
    p+=sprintf(p,"$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",week,tow,
               rtk->sol.stat,x[0],x[1],x[2],STD(rtk,0),STD(rtk,1),STD(rtk,2));
    
    /* receiver velocity and acceleration */
    if (rtk->opt.dynamics) {
        ecef2pos(rtk->sol.rr,pos);
        ecef2enu(pos,rtk->x+3,vel);
        ecef2enu(pos,rtk->x+6,acc);
        p+=sprintf(p,"$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,"
                   "%.4f,%.5f,%.5f,%.5f\n",week,tow,rtk->sol.stat,vel[0],vel[1],
                   vel[2],acc[0],acc[1],acc[2],0.0,0.0,0.0,0.0,0.0,0.0);
    }
    /* receiver clocks */
    i=IC(0,&rtk->opt);
    p+=sprintf(p,"$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
               week,tow,rtk->sol.stat,1,x[i]*1E9/CLIGHT,x[i+1]*1E9/CLIGHT,
               STD(rtk,i)*1E9/CLIGHT,STD(rtk,i+1)*1E9/CLIGHT);
    
    /* tropospheric parameters */
    if (rtk->opt.tropopt==TROPOPT_EST||rtk->opt.tropopt==TROPOPT_ESTG) {
        i=IT(&rtk->opt);
        p+=sprintf(p,"$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n",week,tow,rtk->sol.stat,
                   1,x[i],STD(rtk,i));
    }
    if (rtk->opt.tropopt==TROPOPT_ESTG) {
        i=IT(&rtk->opt);
        p+=sprintf(p,"$TRPG,%d,%.3f,%d,%d,%.5f,%.5f,%.5f,%.5f\n",week,tow,
                   rtk->sol.stat,1,x[i+1],x[i+2],STD(rtk,i+1),STD(rtk,i+2));
    }
    /* ionosphere parameters */
    if (rtk->opt.ionoopt==IONOOPT_EST) {
        for (i=0;i<MAXSAT;i++) {
            ssat=rtk->ssat+i;
            if (!ssat->vs) continue;
            j=II(i+1,&rtk->opt);
            if (rtk->x[j]==0.0) continue;
            satno2id(i+1,id);
            p+=sprintf(p,"$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,id,rtk->ssat[i].azel[0]*R2D,
                       rtk->ssat[i].azel[1]*R2D,x[j],STD(rtk,j));
        }
    }
#ifdef OUTSTAT_AMB
    /* ambiguity parameters */
    for (i=0;i<MAXSAT;i++) for (j=0;j<NF(&rtk->opt);j++) {
        k=IB(i+1,j,&rtk->opt);
        if (rtk->x[k]==0.0) continue;
        satno2id(i+1,id);
        p+=sprintf(p,"$AMB,%d,%.3f,%d,%s,%d,%.4f,%.4f\n",week,tow,
                   rtk->sol.stat,id,j+1,x[k],STD(rtk,k));
    }
#endif
    return (int)(p-buff);
}
/* exclude meas of eclipsing satellite (block IIA) ---------------------------*/
static void testeclipse(const obsd_t *obs, int n, const nav_t *nav, double *rs)
{
    double rsun[3],esun[3],r,ang,erpv[5]={0},cosa;
    int i,j;
    const char *type;
    
    trace(3,"testeclipse:\n");
    
    /* unit vector of sun direction (ecef) */
    sunmoonpos(gpst2utc(obs[0].time),erpv,rsun,NULL,NULL);
    normv3(rsun,esun);
    
    for (i=0;i<n;i++) {
        type=nav->pcvs[obs[i].sat-1].type;
        
        if ((r=norm(rs+i*6,3))<=0.0) continue;
        
        /* only block IIA */
        if (*type&&!strstr(type,"BLOCK IIA")) continue;
        
        /* sun-earth-satellite angle */
        cosa=dot(rs+i*6,esun,3)/r;
        cosa=cosa<-1.0?-1.0:(cosa>1.0?1.0:cosa);
        ang=acos(cosa);
        
        /* test eclipse */
        if (ang<PI/2.0||r*sin(ang)>RE_WGS84) continue;
        
        trace(3,"eclipsing sat excluded %s sat=%2d\n",time_str(obs[0].time,0),
              obs[i].sat);
        
        for (j=0;j<3;j++) rs[j+i*6]=0.0;
    }
}
/* nominal yaw-angle ---------------------------------------------------------*/
static double yaw_nominal(double beta, double mu)
{
    if (fabs(beta)<1E-12&&fabs(mu)<1E-12) return PI;
    return atan2(-tan(beta),sin(mu))+PI;
}
/* yaw-angle of satellite ----------------------------------------------------*/
extern int yaw_angle(int sat, const char *type, int opt, double beta, double mu,
                     double *yaw)
{
    *yaw=yaw_nominal(beta,mu);
    return 1;
}
/* satellite attitude model --------------------------------------------------*/
static int sat_yaw(gtime_t time, int sat, const char *type, int opt,
                   const double *rs, double *exs, double *eys)
{
    double rsun[3],ri[6],es[3],esun[3],n[3],p[3],en[3],ep[3],ex[3],E,beta,mu;
    double yaw,cosy,siny,erpv[5]={0};
    int i;
    
    sunmoonpos(gpst2utc(time),erpv,rsun,NULL,NULL);
    
    /* beta and orbit angle */
    matcpy(ri,rs,6,1);
    ri[3]-=OMGE*ri[1];
    ri[4]+=OMGE*ri[0];
    cross3(ri,ri+3,n);
    cross3(rsun,n,p);
    if (!normv3(rs,es)||!normv3(rsun,esun)||!normv3(n,en)||
        !normv3(p,ep)) return 0;
    beta=PI/2.0-acos(dot(esun,en,3));
    E=acos(dot(es,ep,3));
    mu=PI/2.0+(dot(es,esun,3)<=0?-E:E);
    if      (mu<-PI/2.0) mu+=2.0*PI;
    else if (mu>=PI/2.0) mu-=2.0*PI;
    
    /* yaw-angle of satellite */
    if (!yaw_angle(sat,type,opt,beta,mu,&yaw)) return 0;
    
    /* satellite fixed x,y-vector */
    cross3(en,es,ex);
    cosy=cos(yaw);
    siny=sin(yaw);
    for (i=0;i<3;i++) {
        exs[i]=-siny*en[i]+cosy*ex[i];
        eys[i]=-cosy*en[i]-siny*ex[i];
    }
    return 1;
}
/* phase windup model --------------------------------------------------------*/
static int model_phw(gtime_t time, int sat, const char *type, int opt,
                     const double *rs, const double *rr, double *phw)
{
    double exs[3],eys[3],ek[3],exr[3],eyr[3],eks[3],ekr[3],E[9];
    double dr[3],ds[3],drs[3],r[3],pos[3],cosp,ph;
    int i;
    
    if (opt<=0) return 1; /* no phase windup */
    
    /* satellite yaw attitude model */
    if (!sat_yaw(time,sat,type,opt,rs,exs,eys)) return 0;
    
    /* unit vector satellite to receiver */
    for (i=0;i<3;i++) r[i]=rr[i]-rs[i];
    if (!normv3(r,ek)) return 0;
    
    /* unit vectors of receiver antenna */
    ecef2pos(rr,pos);
    xyz2enu(pos,E);
    exr[0]= E[1]; exr[1]= E[4]; exr[2]= E[7]; /* x = north */
    eyr[0]=-E[0]; eyr[1]=-E[3]; eyr[2]=-E[6]; /* y = west  */
    
    /* phase windup effect */
    cross3(ek,eys,eks);
    cross3(ek,eyr,ekr);
    for (i=0;i<3;i++) {
        ds[i]=exs[i]-ek[i]*dot(ek,exs,3)-eks[i];
        dr[i]=exr[i]-ek[i]*dot(ek,exr,3)+ekr[i];
    }
    cosp=dot(ds,dr,3)/norm(ds,3)/norm(dr,3);
    if      (cosp<-1.0) cosp=-1.0;
    else if (cosp> 1.0) cosp= 1.0;
    ph=acos(cosp)/2.0/PI;
    cross3(ds,dr,drs);
    if (dot(ek,drs,3)<0.0) ph=-ph;
    
    *phw=ph+floor(*phw-ph+0.5); /* in cycle */
    return 1;
}
/* measurement error variance ------------------------------------------------*/
//static double varerr(int sat, int sys, double el, int idx, int type,
//                     const prcopt_t *opt)
//{
//    double fact=1.0,sinel=sin(el);
//    
//    if (type==1) fact*=opt->eratio[idx==0?0:1];
//    fact*=sys==SYS_GLO?EFACT_GLO:(sys==SYS_SBS?EFACT_SBS:EFACT_GPS);
//    
//    if (sys==SYS_GPS||sys==SYS_QZS) {
//        if (idx==2) fact*=EFACT_GPS_L5; /* GPS/QZS L5 error factor */
//    }
//    if (opt->ionoopt==IONOOPT_IFLC) fact*=3.0;///噪声放大三倍
//    return SQR(fact*opt->err[1])+SQR(fact*opt->err[2]/sinel);
//}



/* measurement error variance -----funtion3-------------------------------------------*/
static double varerr(int sat, int sys, double el, int idx, int type,
             const prcopt_t* opt)
{
    double sinel = sin(el);
    if (sinel < 1E-3) sinel = 1E-3;

    /* 论文 Function_3 参数：伪距 a=0.4,b=0.6；载波 a=0.004,b=0.006 */
    double a = (type == 1) ? 0.4 : 0.004; /* type 1=code, 0=phase */
    double b = (type == 1) ? 0.6 : 0.006;

    /* 系统因子沿用 RTKLIB 习惯 */
    double efact = (sys == SYS_GLO) ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);

    /* 若想维持 IF LC 加权，按原逻辑再乘 3 倍 */
    double ionofact = (opt->ionoopt == IONOOPT_IFLC) ? 3.0 : 1.0;

    trace(4, "[DBG] varerr sat=%2d sys=%d el=%.1f idx=%d type=%d a=%.3f b=%.3f sinel=%.3f\n",
        sat, sys, el * R2D, idx, type, a, b, sinel);

    return SQR(ionofact * efact * (a + b / sinel));

}


//
///* measurement error variance -- Function_4: (c + d*exp(-E/E0))^2 */
//static double varerr(int sat, int sys, double el, int idx, int type,
//             const prcopt_t* opt)
//{
//    double elev_deg = el * R2D;
//    double c = (type == 1) ? 1.3   : 0.013; /* code vs phase */
//    double d = (type == 1) ? 5.3   : 0.053;
//    double E0 = 10.0; /* degrees */
//
//    double efact = (sys == SYS_GLO) ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);
//    double ionofact = (opt->ionoopt == IONOOPT_IFLC) ? 3.0 : 1.0;
//
//    double term = c + d * exp(-elev_deg / E0);
//    trace(4, "[DBG] varerr F4 sat=%2d sys=%d el=%.1f idx=%d type=%d c=%.3f d=%.3f term=%.3f\n",
//        sat, sys, elev_deg, idx, type, c, d, term);
//
//    return SQR(ionofact * efact * term);
//}


///* measurement error variance -- Function_1: constant sigma0 */
//static double varerr(int sat, int sys, double el, int idx, int type,
//             const prcopt_t* opt)
//{
//    /* sigma0 per Table: code 0.6 m (or 0.9 for GE1), phase 0.006 m (0.009 for GE1) */
//    double sigma0 = (type == 1) ? 0.6 : 0.006; /* type 1=code, 0=phase */
//
//    /* system factor and iono-free scaling kept consistent with RTKLIB style */
//    double efact = (sys == SYS_GLO) ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);
//    double ionofact = (opt->ionoopt == IONOOPT_IFLC) ? 3.0 : 1.0;
//
//    double sig = ionofact * efact * sigma0;
//    trace(4, "[DBG] varerr F1 sat=%2d sys=%d idx=%d type=%d sigma0=%.3f sig=%.3f\n",
//        sat, sys, idx, type, sigma0, sig);
//
//    return SQR(sig);
//}


/* chi-square threshold for PPP-AREKF ---------------------------------------
* 返回 PPP-AREKF 使用的卡方门限。
* args   : int    dof      I   自由度，一般取当前历元有效观测方程个数 nv
* return : 卡方门限值
* notes  : 1) 当 dof<=100 时，直接复用 RTKLIB 已有 chisqr[] 表
*          2) 当 dof>100 时，使用 Wilson-Hilferty 近似补足门限，避免表长不足
*          3) 当前实现对应 alpha=0.001，与 RTKLIB chisqr[] 一致
*-----------------------------------------------------------------------------*/
static double chi2_thres_arekf(int dof)
{
    const double z=3.0902323061678132; /* alpha=0.001 对应的标准正态分位值 */
    double k,a;
    
    if (dof<=0) return 0.0;
    if (dof<=100) return chisqr[dof-1];
    
    k=(double)dof;
    a=1.0-2.0/(9.0*k)+z*sqrt(2.0/(9.0*k));
    return k*a*a*a;
}
/* PPP gross-error injection option -----------------------------------------*/
typedef struct {
    int enable;         /* 0:off 1:on */
    int epoch0;         /* start epoch index (0-based) */
    int epoch1;         /* end epoch index (inclusive) */
    int sat;            /* RTKLIB sat no or PRN (0: all sats) */
    char obs;           /* observation type, currently only 'P' is used */
    double mag;         /* fault magnitude in meters */
    int continuous;     /* 0: single epoch, 1: epoch0..epoch1 */
} ppp_fault_opt_t;
/* parse PPP gross-error injection options from misc-pppopt ------------------*/
static void get_ppp_faultopt(const char *pppopt, ppp_fault_opt_t *fault)
{
    const char *p;
    char mode[16]="",obs[8]="P";
    
    fault->enable=0;
    fault->epoch0=0;
    fault->epoch1=0;
    fault->sat=0;
    fault->obs='P';
    fault->mag=0.0;
    fault->continuous=0;
    
    if (!pppopt||!*pppopt) return;
    
    if ((p=strstr(pppopt,"-FAULT=")))        sscanf(p,"-FAULT=%d",&fault->enable);
    if ((p=strstr(pppopt,"-FAULT_EPOCH0="))) sscanf(p,"-FAULT_EPOCH0=%d",&fault->epoch0);
    if ((p=strstr(pppopt,"-FAULT_EPOCH1="))) sscanf(p,"-FAULT_EPOCH1=%d",&fault->epoch1);
    if ((p=strstr(pppopt,"-FAULT_SAT=")))    sscanf(p,"-FAULT_SAT=%d",&fault->sat);
    if ((p=strstr(pppopt,"-FAULT_MAG=")))    sscanf(p,"-FAULT_MAG=%lf",&fault->mag);
    if ((p=strstr(pppopt,"-FAULT_OBS="))&&sscanf(p,"-FAULT_OBS=%7s",obs)==1) {
        fault->obs=obs[0]>='a'&&obs[0]<='z'?obs[0]-32:obs[0];
    }
    if ((p=strstr(pppopt,"-FAULT_MODE="))&&sscanf(p,"-FAULT_MODE=%15s",mode)==1) {
        if (mode[0]=='1'||mode[0]=='C'||mode[0]=='c') fault->continuous=1;
    }
    if (fault->epoch1<fault->epoch0) fault->epoch1=fault->epoch0;
}
/* PPP epoch counter for fault injection -------------------------------------*/
static int ppp_fault_epoch(gtime_t time)
{
    static gtime_t time_prev={0};
    static int epoch=-1;
    double dt=timediff(time,time_prev);
    
    if (time_prev.time==0||fabs(dt)>1E-9) {
        if (time_prev.time==0||dt<0.0||fabs(dt)>86400.0) epoch=0;
        else epoch++;
        time_prev=time;
    }
    return epoch<0?0:epoch;
}
/* test whether current observation should be injected -----------------------*/
static int ppp_fault_active(const ppp_fault_opt_t *fault, int epoch, int sat)
{
    int prn=0;
    
    if (!fault->enable||fault->mag==0.0||fault->obs!='P') return 0;
    
    if (fault->continuous) {
        if (epoch<fault->epoch0||epoch>fault->epoch1) return 0;
    }
    else if (epoch!=fault->epoch0) return 0;
    
    satsys(sat,&prn);
    if (fault->sat>0&&fault->sat!=sat&&fault->sat!=prn) return 0;
    
    return 1;
}
/* adaptive robust test for PPP ---------------------------------------------
* 基于论文 AREKF 思想，对当前历元 PPP 的观测协方差 R 做一次更新前鲁棒检验。
* args   : rtk_t  *rtk     I   PPP 控制结构体
*          double *P       I   当前历元预测协方差阵（滤波前）
*          double *H       I   设计矩阵
*          double *v       I   创新残差向量
*          double *R       IO  观测噪声协方差阵，若判定异常则在原地放大
*          int    nv       I   当前历元有效观测方程个数
* return : 1: 当前历元触发了 AREKF 放缩  0: 未触发或计算失败
* notes  : 1) PPP 的 R 由 ppp_res() 构造成对角阵，因此更适合按观测对角元单独调整
*          2) 保留整体马氏距离 d 作为诊断量，用于判断当前历元整体异常程度
*          3) 实际放缩时使用单观测统计量 di=v_i^2/S_ii，仅放大对应的 R[i,i]
*          4) 这样可以避免少数坏观测连带降低整批正常观测的权重
*-----------------------------------------------------------------------------*/
static int arekf_ppp(rtk_t *rtk, const double *P,
                     const double *H, const double *v, double *R, int nv)
{
    const double k_thres=0.10; /* chi-square 门限缩放系数，便于实验阶段快速调参 */
    double *F=NULL,*S=NULL,*Sinv=NULL,*w=NULL;
    double d=0.0,thres_g=0.0,thres_l=0.0,beta_g=1.0,beta_i=1.0,beta_max=1.0;
    double sii,di;
    char tstr[32];
    int i,info,applied=0,nadj=0;
    
    /* 如需通过配置开关启用，可恢复下面这句判断 */
    /*if (!strstr(rtk->opt.pppopt,"-AREKF")||nv<=0) return 0;*/
    if (nv<=0) return 0;
    
    F=mat(rtk->nx,nv);
    S=mat(nv,nv);
    Sinv=mat(nv,nv);
    w=mat(nv,1);
    
    /* 先算 F=P*H，再构造创新协方差 S=H''*P*H+R */
    matmul("NN",rtk->nx,nv,rtk->nx,1.0,P,H,0.0,F);
    matcpy(S,R,nv,nv);
    matmul("TN",nv,nv,rtk->nx,1.0,H,F,1.0,S);
    matcpy(Sinv,S,nv,nv);
    
    /* 求 inv(S)，若失败则放弃本次鲁棒处理，保持原 PPP 流程继续 */
    info=matinv(Sinv,nv);
    if (info) {
        time2str(rtk->sol.time,tstr,2);
        trace(2,"%s AREKF: innovation covariance inversion error nv=%d info=%d\n",
              tstr,nv,info);
        free(F); free(S); free(Sinv); free(w);
        return 0;
    }
    /* 仍然计算整体创新统计量 d=v''*inv(S)*v，用于日志诊断 */
    matmul("NN",nv,1,nv,1.0,Sinv,v,0.0,w);
    for (i=0;i<nv;i++) d+=v[i]*w[i];
    
    thres_g=k_thres*chi2_thres_arekf(nv);
    thres_l=k_thres*chi2_thres_arekf(1);
    if (thres_g>0.0&&d>thres_g) {
        beta_g=MAX(1.0,d/thres_g);
    }
    time2str(rtk->sol.time,tstr,2);
    
    /* PPP 的 R 本来就是对角阵，这里按观测逐条调整对角元 R[i,i]。
     * di=v_i^2/S_ii 可看作单观测标准化残差统计量。
     * 只有当前观测异常时才放大对应的测量噪声，避免整批观测一起降权。 */
    if (thres_l>0.0) {
        for (i=0;i<nv;i++) {
            sii=S[i+i*nv];
            if (sii<=0.0) continue;
            
            di=SQR(v[i])/sii;
            if (di<=thres_l) continue;
            
            beta_i=MAX(1.0,di/thres_l);
            R[i+i*nv]*=beta_i;
            if (beta_max<beta_i) beta_max=beta_i;
            nadj++;
            applied=1;
            
            trace(4,"%s AREKF obs=%d v=%9.4f sii=%9.4e di=%8.3f beta_i=%8.3f\n",
                  tstr,i+1,v[i],sii,di,beta_i);
        }
    }
    /* 输出整体统计量和逐观测放缩摘要，便于验证是否真正触发了逐观测 AREKF */
    trace(3,"%s AREKF: nv=%d d=%.3f thres=%.3f beta_g=%.3f thres1=%.3f nadj=%d beta_max=%.3f applied=%d\n",
          tstr,nv,d,thres_g,beta_g,thres_l,nadj,beta_max,applied);
    
    free(F); free(S); free(Sinv); free(w);
    return applied;
}
/* initialize state（x阵） and covariance（p阵） -------------------------------------------*/
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;//对角线赋方差，其余元素0，P不是权阵，但是方差阵
    }
}
/* geometry-free phase measurement -------------------------------------------*/
static double gfmeas(const obsd_t *obs, const nav_t *nav)
{
    double freq1,freq2;

    freq1=sat2freq(obs->sat,obs->code[0],nav);
    freq2=sat2freq(obs->sat,obs->code[1],nav);
    if (freq1==0.0||freq2==0.0||obs->L[0]==0.0||obs->L[1]==0.0) return 0.0;
    return (obs->L[0]/freq1-obs->L[1]/freq2)*CLIGHT;
}
/* Melbourne-Wubbena linear combination --------------------------------------*/
static double mwmeas(const obsd_t *obs, const nav_t *nav)
{
    double freq1,freq2;

    freq1=sat2freq(obs->sat,obs->code[0],nav);
    freq2=sat2freq(obs->sat,obs->code[1],nav);
    
    if (freq1==0.0||freq2==0.0||obs->L[0]==0.0||obs->L[1]==0.0||
        obs->P[0]==0.0||obs->P[1]==0.0) return 0.0;
    return (obs->L[0]-obs->L[1])*CLIGHT/(freq1-freq2)-
           (freq1*obs->P[0]+freq2*obs->P[1])/(freq1+freq2);
}
/* antenna corrected measurements --------------------------------------------*/
static void corr_meas(const obsd_t *obs, const nav_t *nav, const double *azel,
                      const prcopt_t *opt, const double *dantr,
                      const double *dants, double phw, double *L, double *P,
                      double *Lc, double *Pc)
{
     double freq[NFREQ]={0},C1,C2;
    int i,sys=satsys(obs->sat,NULL);
    
    for (i=0;i<NFREQ;i++) {
        L[i]=P[i]=0.0;
        freq[i]=sat2freq(obs->sat,obs->code[i],nav);
        if (freq[i]==0.0||obs->L[i]==0.0||obs->P[i]==0.0) continue;
        if (testsnr(0,0,azel[1],obs->SNR[i]*SNR_UNIT,&opt->snrmask)) continue;
        
        /* antenna phase center and phase windup correction 天线相位中心改正和相位缠绕改正*/
        L[i]=obs->L[i]*CLIGHT/freq[i]-dants[i]-dantr[i]-phw*CLIGHT/freq[i];
        P[i]=obs->P[i]-dants[i]-dantr[i];
        
        /* P1-C1,P2-C2 dcb correction (C1->P1,C2->P2) */
        if (sys==SYS_GPS||sys==SYS_GLO) {
            if (obs->code[i]==CODE_L1C) P[i]+=nav->cbias[obs->sat-1][1];
            if (obs->code[i]==CODE_L2C) P[i]+=nav->cbias[obs->sat-1][2];
        }
    }


 /*  强制让第三个频点的值到第二个频点，解决第二个频点载波和伪距为0的问题。 if(freq[1]==0.0)
    {
        freq[1] = freq[2];
    }
    if(L[1]==0.0)
    {
        L[1] = L[2];
    }
    if(P[1]==0.0)
    {
        P[1] = P[2];
    }*/

    /* iono-free LC 伽利略/SBAS/北斗系统选用L1/L5组合观测，其它选用L1/L2组合观测 ???!!*/
    *Lc=*Pc=0.0;
    if (freq[0]==0.0||freq[1]==0.0) return;
    C1= SQR(freq[0])/(SQR(freq[0])-SQR(freq[1]));//f1^2/(f1^2-f2^2)
    C2=-SQR(freq[1])/(SQR(freq[0])-SQR(freq[1]));
    
    //添加的调试日志
    //trace(2, "DBG sat=%2d c0=%d c1=%d f0=%8.3f f1=%8.3f L0=%9.3f L1=%9.3f\n",
    //    obs->sat, obs->code[0], obs->code[1],
    //    freq[0], freq[1], L[0], L[1]);


    if (L[0]!=0.0&&L[1]!=0.0) *Lc=C1*L[0]+C2*L[1];
    if (P[0]!=0.0&&P[1]!=0.0) *Pc=C1*P[0]+C2*P[1];
}
/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int n)
{
    int i,j;
    
    trace(3,"detslp_ll: n=%d\n",n);
    
    for (i=0;i<n&&i<MAXOBS;i++) for (j=0;j<rtk->opt.nf;j++) {
        if (obs[i].L[j]==0.0||!(obs[i].LLI[j]&3)) continue;
        
        trace(3,"detslp_ll: slip detected sat=%2d f=%d\n",obs[i].sat,j+1);
        
        rtk->ssat[obs[i].sat-1].slip[j]=1;
    }
}
/* detect cycle slip by geometry free phase jump -----------------------------*/
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double g0,g1;
    int i,j;
    
    trace(3,"detslp_gf: n=%d\n",n);
    
    for (i=0;i<n&&i<MAXOBS;i++) {
        
        if ((g1=gfmeas(obs+i,nav))==0.0) continue;//本历元的GF值
        
        g0=rtk->ssat[obs[i].sat-1].gf[0];//g0：上一个历元的gf值，初始值为0
        rtk->ssat[obs[i].sat-1].gf[0]=g1;//此历元的gf值作为下一个历元的g0
        
        trace(4,"detslip_gf: sat=%2d gf0=%8.3f gf1=%8.3f\n",obs[i].sat,g0,g1);
        
        if (g0!=0.0&&fabs(g1-g0)>rtk->opt.thresslip) {
            trace(3,"detslip_gf: slip detected sat=%2d gf=%8.3f->%8.3f\n",
                  obs[i].sat,g0,g1);
            
            for (j=0;j<rtk->opt.nf;j++) rtk->ssat[obs[i].sat-1].slip[j]|=1;//0无周跳，1有周跳
        }
    }
}
/* detect slip by Melbourne-Wubbena linear combination jump ------------------*/
static void detslp_mw(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double w0,w1;
    int i,j;
    
    trace(3,"detslp_mw: n=%d\n",n);
    
    for (i=0;i<n&&i<MAXOBS;i++) {
        if ((w1=mwmeas(obs+i,nav))==0.0) continue;//本历元的mw值，（宽巷模糊度+宽巷波长）
        
        w0=rtk->ssat[obs[i].sat-1].mw[0];
        rtk->ssat[obs[i].sat-1].mw[0]=w1;//套路同gf
        
        trace(4,"detslip_mw: sat=%2d mw0=%8.3f mw1=%8.3f\n",obs[i].sat,w0,w1);
        
        if (w0!=0.0&&fabs(w1-w0)>THRES_MW_JUMP) {//阈值有问题
            trace(3,"detslip_mw: slip detected sat=%2d mw=%8.3f->%8.3f\n",
                  obs[i].sat,w0,w1);
            
            for (j=0;j<rtk->opt.nf;j++) rtk->ssat[obs[i].sat-1].slip[j]|=1;//0无周跳、1有周跳
        }
    }
}
/* temporal update of position -------------
 args 
 rtk_t* rtk      IO  rtk 控制结构体
 return
 none
----------------------------------*/
static void udpos_ppp(rtk_t *rtk)
{
    double *F,*P,*FP,*x,*xp,pos[3],Q[9]={0},Qv[9];
    int i,j,*ix,nx;
    
    trace(3,"udpos_ppp:\n");
    
    /* fixed mode 非固定模式跳过*/
    if (rtk->opt.mode==PMODE_PPP_FIXED) {
        for (i=0;i<3;i++) initx(rtk,rtk->opt.ru[i],1E-8,i);
        return;
    }
    /* initialize position for first epoch */
    if (norm(rtk->x,3)<=0.0) {//仅PPP首历元执行这步，从次历元开始不再采用当前的历元SPP的解赋值X阵
        for (i=0;i<3;i++)   //初始化xyz
            initx(rtk,rtk->sol.rr[i],VAR_POS,i);//用spp解得的xyz+预设的spp定位方差3600（差别不大）赋值给x阵和p阵，
        if (rtk->opt.dynamics) //非动态跳过这里
        {
            for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
            for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
        }
    }
    /* static ppp mode   rtk->prn[5]为0，实际上并没有给xyz添加过程噪声，因为这里是静态接收机位置不变本代码无效*/
    if (rtk->opt.mode==PMODE_PPP_STATIC) {//次历元来到这里，不动x阵，只给p阵加过程噪声
        for (i=0;i<3;i++) {
            rtk->P[i*(1+rtk->nx)]+=SQR(rtk->opt.prn[5])*fabs(rtk->tt);//EKF之预测协方差阵 P=FPF”+Q 因F阵为单位阵，故此处只需+过程噪声Q阵.
        }
        return;
    }
    /* kinmatic mode without dynamics */
    if (!rtk->opt.dynamics) {
        for (i=0;i<3;i++) {
            initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        }
        return;
    }
    // 动力学模式动态 PPP，构建状态转移矩阵 F
    /* generate valid state index */
    ix=imat(rtk->nx,1);
    for (i=nx=0;i<rtk->nx;i++) {
        if (rtk->x[i]!=0.0&&rtk->P[i+i*rtk->nx]>0.0) ix[nx++]=i;
    }
    if (nx<9) {
        free(ix);
        return;
    }
    // 状态转移矩阵构建
    /* state transition of position/velocity/acceleration */
    F=eye(nx); P=mat(nx,nx); FP=mat(nx,nx); x=mat(nx,1); xp=mat(nx,1);
    
    for (i=0;i<6;i++) {
        F[i+(i+3)*nx]=rtk->tt;
    }
    for (i=0;i<3;i++) {
        F[i+(i+6)*nx]=SQR(rtk->tt)/2.0;
    }
    for (i=0;i<nx;i++) {
        x[i]=rtk->x[ix[i]];
        for (j=0;j<nx;j++) {
            P[i+j*nx]=rtk->P[ix[i]+ix[j]*rtk->nx];
        }
    }
    /* x=F*x, P=F*P*F+Q */
    // 状态转移
    matmul("NN",nx,1,nx,1.0,F,x,0.0,xp);
    matmul("NN",nx,nx,nx,1.0,F,P,0.0,FP);
    matmul("NT",nx,nx,nx,1.0,FP,F,0.0,P);
    
    for (i=0;i<nx;i++) {
        rtk->x[ix[i]]=xp[i];
        for (j=0;j<nx;j++) {
            rtk->P[ix[i]+ix[j]*rtk->nx]=P[i+j*nx];
        }
    }
    // 为 Q 矩阵加速度部分加过程噪声
    /* process noise added to only acceleration */
    Q[0]=Q[4]=SQR(rtk->opt.prn[3])*fabs(rtk->tt);
    Q[8]=SQR(rtk->opt.prn[4])*fabs(rtk->tt);
    ecef2pos(rtk->x,pos);
    covecef(pos,Q,Qv);
    for (i=0;i<3;i++) for (j=0;j<3;j++) {
        rtk->P[i+6+(j+6)*rtk->nx]+=Qv[i+j*3];
    }
    free(ix); free(F); free(P); free(FP); free(x); free(xp);
}
/* temporal update of clock --------------------------------------------------*/
static void udclk_ppp(rtk_t *rtk)
{
    double dtr;
    int i;
    
    trace(3,"udclk_ppp:\n");
    
    ///* initialize every epoch for clock (white noise) */
    //for (i=0;i<NSYS;i++) {
    //    if (rtk->opt.sateph==EPHOPT_PREC) {
    //        /* time of prec ephemeris is based gpst 检查是否为精密星历，精密星历的时间是基于gps时间的，只给gps槽的ic 0取值。*/
    //        /* negelect receiver inter-system bias  */
    //        dtr=rtk->sol.dtr[3];//PPP的EKF  dtr初值，直接采用本历元spp的结果，注意是所有的历元而非仅仅首个历元！
    //    }
    //    else {  //不是精密星历采用前一秒的数据，并考虑系统间时差信息。
    //        dtr=i==0?rtk->sol.dtr[0]:rtk->sol.dtr[0]+rtk->sol.dtr[i];
    //    }
    //    initx(rtk,CLIGHT*dtr,VAR_CLK,IC(i,&rtk->opt));//赋值x阵和P阵中钟差对应的位置，因为接收机钟差相互独立，所以用spp的值，此时前两个方程失效
    //}


    /*使用北斗加上gps的针对钟差的修改方式*/
    for (i = 0; i < NSYS; i++) {
        /* 统一策略：GPS 用自身，其他系统=GPS + 该系统的 SPP 相对偏差 */
        if (i == 0) {
            dtr = rtk->sol.dtr[0];                  /* GPS */
        }
        else {
            dtr = rtk->sol.dtr[0] + rtk->sol.dtr[i];/* BDS 等 */
        }
        initx(rtk, CLIGHT * dtr, VAR_CLK, IC(i, &rtk->opt));
    }
}
/* temporal update of tropospheric parameters --------------------------------*/
static void udtrop_ppp(rtk_t *rtk)
{
    double pos[3],azel[]={0.0,PI/2.0},ztd,var;// pos: 接收机位置（ECEF坐标），azel: 方位角和高度角（zenith方向），ztd: 对流层湿延迟，var: 湿延迟方差
    int i=IT(&rtk->opt),j;          // i: 对流层状态向量的起始索引（由选项确定），j: 循环变量
    
    trace(3,"udtrop_ppp:\n");
    
    if (rtk->x[i]==0.0) {//首次迭代，对流层未知数初值为0
        ecef2pos(rtk->sol.rr,pos);// 将ECEF坐标（rtk->sol.rr）转换为地理坐标（经纬度高度）存储到pos
        ztd=sbstropcorr(rtk->sol.time,pos,azel,&var);// 根据当前时间和位置计算初始ZTD及方差，azel为zenith方向
        initx(rtk,ztd,var,i);//赋值x阵和P阵中对流层对应的位置 ，trop
        
        if (rtk->opt.tropopt>=TROPOPT_ESTG) {// 如果选项tropopt>=TROPOPT_ESTG，估计对流层梯度
            for (j=i+1;j<i+3;j++)           // 循环初始化两个梯度参数（通常为南北和东西向）
                initx(rtk,1E-6,VAR_GRA,j);// 初始化状态为1E-6（小值），方差为VAR_GRA（预定义常数）
        }
    }
    else {//非首次迭代,如果已初始化，进行时间更新
        rtk->P[i+i*rtk->nx]+=SQR(rtk->opt.prn[2])*fabs(rtk->tt);//噪声输入阵*噪声驱动矩阵（时间）=过程噪声 非矩阵运算，但是逐元素赋值,
        // 更新ZTD协方差，增加时间相关的方差项，prn[2]为噪声标准差，tt为时间间隔
        if (rtk->opt.tropopt>=TROPOPT_ESTG) {// 如果估计梯度参数
            for (j=i+1;j<i+3;j++) {          // 循环更新两个梯度参数的协方差
                rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2]*0.1)*fabs(rtk->tt);     // 梯度噪声较ZTD小（乘0.1），反映变化较缓
            }
        }
    }
}
/* temporal update of ionospheric parameters ---------------------------------*/
static void udiono_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double freq1,freq2,ion,sinel,pos[3],*azel;
    char *p;
    int i,j,gap_resion=GAP_RESION;
    
    trace(3,"udiono_ppp:\n");
    
    if ((p=strstr(rtk->opt.pppopt,"-GAP_RESION="))) {
        sscanf(p,"-GAP_RESION=%d",&gap_resion);
    }
    for (i=0;i<MAXSAT;i++) {
        j=II(i+1,&rtk->opt);
        if (rtk->x[j]!=0.0&&(int)rtk->ssat[i].outc[0]>gap_resion) {
            rtk->x[j]=0.0;
        }
    }
    for (i=0;i<n;i++) {
        j=II(obs[i].sat,&rtk->opt);
        if (rtk->x[j]==0.0) {
            freq1=sat2freq(obs[i].sat,obs[i].code[0],nav);
            freq2=sat2freq(obs[i].sat,obs[i].code[1],nav);
            if (obs[i].P[0]==0.0||obs[i].P[1]==0.0||freq1==0.0||freq2==0.0) {
                continue;
            }
            ion=(obs[i].P[0]-obs[i].P[1])/(SQR(FREQ1/freq1)-SQR(FREQ1/freq2));
            ecef2pos(rtk->sol.rr,pos);
            azel=rtk->ssat[obs[i].sat-1].azel;
            ion/=ionmapf(pos,azel);
            initx(rtk,ion,VAR_IONO,j);
        }
        else {
            sinel=sin(MAX(rtk->ssat[obs[i].sat-1].azel[1],5.0*D2R));
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[1]/sinel)*fabs(rtk->tt);
        }
    }
}
/* temporal update of L5-receiver-dcb parameters -----------------------------*/
static void uddcb_ppp(rtk_t *rtk)
{
    int i=ID(&rtk->opt);
    
    trace(3,"uddcb_ppp:\n");
    
    if (rtk->x[i]==0.0) {
        initx(rtk,1E-6,VAR_DCB,i);
    }
}
/* temporal update of phase biases -------------------------------------------*/
static void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double L[NFREQ],P[NFREQ],Lc,Pc,bias[MAXOBS],offset=0.0,pos[3]={0};
    double freq1,freq2,ion,dantr[NFREQ]={0},dants[NFREQ]={0};
    int i,j,k,f,sat,slip[MAXOBS]={0},clk_jump=0;
    
    trace(3,"udbias  : n=%d\n",n);
    
    /* handle day-boundary clock jump 日边界钟跳检测 */
    if (rtk->opt.posopt[5]) {
        clk_jump=ROUND(time2gpst(obs[0].time,NULL)*10)%864000==0;
    }
    for (i=0;i<MAXSAT;i++)
        for (j=0;j<rtk->opt.nf;j++) {
        rtk->ssat[i].slip[j]=0;//先默认所有历元没有周跳，1表示有周跳
    }
    /* detect cycle slip by LLI */
    detslp_ll(rtk,obs,n);
    
    /* detect cycle slip by geometry-free phase jump   GF组合*/
    detslp_gf(rtk,obs,n,nav);
    
    /* detect slip by Melbourne-Wubbena linear combination jump  MW组合 */
    detslp_mw(rtk,obs,n,nav);
    
    ecef2pos(rtk->sol.rr,pos);
    
    for (f=0;f<NF(&rtk->opt);f++) {//对于IF组合，f恒为0
        
        /* reset phase-bias if expire obs outage counter */
        for (i=0;i<MAXSAT;i++) {
            if (++rtk->ssat[i].outc[f]>(uint32_t)rtk->opt.maxout||
                rtk->opt.modear==ARMODE_INST||clk_jump) {
                initx(rtk,0.0,0.0,IB(i+1,f,&rtk->opt));//模糊度置0，首个历元会执行此步
            }
        }
        for (i=k=0;i<n&&i<MAXOBS;i++) {
            sat=obs[i].sat;
            j=IB(sat,f,&rtk->opt);//模糊度在x阵中的索引
            //计算pc和Lc（此处不进行天线和相位缠绕改正，dante和dants都为0）
            corr_meas(obs+i,nav,rtk->ssat[sat-1].azel,&rtk->opt,dantr,dants,
                      0.0,L,P,&Lc,&Pc);
            
            bias[i]=0.0;//bias:存储当前历元所有卫星(96)的模糊度，区分与681行的不同:bias[i]是临时变量，用于更新rtk->x
            
            if (rtk->opt.ionoopt==IONOOPT_IFLC) {
                bias[i]=Lc-Pc;//当前历元的宽巷模糊度*宽巷波长！整周模糊度有正和负，o文件相位观测值不包含整周模糊度，仅bias本身带有负号
                slip[i]=rtk->ssat[sat-1].slip[0]||rtk->ssat[sat-1].slip[1];//双频观测，只有一频有周跳就废了
            }
            else if (L[f]!=0.0&&P[f]!=0.0) {
                freq1=sat2freq(sat,obs[i].code[0],nav);
                freq2=sat2freq(sat,obs[i].code[f],nav);
                slip[i]=rtk->ssat[sat-1].slip[f];
                if (obs[i].P[0]==0.0||obs[i].P[1]==0.0||freq1==0.0||freq2==0.0) {
                    continue;
                }
                ion=(obs[i].P[0]-obs[i].P[f])/(1.0-SQR(freq1/freq2));
                bias[i]=L[f]-P[f]+2.0*ion*SQR(freq1/freq2);
            }
            if (rtk->x[j]==0.0||slip[i]||bias[i]==0.0) continue;
            //首历元不会执行此步，因为rtk—>x[模糊度]均=0，在上一步就continue了
            offset+=bias[i]-rtk->x[j];//offset：当前历元所有卫星计算所得模糊度与所继承模糊度之差的总和，即所有卫星前后历元bias之差的总和
            k++;//本历元卫星个数
        }
        /* correct phase-code jump to ensure phase-code coherency    mine:防伪距和相位观测值有大跳变，确保前后历元Lc-Pc相差不大*/ 
        if (k>=2&&fabs(offset/k)>0.0005*CLIGHT) {
            for (i=0;i<MAXSAT;i++) {
                j=IB(i+1,f,&rtk->opt);
                if (rtk->x[j]!=0.0) rtk->x[j]+=offset/k;//微调修正继承的（前个历元）的模糊度，+=总偏差取平均值
            }
            trace(2,"phase-code jump corrected: %s n=%2d dt=%12.9fs\n",
                  time_str(rtk->sol.time,0),k,offset/k/CLIGHT);
        }
        for (i=0;i<n&&i<MAXOBS;i++) {//n表示当前历元观测到的卫星数
            sat=obs[i].sat;
            j=IB(sat,f,&rtk->opt);
            
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[0])*fabs(rtk->tt);//此处的prn[0]就是conf配置文件中设置的越小越好的stats-prnbias,rtk->x继承 了上一个历元s，rtk->P为上一个历元s+过程噪声
            
            if (bias[i]==0.0||(rtk->x[j]!=0.0&&!slip[i])) continue;//不continue的条件：rtk->x[j]=0.0(首历元)||slip[i]=1(有周跳)；定位理想情况：模糊度不为0且无周跳，则继续
            
            /* reinitialize phase-bias if detecting cycle slip */
            initx(rtk,bias[i],VAR_BIAS,IB(sat,f,&rtk->opt));//用当前计算的bias更新rtk->x，用VAR_BIAS更新rtk->P
            
            /* reset fix flags */
            for (k=0;k<MAXSAT;k++) rtk->ambc[sat-1].flags[k]=0;
            
            trace(5,"udbias_ppp: sat=%2d bias=%.3f\n",sat,bias[i]);
        }
    }
}
/* temporal update of states --------
args 
rtk_t* rtk      IO  rtk 控制结构体
obsd_t* obs     I   obs 观测数据
int      n      I   obs 观测数据的数量
nav_t* nav      I   导航数据
 return
none
------------------------------------------*/
static void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    trace(3,"udstate_ppp: n=%d\n",n);
    
    /* temporal update of position */// 调用 udpos_ppp 根据不同模式初始化状态 rtk->x 中的位置值
    udpos_ppp(rtk);//位置初始化
    
    /* temporal update of clock */// 调用 udclk_ppp 初始化状态 rtk->x 中的钟差值（6个，因有6个系统）
    udclk_ppp(rtk);//dtr初始化 (以pst为基准)EKF初值采用当前历元的spp dtr结果
    
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt==TROPOPT_EST||rtk->opt.tropopt==TROPOPT_ESTG) {
        udtrop_ppp(rtk);//对流层初始化
    }
    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt==IONOOPT_EST) {
        udiono_ppp(rtk,obs,n,nav);//电离层初始化（非lc组合时）
    }
    /* temporal update of L5-receiver-dcb parameters   3频加上观测才会用到*/
    if (rtk->opt.nf>=3) {
        uddcb_ppp(rtk);
    }
    /* temporal update of phase-bias */// 调用 udbias_ppp 更新载波相位偏移状态值以及其误差协方差。
    udbias_ppp(rtk,obs,n,nav);//phase-bias   模糊度初始化
}
/* satellite antenna phase center variation -----------
* 计算卫星天线相位中心变化修正值
* args   : const double *rs    I   卫星位置 [x,y,z] (ECEF坐标系，米)
*          const double *rr    I   接收机位置 [x,y,z] (ECEF坐标系，米)
*          const pcv_t *pcv    I   卫星天线参数结构体（包含PCV数据）
*          double *dant        O   输出的PCV修正值 [dx,dy,dz] (米)
* return : none
* notes  : 基于天底角计算卫星天线相位中心变化修正
*          天底角=0°：卫星正对地心
*          天底角=90°：卫星在地平线上
*          使用插值方法计算PCV修正值
-----------------------*/
static void satantpcv(const double *rs, const double *rr, const pcv_t *pcv,
                      double *dant)
{
    double ru[3],rz[3],eu[3],ez[3],nadir,cosa;
    int i;
    
    for (i=0;i<3;i++) {
        ru[i]=rr[i]-rs[i];
        rz[i]=-rs[i];
    }
    if (!normv3(ru,eu)||!normv3(rz,ez)) return;
    
    cosa=dot(eu,ez,3);
    cosa=cosa<-1.0?-1.0:(cosa>1.0?1.0:cosa);
    nadir=acos(cosa);
    
    antmodel_s(pcv,nadir,dant);
    // 四级日志：验证卫星PCV修正计算
    tracet(4,"satantpcv: type=%s nadir=%.1f dant=[%.6f %.6f %.6f]\n",
        pcv->type,nadir*R2D,dant[0],dant[1],dant[2]);
}
/* precise tropospheric model ------------------------------------------------*/
static double trop_model_prec(gtime_t time, const double *pos,
                              const double *azel, const double *x, double *dtdx,
                              double *var)
{
    const double zazel[]={0.0,PI/2.0};
    double zhd,m_h,m_w,cotz,grad_n,grad_e;
    
    /* zenith hydrostatic delay */
    zhd=tropmodel(time,pos,zazel,0.0);
    
    /* mapping function */
    m_h=tropmapf(time,pos,azel,&m_w);
    
    if (azel[1]>0.0) {
        
        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz=1.0/tan(azel[1]);
        grad_n=m_w*cotz*cos(azel[0]);
        grad_e=m_w*cotz*sin(azel[0]);
        m_w+=grad_n*x[1]+grad_e*x[2];
        dtdx[1]=grad_n*(x[0]-zhd);
        dtdx[2]=grad_e*(x[0]-zhd);
    }
    dtdx[0]=m_w;
    *var=SQR(0.01);
    return m_h*zhd+m_w*(x[0]-zhd);
}
/* tropospheric model ---------------------------------------------------------*/
static int model_trop(gtime_t time, const double *pos, const double *azel,
                      const prcopt_t *opt, const double *x, double *dtdx,
                      const nav_t *nav, double *dtrp, double *var)
{
    double trp[3]={0};
    
    if (opt->tropopt==TROPOPT_SAAS) {
        *dtrp=tropmodel(time,pos,azel,REL_HUMI);
        *var=SQR(ERR_SAAS);
        return 1;
    }
    if (opt->tropopt==TROPOPT_SBAS) {
        *dtrp=sbstropcorr(time,pos,azel,var);
        return 1;
    }
    if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {
        matcpy(trp,x+IT(opt),opt->tropopt==TROPOPT_EST?1:3,1);
        *dtrp=trop_model_prec(time,pos,azel,trp,dtdx,var);
        return 1;
    }
    return 0;
}
/* ionospheric model ---------------------------------------------------------*/
static int model_iono(gtime_t time, const double *pos, const double *azel,
                      const prcopt_t *opt, int sat, const double *x,
                      const nav_t *nav, double *dion, double *var)
{
    if (opt->ionoopt==IONOOPT_SBAS) {
        return sbsioncorr(time,nav,pos,azel,dion,var);
    }
    if (opt->ionoopt==IONOOPT_TEC) {
        return iontec(time,nav,pos,azel,1,dion,var);
    }
    if (opt->ionoopt==IONOOPT_BRDC) {
        *dion=ionmodel(time,nav->ion_gps,pos,azel);
        *var=SQR(*dion*ERR_BRDCI);
        return 1;
    }
    if (opt->ionoopt==IONOOPT_EST) {
        *dion=x[II(sat,opt)];
        *var=0.0;
        return 1;
    }
    if (opt->ionoopt==IONOOPT_IFLC) {
        *dion=*var=0.0;
        return 1;
    }
    return 0;
}
/* phase and code residuals --------------------------------------------------*/
static int ppp_res(int post, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *var_rs, const int *svh,
                   const double *dr, int *exc, const nav_t *nav,
                   const double *x, rtk_t *rtk, double *v, double *H, double *R,
                   double *azel)
{
    prcopt_t *opt=&rtk->opt;
    ppp_fault_opt_t fault={0};
    double y,r,cdtr,bias,C=0.0,rr[3],pos[3],e[3],dtdx[3],L[NFREQ],P[NFREQ],Lc,Pc;
    double var[MAXOBS*2],dtrp=0.0,dion=0.0,vart=0.0,vari=0.0,dcb,freq;
    double dantr[NFREQ]={0},dants[NFREQ]={0};
    double ve[MAXOBS*2*NFREQ]={0},vmax=0;
    char str[32];
    int ne=0,obsi[MAXOBS*2*NFREQ]={0},frqi[MAXOBS*2*NFREQ],maxobs,maxfrq,rej;
    int epoch=0,prn=0;
    int i,j,k,sat,sys,nv=0,nx=rtk->nx,stat=1;
    
    time2str(obs[0].time,str,2);
    get_ppp_faultopt(opt->pppopt,&fault);
    epoch=ppp_fault_epoch(obs[0].time);
    
    for (i=0;i<MAXSAT;i++) for (j=0;j<opt->nf;j++) rtk->ssat[i].vsat[j]=0;//卫星有效标志置0
    
    for (i=0;i<3;i++) rr[i]=x[i]+dr[i];//坐标值+潮汐改正dr
    ecef2pos(rr,pos);
    //trace(2, " [DEBUG]→ sat=%2d sys=%d  k=%d  IC=%d  cdtr=%.9f\n",
    //    sat, sys, k, IC(k, opt), x[IC(k, opt)]);


    for (i=0;i<n&&i<MAXOBS;i++) {
        sat=obs[i].sat;
        //根据测站位置、卫星仰角、卫星系统是否正常、卫星有效标志、是否在剔除名单中，重置排除标志exc
        if ((r=geodist(rs+i*6,rr,e))<=0.0||//r是几何距离
            satazel(pos,e,azel+i*2)<opt->elmin) {
            exc[i]=1;
            /*trace(2, "REJ A low_el sat=%d el=%.1f\n", sat, azel[i * 2 + 1] * R2D);*/
            continue;
        }
        if (!(sys=satsys(sat,NULL))||!rtk->ssat[sat-1].vs||//排除不健康卫星
            satexclude(obs[i].sat,var_rs[i],svh[i],opt)||exc[i]) {
            exc[i]=1;
            /*trace(2, "REJ B svh/exs sat=%d svh=%d ex=%d\n", sat, svh[i], exc[i]);*/
            continue;
        }

        /* 对流层模型计算延迟dtrp;电离层模型计算延迟dion*/
        /* tropospheric and ionospheric model */
        if (!model_trop(obs[i].time,pos,azel+i*2,opt,x,dtdx,nav,&dtrp,&vart)||
            !model_iono(obs[i].time,pos,azel+i*2,opt,sat,x,nav,&dion,&vari)) {
            //trace(2, "REJ C trop/iono fail sat=%d\n", sat);
            continue;
        }
        

        /* satellite and receiver antenna model
        天线模型计算卫星天线校正值dants + 接收机天线校正值dantr */
        if (opt->posopt[0]) satantpcv(rs+i*6,rr,nav->pcvs+sat-1,dants);
        antmodel(opt->pcvr,opt->antdel[0],azel+i*2,opt->posopt[1],dantr);
        // 一级日志：验证天线修正是否应用到PPP解算（只输出到trace文件，每个历元第一个卫星）
        if (i==0) tracet(4,"ppp_res: sat=%d posopt[0]=%d posopt[1]=%d dantr[0]=%.6f dantr[2]=%.6f\n",
            sat, opt->posopt[0], opt->posopt[1], dantr[0], dantr[2]);
        
        /* phase windup model 相位缠绕模型计算校正值phw */
        if (!model_phw(rtk->sol.time,sat,nav->pcvs[sat-1].type,
                       opt->posopt[2]?2:0,rs+i*6,rr,&rtk->ssat[sat-1].phw)) {
           /* trace(2, "REJ D phw fail sat=%d\n", sat);*/
            continue;
        }


        //消除电离层组合
        /* corrected phase and code measurements */
        /* 利用dants dants phw校正伪距和相位观测值，并计算Lc、Pc */
        corr_meas(obs+i,nav,azel+i*2,&rtk->opt,dantr,dants,
                  rtk->ssat[sat-1].phw,L,P,&Lc,&Pc);
        
        /* stack phase and code residuals {L1,P1,L2,P2,...} */
        for (j=0;j<2*NF(opt);j++) {
            
            dcb=bias=0.0;
            
            if (opt->ionoopt==IONOOPT_IFLC) {
                if ((y=j%2==0?Lc:Pc)==0.0) 
             /*      trace(2, "REJ E Lc/Pc=0 sat=%d\n", sat);*/
                    continue;
                //y先后取了相位Lc和伪距Pc
            }
            else {
                if ((y=j%2==0?L[j/2]:P[j/2])==0.0) continue;
                
                if ((freq=sat2freq(sat,obs[i].code[j/2],nav))==0.0) continue;
                C=SQR(FREQ1/freq)*ionmapf(pos,azel+i*2)*(j%2==0?-1.0:1.0);
            }
            /* Inject a configurable gross error into pseudorange only.
             * The offset is added after PPP measurement correction and before
             * residual formation, so the original PPP observation model stays intact. */
            if (j%2==1&&ppp_fault_active(&fault,epoch,sat)) {
                double y0=y;
                y+=fault.mag;
                if (!post) {
                    satsys(sat,&prn);
                    trace(2,"%s FAULT_INJ: epoch=%d sat=%2d prn=%2d type=%s%d mode=%s mag=%8.3f before=%12.4f after=%12.4f\n",
                          str,epoch,sat,prn,opt->ionoopt==IONOOPT_IFLC?"PC":"P",
                          j/2+1,fault.continuous?"CONT":"ONCE",fault.mag,y0,y);
                }
            }
            for (k=0;k<nx;k++) H[k+nx*nv]=k<3?-e[k]:0.0;//1.H阵每行的前三个值是卫地距方向向量，1-3列是位置改正项xyz
            
            /* receiver clock */
            switch (sys) {
                case SYS_GLO: k=1; break;
                case SYS_GAL: k=2; break;
                case SYS_CMP: k=3; break;
                case SYS_IRN: k=4; break;
                default:      k=0; break;//gps
            }
/**/
            {
                int idx_clk = IC(k, opt);
                double cdtr_val = (idx_clk >= 0 && idx_clk < rtk->nx)
                    ? x[idx_clk] : 0.0;
                trace(2, "DBG_IC: sat=%2d sys=%2d k=%2d IC=%2d cdtr=%.9f\n",
                    sat, sys, k, idx_clk, cdtr_val);
            }
            cdtr=x[IC(k,opt)];//接收机钟差
            H[IC(k,opt)+nx*nv]=1.0;//2.dtr系数为1.H阵中接收机钟差项对应系数为1
            
            if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {//当前conf选的是TROPOPT_EST
                for (k=0;k<(opt->tropopt>=TROPOPT_ESTG?3:1);k++) {
                    H[IT(opt)+k+nx*nv]=dtdx[k];//3.对流层系数设置为投影函数
                }
            }
            if (opt->ionoopt==IONOOPT_EST) {
                if (rtk->x[II(sat,opt)]==0.0) continue;
                H[II(sat,opt)+nx*nv]=C;
            }
            if (j/2==2&&j%2==1) { /* L5-receiver-dcb */
                dcb+=rtk->x[ID(opt)];
                H[ID(opt)+nx*nv]=1.0;
            }
            if (j%2==0) { /* phase bias 观测值是载波相位*/
                if ((bias=x[IB(sat,j/2,opt)])==0.0) //若该模糊度=0，说明有周跳，舍弃
                    continue;
                H[IB(sat,j/2,opt)+nx*nv]=1.0;//4.模糊度参数系数为1
            }


            /* residual */
            v[nv]=y-(r+cdtr-CLIGHT*dts[i*2]+dtrp+C*dion+dcb+bias);//残差（一行相位‘s一行伪距）IF组合的参数C=0
            

            trace(2, "[DEBUG] sat=%2d freq=%d type=%s y=%.4f r=%.4f cdtr=%.4f dts=%.4f dtrp=%.4f dion=%.4f dcb=%.4f bias=%.4f -> v=%.4f\n",
                sat, j / 2 + 1, j % 2 == 0 ? "L" : "P", y, r, cdtr, CLIGHT * dts[i * 2], dtrp, C * dion, dcb, bias, v[nv]);


            if (j%2==0) rtk->ssat[sat-1].resc[j/2]=v[nv];//相位残差
            else        rtk->ssat[sat-1].resp[j/2]=v[nv];//伪距残差
            
            /* variance */
            var[nv]=varerr(obs[i].sat,sys,azel[1+i*2],j/2,j%2,opt)+vart+SQR(C)*vari+var_rs[i];
            //方差噪声剔除质量不好的卫星，一般是信噪比和截止高度角来作为筛选标准,各观测值方差求和，用于构建R阵
            if (sys==SYS_GLO&&j%2==1) var[nv]+=VAR_GLO_IFB;//格洛纳斯伪距加大方差，噪声还需要+IFB(频间噪声)
            trace(4,"%s sat=%2d %s%d res=%9.4f sig=%9.4f el=%4.1f\n",str,sat,
                  j%2?"P":"L",j/2+1,v[nv],sqrt(var[nv]),azel[1+i*2]*R2D);
            
            /*//排除一些卫星,当残差是验前残差（！post=1 EKF之前）时，若＞限差30则 reject satellite by pre-fit residuals */
            if (!post&&opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
                trace(2,"outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
                      post,str,sat,j%2?"P":"L",j/2+1,v[nv],azel[1+i*2]*R2D);
                exc[i]=1; rtk->ssat[sat-1].rejc[j%2]++;
                continue;
            }    
            /* record large post-fit residuals//当残差是验后残差(post!=0 EKF后)时，若>4倍std，则记录下这些残差于ve中 */
            if (post&&fabs(v[nv])>sqrt(var[nv])*THRES_REJECT) {//拒绝的卫星数量
                obsi[ne]=i; frqi[ne]=j; ve[ne]=v[nv]; ne++;
            }
            if (j%2==0) rtk->ssat[sat-1].vsat[j/2]=1;
            nv++;
        }
    }
    /* reject satellite with large and max post-fit residual */
    if (post&&ne>0)
    {
        vmax=ve[0]; maxobs=obsi[0]; maxfrq=frqi[0]; rej=0;
        for (j=1;j<ne;j++) {
            if (fabs(vmax)>=fabs(ve[j])) continue;
            vmax=ve[j]; maxobs=obsi[j]; maxfrq=frqi[j]; rej=j;
        }
        sat=obs[maxobs].sat; //这边每次找残差max值
        trace(2,"outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
            post,str,sat,maxfrq%2?"P":"L",maxfrq/2+1,vmax,azel[1+maxobs*2]*R2D);
        exc[maxobs]=1; rtk->ssat[sat-1].rejc[maxfrq%2]++; stat=0;  //只要大于四倍std残差没有全部剔除完，就来到此处，stat会被置0
        ve[rej]=0;
    }
    for (i=0;i<nv;i++) for (j=0;j<nv;j++) {//利用方差构造R阵（方差用到了高度角模型，R阵是对角阵），这里就是取权重
        R[i+j*nv]=i==j?var[i]:0.0;
    }
    trace(2, "PPP_RES: post=%d nv=%d n=%d\n", post, nv, n);
    return post?stat:nv; // 验前阶段(post = 0)返回有效方程个数，验后阶段(post != 0)返回状态值，0: > 4倍std的残差未全部别完，1:...全部别完
}
/* number of estimated states ------------------------------------------------*/
extern int pppnx(const prcopt_t *opt)
{
    return NX(opt);
}
/* update solution status ----------------------------------------------------*/
static void update_stat(rtk_t *rtk, const obsd_t *obs, int n, int stat)
{
    const prcopt_t *opt=&rtk->opt;
    int i,j;
    
    /* test # of valid satellites */
    rtk->sol.ns=0;
    for (i=0;i<n&&i<MAXOBS;i++) {
        for (j=0;j<opt->nf;j++) {
            if (!rtk->ssat[obs[i].sat-1].vsat[j]) continue;
            rtk->ssat[obs[i].sat-1].lock[j]++;
            rtk->ssat[obs[i].sat-1].outc[j]=0;
            if (j==0) rtk->sol.ns++;
        }
    }
    rtk->sol.stat=rtk->sol.ns<MIN_NSAT_SOL?SOLQ_NONE:stat;
    
    if (rtk->sol.stat==SOLQ_FIX) {
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->xa[i];
            rtk->sol.qr[i]=(float)rtk->Pa[i+i*rtk->na];
        }
        rtk->sol.qr[3]=(float)rtk->Pa[1];
        rtk->sol.qr[4]=(float)rtk->Pa[1+2*rtk->na];
        rtk->sol.qr[5]=(float)rtk->Pa[2];
    }
    else {
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->x[i];
            rtk->sol.qr[i]=(float)rtk->P[i+i*rtk->nx];
        }
        rtk->sol.qr[3]=(float)rtk->P[1];
        rtk->sol.qr[4]=(float)rtk->P[2+rtk->nx];
        rtk->sol.qr[5]=(float)rtk->P[2];
    }
    rtk->sol.dtr[0]=rtk->x[IC(0,opt)];
    rtk->sol.dtr[1]=rtk->x[IC(1,opt)]-rtk->x[IC(0,opt)];
    
    for (i=0;i<n&&i<MAXOBS;i++) for (j=0;j<opt->nf;j++) {
        rtk->ssat[obs[i].sat-1].snr[j]=obs[i].SNR[j];
    }
    for (i=0;i<MAXSAT;i++) for (j=0;j<opt->nf;j++) {
        if (rtk->ssat[i].slip[j]&3) rtk->ssat[i].slipc[j]++;
        if (rtk->ssat[i].fix[j]==2&&stat!=SOLQ_FIX) rtk->ssat[i].fix[j]=1;
    }
}
/* test hold ambiguity -------------------------------------------------------*/
static int test_hold_amb(rtk_t *rtk)
{
    int i,j,stat=0;
    
    /* no fix-and-hold mode */
    if (rtk->opt.modear!=ARMODE_FIXHOLD) return 0;
    
    /* reset # of continuous fixed if new ambiguity introduced */
    for (i=0;i<MAXSAT;i++) {
        if (rtk->ssat[i].fix[0]!=2&&rtk->ssat[i].fix[1]!=2) continue;
        for (j=0;j<MAXSAT;j++) {
            if (rtk->ssat[j].fix[0]!=2&&rtk->ssat[j].fix[1]!=2) continue;
            if (!rtk->ambc[j].flags[i]||!rtk->ambc[i].flags[j]) stat=1;
            rtk->ambc[j].flags[i]=rtk->ambc[i].flags[j]=1;
        }
    }
    if (stat) {
        rtk->nfix=0;
        return 0;
    }
    /* test # of continuous fixed */
    return ++rtk->nfix>=rtk->opt.minfix;
}
/* precise point positioning ---
 args 
rtk_t* rtk       IO  rtk控制结构体
obsd_t* obs      I   OBS观测数据
int      n       I   OBS观测数据的数量
nav_t* nav       I   导航数据
 return 
   none
----------------------------------------------*/
extern void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt=&rtk->opt;
    double *rs,*dts,*var,*v,*H,*R,*azel,*xp,*Pp,dr[3]={0},std[3];
    char str[32];
    int i,j,nv,info,svh[MAXOBS],exc[MAXOBS]={0},stat=SOLQ_SINGLE;
    
    time2str(obs[0].time,str,2);
    trace(3,"pppos   : time=%s nx=%d n=%d\n",str,rtk->nx,n);
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel=zeros(2,n);
    
    for (i=0;i<MAXSAT;i++) for (j=0;j<opt->nf;j++) rtk->ssat[i].fix[j]=0;
    
    /* temporal update of ekf states      EKF首个历元的初始化，X阵和P阵*/
    udstate_ppp(rtk,obs,n,nav);
    //状态方程的更新 X，P(协方差)，时间更新



    //     /* ========== 使用tracemat调试输出 ========== */
    //trace(3, "\n========== PPP State Update Debug Info ==========\n");
    //trace(3, "time: %s\n", str);
    //trace(3, "State vector dimension: nx=%d\n", rtk->nx);
    //trace(3, "Number of observations: n=%d\n", n);

    ///* 打印状态向量x */
    //trace(3, "State vectorx=\n");
    //tracemat(3, rtk->x, 1, rtk->nx, 13, 4);

    ///* 打印协方差矩阵P */
    //trace(3, "Covariance matrix P=\n");
    //tracemat(3, rtk->P, rtk->nx, rtk->nx, 13, 4);

   
    /* ========== 调试输出结束 ========== */


    
    /* satellite positions and clocks */
    satposs(obs[0].time,obs,n,nav,rtk->opt.sateph,rs,dts,var,svh);
    //广播星历n文件，SPP精密星历sp3，精度在2cm以下
    //精密星历SP3,CLK，精度在2cm一下，ppp
    //精密位置和精密钟差都求出来了

    /* exclude measurements of eclipsing satellite (block IIA) */
    if (rtk->opt.posopt[3]) {
        testeclipse(obs,n,nav,rs);
    }
    /* earth tides correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rtk->x,opt->tidecorr==1?1:7,&nav->erp,
                 opt->odisp[0],dr);
    }
    //固体潮，海潮，极移等进行修正做掉，用dr来存储,课本讲得更详细
    //EKF参数初始化
    nv=n*rtk->opt.nf*2+MAXSAT+3;//位置参数的个数,尽量先去大的值，n为可视卫星个数，不含指定双频的本系统卫星也包含在内
    xp=mat(rtk->nx,1); 
    Pp=zeros(rtk->nx,rtk->nx);//EKF后的X和P
    v=mat(nv,1);        //残差矩阵
    H=mat(rtk->nx,nv);  
    R=mat(nv,nv);       //观测卫星的噪声协方差矩阵，可以取权重，来自观测方程故为nv*nv
    //H阵的一列：[-E  0  1  M  I ] 相位
    //H阵的一列：[-E  0  1  M  0 ] 伪距
    
    for (i=0;i<MAX_ITER;i++) {
        
        matcpy(xp,rtk->x,rtk->nx,1);//进行了结果的复制，方便书写？
        matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
        
        /* prefit residuals 验前*/

        if (!(nv=ppp_res(0,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel))) {
            trace(2,"%s ppp (%d) no valid obs data\n",str,i+1);
            break;
        }//第一次进这里，后面的矩阵在下面进行准备
        /* adaptive robust test before Kalman update */
        /*arekf_ppp(rtk,Pp,H,v,R,nv);*/
        /* measurement update of ekf states */
        if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv))) {//观测值的更新部分
            trace(2,"%s ppp (%d) filter error info=%d\n",str,i+1,info);
            break;
        }
        /* postfit residuals 验后*/
        if (ppp_res(i+1,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel)) {
            matcpy(rtk->x,xp,rtk->nx,1);  //用本历元ppp结果xp更新原(上一历元ppp)结果rtk->xppp所得模糊度为浮点解
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
            stat=SOLQ_PPP;
            break;
        }
    }
    if (i>=MAX_ITER) {
        trace(2,"%s ppp (%d) iteration overflows\n",str,i);
    }
    if (stat==SOLQ_PPP) {
        
        /* ambiguity resolution in ppp *///这个版本的模糊度固定是空的
        if (ppp_ar(rtk,obs,n,exc,nav,azel,xp,Pp)&&
            ppp_res(9,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel)) {
            
            matcpy(rtk->xa,xp,rtk->nx,1);
            matcpy(rtk->Pa,Pp,rtk->nx,rtk->nx);
            
            for (i=0;i<3;i++) std[i]=sqrt(Pp[i+i*rtk->nx]);
            if (norm(std,3)<MAX_STD_FIX) stat=SOLQ_FIX;
        }
        else {
            rtk->nfix=0;
        }
        /* update solution status */
        update_stat(rtk,obs,n,stat);
        
        /* hold fixed ambiguities */
        if (stat==SOLQ_FIX&&test_hold_amb(rtk)) {
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
            trace(2,"%s hold ambiguity\n",str);
            rtk->nfix=0;
        }
    }
    free(rs); free(dts); free(var); free(azel);
    free(xp); free(Pp); free(v); free(H); free(R);
}
