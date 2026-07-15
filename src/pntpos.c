/*------------------------------------------------------------------------------
* pntpos.c : standard positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.0  moved from rtkcmn.c
*                           changed api:
*                               pntpos()
*                           deleted api:
*                               pntvel()
*           2011/01/12 1.1  add option to include unhealthy satellite
*                           reject duplicated observation data
*                           changed api: ionocorr()
*           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
*           2012/12/25 1.3  add variable snr mask
*           2014/05/26 1.4  support galileo and beidou
*           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
*           2018/10/10 1.6  support api change of satexclude()
*           2020/11/30 1.7  support NavIC/IRNSS in pntpos()
*                           no support IONOOPT_LEX option in ioncorr()
*                           improve handling of TGD correction for each system
*                           use E1-E5b for Galileo dual-freq iono-correction
*                           use API sat2freq() to get carrier frequency
*                           add output of velocity estimation error in estvel()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/

#define SQR(x)      ((x)*(x))

#if 0 /* enable GPS-QZS time offset estimation */
#define NX          (4+5)       /* # of estimated parameters */
#else
#define NX          (4+4)       /* # of estimated parameters */
#endif
#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay Std (m) */
#define ERR_TROP    3.0         /* tropspheric delay Std (m) */
#define ERR_SAAS    0.3         /* Saastamoinen model error Std (m) */
#define ERR_BRDCI   0.5         /* broadcast ionosphere model error factor */
#define ERR_CBIAS   0.3         /* code bias error Std (m) */
#define REL_HUMI    0.7         /* relative humidity for Saastamoinen model */
#define MIN_EL      (5.0*D2R)   /* min elevation for measurement error (rad) */

/* pseudorange measurement error variance ----

prcopt_t  *opt    I   processing options
double     el     I   elevation angle (rad)
int        sys    I   所属的导航系统
return:
double     varr   -   导航系统伪距测量值的误差

--------------------------------*/
static double varerr(const prcopt_t *opt, double el, int sys)
{
    double fact,varr;
    fact=sys==SYS_GLO?EFACT_GLO:(sys==SYS_SBS?EFACT_SBS:EFACT_GPS);
    if (el<MIN_EL) el=MIN_EL;//果高度角小于5度，则按5度计算
    varr=SQR(opt->err[0])*(SQR(opt->err[1])+SQR(opt->err[2])/sin(el));
    if (opt->ionoopt==IONOOPT_IFLC) varr*=SQR(3.0); /* iono-free */
    return SQR(fact)*varr;
}
/* select BDS ephemeris for the signal-specific group delay ----------------*/
static const eph_t *selbdseph_tgd(gtime_t time, int sat, const nav_t *nav,
                                     int type)
{
    double t,tmin=MAXDTOE_CMP+2.0;
    int i,j=-1,want_cnv1=type>=2;

    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].sat!=sat) continue;
        if ((nav->eph[i].code==EPHCODE_BDS_CNV1)!=want_cnv1) continue;
        if ((t=fabs(timediff(nav->eph[i].toe,time)))>MAXDTOE_CMP+1.0) continue;
        if (t<=tmin) {j=i; tmin=t;}
    }
    return j<0?NULL:nav->eph+j;
}
/* get group delay parameter (m) ---------------------------------------------*/
static double gettgd(gtime_t time, int sat, const nav_t *nav, int type)
{
    const eph_t *eph;
    int i,sys=satsys(sat,NULL);
    
    if (sys==SYS_GLO) {
        for (i=0;i<nav->ng;i++) {
            if (nav->geph[i].sat==sat) break;
        }
        return (i>=nav->ng)?0.0:-nav->geph[i].dtaun*CLIGHT;
    }
    else if (sys==SYS_CMP) {
        if (!(eph=selbdseph_tgd(time,sat,nav,type))) return 0.0;
        return eph->tgd[type]*CLIGHT;
    }
    else {
        for (i=0;i<nav->n;i++) {
            if (nav->eph[i].sat==sat) break;
        }
        return (i>=nav->n)?0.0:nav->eph[i].tgd[type]*CLIGHT;
    }
}
/* test SNR mask -------------------------------------------------------------*/
static int snrmask(const obsd_t *obs, const double *azel, const prcopt_t *opt)
{
    if (testsnr(0,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask)) {
        return 0;
    }
    if (opt->ionoopt==IONOOPT_IFLC) {
        if (testsnr(0,1,azel[1],obs->SNR[1]*SNR_UNIT,&opt->snrmask)) return 0;
    }
    return 1;
}
/* psendorange with code bias correction ---做DCB& tgd改正----
obsd_t    *obs      I   观测数据
nav_t     *nav      I   导航数据
double    *azel     I   对于当前定位值，每一颗观测卫星的 {方位角、高度角}
int        iter     I   迭代次数
prcopt_t  *opt      I   配置参数
double    *vare     O   伪距测量的码偏移误差

return:
double     P1       -   最终参与定位解算的伪距值

------------------------------*/
static double prange(const obsd_t *obs, const nav_t *nav, const prcopt_t *opt,
                     double *var)
{
    double P1,P2,gamma,b1,b2;                   /*P1, P2: 存储伪距观测值（对应于不同频率的伪距）。
                                                gamma: 频率比值，通常用于双频或多频修正。
                                                b1, b2: 存储时钟差（TGD），用于修正伪距。
                                                sat, sys: 分别代表卫星编号和卫星系统类型。*/
    int sat,sys;
    
    sat=obs->sat;           // 获取卫星编号
    sys=satsys(sat,NULL);   // 获取卫星系统（GPS, GLONASS, Galileo等）
    P1=obs->P[0];           // 获取伪距 P1
    P2=obs->P[1];           // 获取伪距 P2
    *var=0.0;               // 初始化误差值为0
    
    if (P1==0.0||(opt->ionoopt==IONOOPT_IFLC&&P2==0.0)) return 0.0; // 如果 P1 或者 采用双频方法但是P2 无效，则返回0
    
    /* P1-C1,P2-C2 DCB correction */
    if (sys==SYS_GPS||sys==SYS_GLO) {
        if (obs->code[0]==CODE_L1C) P1+=nav->cbias[sat-1][1]; /* C1->P1 */// 修正 C1 -> P1
        if (obs->code[1]==CODE_L2C) P2+=nav->cbias[sat-1][2]; /* C2->P2 */// 修正 C2 -> P2
    }
    if (opt->ionoopt==IONOOPT_IFLC) { /* dual-frequency */// 如果采用双频修正（IFLC：双频伪距修正）
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1-L2,G1-G2 */
            gamma=SQR(FREQ1/FREQ2);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GLO) { /* G1-G2 */
            gamma=SQR(FREQ1_GLO/FREQ2_GLO);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GAL) { /* E1-E5b */
            gamma=SQR(FREQ1/FREQ7);
            if (getseleph(SYS_GAL)) { /* F/NAV */
                P2-=gettgd(obs->time,sat,nav,0)-gettgd(obs->time,sat,nav,1); /* BGD_E5aE5b */
            }
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_CMP) { /* B1-B2 */
            gamma=SQR(((obs->code[0]==CODE_L2I)?FREQ1_CMP:FREQ1)/FREQ2_CMP);
            if      (obs->code[0]==CODE_L2I) b1=gettgd(obs->time,sat,nav,0); /* TGD_B1I*/
            else if (obs->code[0]==CODE_L1P) b1=gettgd(obs->time,sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(obs->time,sat,nav,2)+gettgd(obs->time,sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            b2=gettgd(obs->time,sat,nav,1); /* TGD_B2I/B2bI (m) *///B2对应的TGD
            return ((P2-gamma*P1)-(b2-gamma*b1))/(1.0-gamma);//完整的TGD修正双频伪距组合
        }
        else if (sys==SYS_IRN) { /* L5-S */
            gamma=SQR(FREQ5/FREQ9);
            return (P2-gamma*P1)/(1.0-gamma);
        }
    }

    /*单频情况，直接修正tgd*/
    else { /* single-freq (L1/E1/B1) */
        *var=SQR(ERR_CBIAS);
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1 */// 对于 GPS 或 QZS
            b1=gettgd(obs->time,sat,nav,0); /* TGD (m) */// 获取 TGD（卫星与接收机之间的时钟差
            return P1-b1;                    // 修正 P1
        }
        else if (sys==SYS_GLO) { /* G1 */
            gamma=SQR(FREQ1_GLO/FREQ2_GLO);
            b1=gettgd(obs->time,sat,nav,0); /* -dtaun (m) */
            return P1-b1/(gamma-1.0);
        }
        else if (sys==SYS_GAL) { /* E1 */
            if (getseleph(SYS_GAL)) b1=gettgd(obs->time,sat,nav,0); /* BGD_E1E5a */
            else                    b1=gettgd(obs->time,sat,nav,1); /* BGD_E1E5b */
            return P1-b1;
        }
        else if (sys==SYS_CMP) { /* B1I/B1Cp/B1Cd */
            if      (obs->code[0]==CODE_L2I) b1=gettgd(obs->time,sat,nav,0); /* TGD_B1I */
            else if (obs->code[0]==CODE_L1P) b1=gettgd(obs->time,sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(obs->time,sat,nav,2)+gettgd(obs->time,sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            return P1-b1;
        }
        else if (sys==SYS_IRN) { /* L5 */
            gamma=SQR(FREQ9/FREQ5);
            b1=gettgd(obs->time,sat,nav,0); /* TGD (m) */
            return P1-gamma*b1;// 默认返回未修正的 P1（理论上不会执行到）
        }
    }
    return P1;//默认返回未修正的P1
}


/*自行增加的支持多频点伪距码偏差修正*/
/*static double prange_mulfreq(const obsd_t *obs, const nav_t *nav, const prcopt_t *opt,
                              double *var) {
    double P1, P2, P3, gamma1, gamma2, gamma3, b1, b2, b3;
    int sat, sys;

    sat = obs->sat;           // 获取卫星编号
    sys = satsys(sat, NULL);  // 获取卫星系统
    P1 = obs->P[0];           // 第一频点伪距
    P2 = obs->P[1];           // 第二频点伪距
    P3 = obs->P[2];           // 第三频点伪距（如果适用）
    *var = 0.0;               // 初始化误差为0

    // 检查伪距有效性
    if (P1 == 0.0 || P2 == 0.0 || P3 == 0.0) return 0.0;

    if (sys == SYS_GPS || sys == SYS_QZS) {  // GPS 或 QZS
        gamma1 = SQR(FREQ1 / FREQ2);         // 频率比
        gamma2 = SQR(FREQ1 / FREQ3);         // 频率比（第三频点）
        return (P2 - gamma1 * P1) / (1.0 - gamma1) - (P3 - gamma2 * P1) / (1.0 - gamma2);
    }
    else if (sys == SYS_GLO) {  // GLONASS
        gamma1 = SQR(FREQ1_GLO / FREQ2_GLO);  // 频率比
        return (P2 - gamma1 * P1) / (1.0 - gamma1);
    }
    else if (sys == SYS_GAL) {  // Galileo
        gamma1 = SQR(FREQ1 / FREQ7);           // 频率比
        if (getseleph(SYS_GAL)) {              // 如果是 F/NAV 系统
            P2 -= gettgd(sat, nav, 0) - gettgd(sat, nav, 1);  // 校正 BGD_E5aE5b
        }
        return (P2 - gamma1 * P1) / (1.0 - gamma1);
    }
    else if (sys == SYS_CMP) {  // BeiDou
        gamma1 = SQR(FREQ1_CMP / FREQ2_CMP);   // 频率比
        b1 = gettgd(sat, nav, 0);              // TGD_B1I
        b2 = gettgd(sat, nav, 1);              // TGD_B2I
        return ((P2 - gamma1 * P1) - (b2 - gamma1 * b1)) / (1.0 - gamma1);
    }
    else if (sys == SYS_IRN) {  // IRNSS
        gamma1 = SQR(FREQ5 / FREQ9);           // 频率比
        return (P2 - gamma1 * P1) / (1.0 - gamma1);
    }

    return P1;  // 默认返回 P1
}
*/


/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
                    const double *azel, int ionoopt, double *ion, double *var)
{
    trace(4,"ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),ionoopt,sat,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* GPS broadcast ionosphere model */
    if (ionoopt==IONOOPT_BRDC) {
        *ion=ionmodel(time,nav->ion_gps,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    /* SBAS ionosphere model */
    if (ionoopt==IONOOPT_SBAS) {
        return sbsioncorr(time,nav,pos,azel,ion,var);
    }
    /* IONEX TEC model */
    if (ionoopt==IONOOPT_TEC) {
        return iontec(time,nav,pos,azel,1,ion,var);
    }
    /* QZSS broadcast ionosphere model */
    if (ionoopt==IONOOPT_QZS&&norm(nav->ion_qzs,8)>0.0) {
        *ion=ionmodel(time,nav->ion_qzs,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    *ion=0.0;
    *var=ionoopt==IONOOPT_OFF?SQR(ERR_ION):0.0;
    return 1;
}
/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, int tropopt, double *trp, double *var)
{
    trace(4,"tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),tropopt,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* Saastamoinen model */
    if (tropopt==TROPOPT_SAAS||tropopt==TROPOPT_EST||tropopt==TROPOPT_ESTG) {
        *trp=tropmodel(time,pos,azel,REL_HUMI);
        *var=SQR(ERR_SAAS/(sin(azel[1])+0.1));
        return 1;
    }
    /* SBAS (MOPS) troposphere model */
    if (tropopt==TROPOPT_SBAS) {
        *trp=sbstropcorr(time,pos,azel,var);
        return 1;
    }
    /* no correction */
    *trp=0.0;
    *var=tropopt==TROPOPT_OFF?SQR(ERR_TROP):0.0;
    return 1;
}

/*int      iter      I   迭代次数，在estpos()里迭代调用，第i次迭代就传i
obsd_t   *obs      I   观测量数据
int      n         I   观测量数据的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
double   *vare     I   卫星位置和钟差的协方差 (m^2)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   导航数据
double   *x        I   本次迭代开始之前的定位值,7*1,前3个是本次迭代开始之前的定位值，第4个是钟差，后三个分别是gps系统与glonass、galileo、bds系统的钟差。
prcopt_t *opt      I   处理过程选项
double   *v        O   定位方程的右端部分，伪距残差
double   *H        O   定位方程中的几何矩阵
double   *var      O   参与定位的伪距残差的方差
double   *azel     O   对于当前定位值，所有观测卫星的 {方位角、高度角} (2*n)
int      *vsat     O   所有观测卫星在当前定位时是否有效 (1*n)
double   *resp     O   所有观测卫星的伪距残差，(P-(r+c*dtr-c*dts+I+T)) (1*n)
int      *ns       O   参与定位的卫星的个数

return：
int   nv     - 定位方程组的方程个数

*/
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *vare, const int *svh,
                   const nav_t *nav, const double *x, const prcopt_t *opt,
                   double *v, double *H, double *var, double *azel, int *vsat,
                   double *resp, int *ns)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,rr[3],pos[3],dtr,e[3],P;
    int i,j,nv=0,sat,sys,mask[NX-3]={0};
    
    trace(3,"resprng : n=%d\n",n);
    //将之前得到的定位解信息赋值给 rr 和 dtr 数组，以进行关于当前解的伪距残差的相关计算
    for (i=0;i<3;i++) rr[i]=x[i];//获取接收机位置 ， rr{x,y,z}->pos{lat,lon,h}  
    dtr=x[3];//获取接收机钟差
    
    ecef2pos(rr,pos);// 将ECEF坐标转换为地理坐标,到接收机位置 pos{lat,lon,h}，单位为弧度和米
    
    for (i=*ns=0;i<n&&i<MAXOBS;i++) {
        vsat[i]=0; azel[i*2]=azel[1+i*2]=resp[i]=0.0;// 将vsat、azel和resp数组置 0，因为在前后两次定位结果中，每颗卫星的上述信息都会发生变化。
        time=obs[i].time;//观测时间
        sat=obs[i].sat;//卫星编号
        if (!(sys=satsys(sat,NULL))) continue;//调用satsys()函数，验证卫星编号是否合理及其所属的导航系统
        
        /* reject duplicated observation data （检查重复数据）*/
        if (i<n-1&&i<MAXOBS-1&&sat==obs[i+1].sat) {
            trace(2,"duplicated obs data %s sat=%d\n",time_str(time,3),sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;
        //调用 geodist 函数，计算卫星和当前接收机位置之间的几何距离 r和接收机到卫星方向的观测矢量。
        //然后检验几何距离是否 >0。此函数中会进行地球自转影响的校正（Sagnac效应）
        /* geometric distance */
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;//计算卫地距的几何距离，卫地距单位向量e
        
        if (iter>0) {
            /* test elevation mask */
            if (satazel(pos,e,azel+i*2)<opt->elmin) continue;
            // 调用 satazel 函数，计算在接收机位置处的站心坐标系中卫星的方位角和仰角；若仰角低于截断值，不处理此数据。
            /* test SNR mask */
            if (!snrmask(obs+i,azel+i*2,opt)) continue;
            // 调用snrmask()->testsnr()，根据接收机高度角和信号频率来检测该信号是否可用
            /* ionospheric correction */
            if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {//这里计算的是GPS L1频段的，跟其他系统存在区别
                continue;
            }
            if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;//获取卫星频率
            dion*=SQR(FREQ1/freq);//电离层误差修正
            vion*=SQR(FREQ1/freq);//电离层误差的方程修正
            
            /* tropospheric correction */
            if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
                continue;
            }
        }
		else {
			dion=vion=dtrp=vtrp=0.0; //首个历元，dion和dtrp都为0，不加电离层对流层改正
		}
        /* psendorange with code bias correction */
        if ((P=prange(obs+i,nav,opt,&vmeas))==0.0) continue;//伪距测量与代码偏差修正
         
        /* pseudorange residual */
        v[nv]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);// 伪距残差，观测值与理论值的差异，var[i]过大，表示第i个方程不可信
        
        /* design matrix */
        for (j=0;j<NX;j++) {
            H[j+nv*NX]=j<3?-e[j]:(j==3?1.0:0.0);// 计算设计矩阵,每列为 x y z 1 0 0 0 0
        }
        /* time system offset and receiver bias correction  这一块吴桐讲得很好 */
        if      (sys==SYS_GLO) {v[nv]-=x[4]; H[4+nv*NX]=1.0; mask[1]=1;}
        else if (sys==SYS_GAL) {v[nv]-=x[5]; H[5+nv*NX]=1.0; mask[2]=1;}
        else if (sys==SYS_CMP) {v[nv]-=x[6]; H[6+nv*NX]=1.0; mask[3]=1;}
        else if (sys==SYS_IRN) {v[nv]-=x[7]; H[7+nv*NX]=1.0; mask[4]=1;}
#if 0 /* enable QZS-GPS time offset estimation */
        else if (sys==SYS_QZS) {v[nv]-=x[8]; H[8+nv*NX]=1.0; mask[5]=1;}
#endif
        else mask[0]=1;
        
        vsat[i]=1; resp[i]=v[nv]; (*ns)++;// 更新卫星状态和残差
        
        /* variance of pseudorange error */
        var[nv++]=varerr(opt,azel[1+i*2],sys)+vare[i]+vmeas+vion+vtrp;//包括观测值方差，卫星位置和钟的方差，电离层方差
        
        trace(4,"sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n",obs[i].sat,
              azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
    }
    /* constraint to avoid rank-deficient *///加约束防止秩亏
    for (i=0;i<NX-3;i++) {
        if (mask[i]) continue;
        v[nv]=0.0;// 设置残差为零
        for (j=0;j<NX;j++) H[j+nv*NX]=j==i+3?1.0:0.0;
        var[nv++]=0.01;
    }
    return nv;//返回残差的个数
}

/*
 args :
const double* azel     I  方位角、高度角
const int* vsat     I  观测卫星在当前定位时是否有效(1 * n)
int             n        I  观测值个数
const prcopt_t* opt      I  处理选项
const double* v        I  定位方程的右端部分，伪距残差
int             nv       I  观测值数
int             nx       O  待估计参数数
/* return */
//int             status - 1:ok, 0 : error
/* validate solution ---------------------------------------------------------*/
static int valsol(const double *azel, const int *vsat, int n,
                  const prcopt_t *opt, const double *v, int nv, int nx,
                  char *msg)
{
    double azels[MAXOBS*2],dop[4],vv;
    int i,ns;
    
    trace(3,"valsol  : n=%d nv=%d\n",n,nv);
    
    /* Chi-square validation of residuals *///利用所有残差平方和去符合卡方分布
    vv=dot(v,v,nv);
    if (nv>nx&&vv>chisqr[nv-nx-1]) {
        sprintf(msg,"chi-square error nv=%d vv=%.1f cs=%.1f",nv,vv,chisqr[nv-nx-1]);
        return 0;
    }
    /* large GDOP check *///定位精度因子的计算，
    for (i=ns=0;i<n;i++) {
        if (!vsat[i]) continue;
        azels[  ns*2]=azel[  i*2];
        azels[1+ns*2]=azel[1+i*2];
        ns++;
    }
    dops(ns,azels,opt->elmin,dop);
    if (dop[0]<=0.0||dop[0]>opt->maxgdop) {
        sprintf(msg,"gdop error nv=%d gdop=%.1f",nv,dop[0]);
        return 0;
    }
    return 1;
}
/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const double *vare, const int *svh, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel, int *vsat,
                  double *resp, char *msg)
{
    double x[NX]={0},dx[NX],Q[NX*NX],*v,*H,*var,sig;
    int i,j,k,info,stat,nv,ns;
    
    trace(3,"estpos  : n=%d\n",n);
    
    v=mat(n+4,1); H=mat(NX,n+4); var=mat(n+4,1);//h：八行（未知数个数）
    
    for (i=0;i<3;i++) x[i]=sol->rr[i];//首个历元，sol为空，x=0；非首历元泽取上一个结果作为初值
    
    for (i=0;i<MAXITR;i++) {
        //计算伪距残差v(&方差var)+构建H阵， = Hx + e;x阵:xyz+dtr  v 阵，加了各种改正项的伪距-卫地距
        /* pseudorange residuals (m) */
        nv=rescode(i,obs,n,rs,dts,vare,svh,nav,x,opt,v,H,var,azel,vsat,resp,
                   &ns);
        //nv是有效卫星的数量，ns是有效卫星的编号
        if (nv<NX) {//方程个数小于未知数个数，跳过
            sprintf(msg,"lack of valid sats ns=%d",nv);
            break;
        }
        /* weighted by Std 方程定权，重点*/
        for (j=0;j<nv;j++) {
            sig=sqrt(var[j]);
            v[j]/=sig;//左乘半权阵，v继续保持m*1
            for (k=0;k<NX;k++) H[k+j*NX]/=sig;//右乘半权阵，H继续保持n*m
        }
        /* least square estimation */
        if ((info=lsq(H,v,NX,nv,dx,Q))) {//H是转置了的
            sprintf(msg,"lsq error info=%d",info);
            break;
        }
        for (j=0;j<NX;j++) {//初值+改正值
            x[j]+=dx[j];//概率位置回带回接收机钟的位置
        }
        if (norm(dx,NX)<1E-4) {//做差值判断变化量是否收敛了，改正数足够小的时候，存储结果结束迭代
            sol->type=0;//坐标采用xyz-ecef
            sol->time=timeadd(obs[0].time,-x[3]/CLIGHT);//修正gps的接收机钟偏,去除了dtr后的信号发射瞬间的卫星钟面时刻
            sol->dtr[0] = x[3] / CLIGHT; /* receiver clock bias (s) 程序原本的版本*/
   //         switch(opt->navsys) {
			//	case SYS_GPS: sol->dtr[0] = x[3] / CLIGHT; break; /* GPS time */
			//	case SYS_GLO: sol->dtr[0] = x[4] / CLIGHT; break; /* GLO time */
			//	case SYS_GAL: sol->dtr[0] = x[5] / CLIGHT; break; /* GAL time */
   //             case SYS_CMP: sol->dtr[0] = x[6] / CLIGHT; break; /* BDS time 北斗的接收机钟差赋值*/
   //             case SYS_IRN: sol->dtr[0] = x[7] / CLIGHT; break;    
			//}
            /*以上是添加多系统的原因*/
            sol->dtr[1]=x[4]/CLIGHT; /* GLO-GPS time offset (s)单系统定位，以下四者都为零，无用 */
            sol->dtr[2]=x[5]/CLIGHT; /* GAL-GPS time offset (s) */
            sol->dtr[3]=x[6]/CLIGHT; /* BDS-GPS time offset (s) */
            sol->dtr[4]=x[7]/CLIGHT; /* IRN-GPS time offset (s) */
            for (j=0;j<6;j++) sol->rr[j]=j<3?x[j]:0.0;//更新了位置
            for (j=0;j<3;j++) sol->qr[j]=(float)Q[j+j*NX];//存位置的对角线方差也放在里面
            sol->qr[3]=(float)Q[1];    /* cov xy */
            sol->qr[4]=(float)Q[2+NX]; /* cov yz */
            sol->qr[5]=(float)Q[2];    /* cov zx *///存协方差
            sol->ns=(uint8_t)ns;
            sol->age=sol->ratio=0.0;
            
            /* validate solution *///结果检验
            if ((stat=valsol(azel,vsat,n,opt,v,nv,NX,msg))) {           //返回值stat=1表示检验通过,0表示检验不通过
                sol->stat=opt->sateph==EPHOPT_SBAS?SOLQ_SBAS:SOLQ_SINGLE;//ppp的spp此步会=SOLQ_SINGLE
            }
            free(v); free(H); free(var);
            return stat;//精度的理想定位，到此结束spp的流程
        }
    }
    if (i>=MAXITR) sprintf(msg,"iteration divergent i=%d",i);
    
    free(v); free(H); free(var);
    return 0;
}
/* RAIM FDE (failure detection and exclution) -------
/* args 
const obsd_t* obs     I    OBS观测数据
int             n       I    观测数据的数量
const double* rs      I    卫星位置和速度，长度为6* n，{ x,y,z,vx,vy,vz }(ecef)(m, m / s)
const double* dts     I    卫星钟差，长度为2* n，{ bias,drift } (s | s / s)
const double* vare    I    卫星位置和钟差的协方差(m ^ 2)
const int* svh     I    卫星健康标志(-1:correction not available)
const nav_t* nav     I    导航数据
const prcopt_t* opt     I    处理过程选项
sol_t* sol     IO   解算结果
double* azel    IO   方位角和俯仰角(rad)
int* vsat    IO   表征卫星在定位时是否有效
double* resp    IO   观测卫星的伪距残差，(P - (r + c * dtr - c * dts + I + T)) (1 * n)
char* msg     O    错误信息
 return *
int             status - 1:ok, 0 : error------------------------*/
static int raim_fde(const obsd_t *obs, int n, const double *rs,
                    const double *dts, const double *vare, const int *svh,
                    const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                    double *azel, int *vsat, double *resp, char *msg)
{
    obsd_t *obs_e;
    sol_t sol_e={{0}};
    char tstr[32],name[16],msg_e[128];
    double *rs_e,*dts_e,*vare_e,*azel_e,*resp_e,rms_e,rms=100.0;
    int i,j,k,nvsat,stat=0,*svh_e,*vsat_e,sat=0;
    
    trace(3,"raim_fde: %s n=%2d\n",time_str(obs[0].time,0),n);
    
    if (!(obs_e=(obsd_t *)malloc(sizeof(obsd_t)*n))) return 0;
    rs_e = mat(6,n); dts_e = mat(2,n); vare_e=mat(1,n); azel_e=zeros(2,n);
    svh_e=imat(1,n); vsat_e=imat(1,n); resp_e=mat(1,n); 
    
    for (i=0;i<n;i++) {
        
        /* satellite exclution */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            obs_e[k]=obs[j];
            matcpy(rs_e +6*k,rs +6*j,6,1);
            matcpy(dts_e+2*k,dts+2*j,2,1);
            vare_e[k]=vare[j];
            svh_e[k++]=svh[j];
        }
        /* estimate receiver position without a satellite */
        if (!estpos(obs_e,n-1,rs_e,dts_e,vare_e,svh_e,nav,opt,&sol_e,azel_e,
                    vsat_e,resp_e,msg_e)) {
            trace(3,"raim_fde: exsat=%2d (%s)\n",obs[i].sat,msg);
            continue;
        }
        for (j=nvsat=0,rms_e=0.0;j<n-1;j++) {
            if (!vsat_e[j]) continue;
            rms_e+=SQR(resp_e[j]);
            nvsat++;
        }
        if (nvsat<5) {
            trace(3,"raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                  obs[i].sat,nvsat);
            continue;
        }
        rms_e=sqrt(rms_e/nvsat);
        
        trace(3,"raim_fde: exsat=%2d rms=%8.3f\n",obs[i].sat,rms_e);
        
        if (rms_e>rms) continue;
        
        /* save result */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            matcpy(azel+2*j,azel_e+2*k,2,1);
            vsat[j]=vsat_e[k];
            resp[j]=resp_e[k++];
        }
        stat=1;
        *sol=sol_e;
        sat=obs[i].sat;
        rms=rms_e;
        vsat[i]=0;
        strcpy(msg,msg_e);
    }
    //如果 stat不为 0，则说明在弃用卫星的前提下有更好的解出现，输出信息，指出弃用了哪颗卫星。
    if (stat) {
        time2str(obs[0].time,tstr,2); satno2id(sat,name);
        trace(2,"%s: %s excluded by raim\n",tstr+11,name);
    }
    free(obs_e);
    free(rs_e ); free(dts_e ); free(vare_e); free(azel_e);
    free(svh_e); free(vsat_e); free(resp_e);
    return stat;
}
/* range rate residual  s ------------填充设计矩阵和残差------------------------------------------*/
static int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const nav_t *nav, const double *rr, const double *x,
                  const double *azel, const int *vsat, double err, double *v,
                  double *H)
{
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    int i,j,nv=0;
    
    trace(3,"resdop  : n=%d\n",n);
    
    ecef2pos(rr,pos); xyz2enu(pos,E);
    
    for (i=0;i<n&&i<MAXOBS;i++) {
        
        freq=sat2freq(obs[i].sat,obs[i].code[0],nav);
        //多普勒观测不为0，卫星频率有效，速度向量不为0，去除定速时不可以用的卫星
        if (obs[i].D[0]==0.0||freq==0.0||!vsat[i]||norm(rs+3+i*6,3)<=0.0) {
            continue;
        }
        /* LOS (line-of-sight) vector in ECEF */
        cosel=cos(azel[1+i*2]);//通过高度角和方位角计算出在enu下的系数阵，OA,
        a[0]=sin(azel[i*2])*cosel;//e方向上的投影
        a[1]=cos(azel[i*2])*cosel;
        a[2]=sin(azel[1+i*2]);//U方向投影
        matmul("TN",3,1,3,1.0,E,a,0.0,e);//得到了设计矩阵，存在e里面，e=E^T*a
         
        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
            vs[j]=rs[j+3+i*6]-x[j];//卫星速度减去接收机速度
        }
        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);//（F.6.29）
        
        /* Std of range rate error (m/s) */
        sig=(err<=0.0)?1.0:err*CLIGHT/freq;//多普勒的噪声误差，不同高度角模型对于测速的影响
        
        /* range rate residual (m/s) */
        v[nv]=(-obs[i].D[0]*CLIGHT/freq-(rate+x[3]-CLIGHT*dts[1+i*2]))/sig;
        
        /* design matrix */
        for (j=0;j<4;j++) {
            H[j+nv*4]=((j<3)?-e[j]:1.0)/sig;//(E.6.28)
        }
        nv++;
    }
    return nv;
}
/* estimate receiver velocity --------------------
/* args 
obsd_t* obs      I   OBS观测数据
int       n        I   观测数据的数量
double* rs       I   卫星位置和速度，长度为6* n，{ x,y,z,vx,vy,vz }(ecef)(m, m / s)
double* dts      I   卫星钟差，长度为2* n，{ bias,drift } (s | s / s)
nav_t* nav      I   导航数据
prcopt_t* opt      I   处理过程选项
sol_t* sol      IO  solution
double* azel     IO  方位角和俯仰角(rad)
int* vsat     IO  定位时有效卫星
char* msg      O   错误消息
 return 
int       status - 1:ok，0:error----------------------------*/
static void estvel(const obsd_t *obs, int n, const double *rs, const double *dts,
                   const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                   const double *azel, const int *vsat)
{
    double x[4]={0},dx[4],Q[16],*v,*H;//单位时间的变化率都一样，接收机的速度加一个钟漂
    double err=opt->err[4]; /* Doppler error (Hz) */
    int i,j,nv;
    
    trace(3,"estvel  : n=%d\n",n);
    
    v=mat(n,1); H=mat(4,n);
    
    for (i=0;i<MAXITR;i++) {
        //调用 resdop，计算定速方程组左边的雅可比矩阵和右端的速度残余，返回定速时所使用的卫星数目
        /* range rate residuals (m/s) */
        if ((nv=resdop(obs,n,rs,dts,nav,sol->rr,x,azel,vsat,err,v,H))<4) {
            break;
        }
        //
        /* least square estimation */
        if (lsq(H,v,4,nv,dx,Q)) break;
        
        for (j=0;j<4;j++) x[j]+=dx[j];
        
        if (norm(dx,4)<1E-6) {
            matcpy(sol->rr+3,x,3,1);
            sol->qv[0]=(float)Q[0];  /* xx */
            sol->qv[1]=(float)Q[5];  /* yy */
            sol->qv[2]=(float)Q[10]; /* zz */
            sol->qv[3]=(float)Q[1];  /* xy */
            sol->qv[4]=(float)Q[6];  /* yz */
            sol->qv[5]=(float)Q[2];  /* zx */
            break;//缺乏对速度的校验查看他是否是正确的
        }
    }
    free(v); free(H);
}
/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int pntpos(const obsd_t *obs, int n, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat,
                  char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var,*azel_,*resp;
    int i,stat,vsat[MAXOBS]={0},svh[MAXOBS];
    
    trace(3,"pntpos  : tobs=%s n=%d\n",time_str(obs[0].time,3),n); //打印调试信息，仅用于debug模式
    
    sol->stat=SOLQ_NONE;  //输出状态设置为没有输出
    
    if (n<=0) {     //检验观测值数是否＞0
        strcpy(msg,"no observation data"); 
        return 0;
    }
    sol->time=obs[0].time; //输出时间设置为接收机时间
    msg[0]='\0';  //\0表示字符串结束的标志
    //生成矩阵，在内存里开普出6*n个double空间
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel_=zeros(2,n); resp=mat(1,n);
    
    //if (opt_.mode!=EPHOPT_BRDC) { /* for precise positioning */ //用于高精度定位，例如ppp会先用spp，来到此步时，不算是废代码
        opt_.sateph = EPHOPT_BRDC;
        opt_.ionoopt=IONOOPT_BRDC;//电离层改正采用广播星历klobchar模型
        opt_.tropopt=TROPOPT_SAAS;//对流层采用saastamoinen模型
    /*}*///北斗ppp的spp必须采用广播星历，否则不会通过卡方检验，需要补充代码
    /* satellite positons, velocities and clocks */
    satposs(sol->time,obs,n,nav,opt_.sateph,rs,dts,var,svh); //说白了是是算已知量。计算卫星的位置、钟差（&钟速）
    
    /* estimate receiver position with pseudorange */
    stat=estpos(obs,n,rs,dts,var,svh,nav,&opt_,sol,azel_,vsat,resp,msg);//spp计算测站位置，stat0-不OK：1-OK
    //上面两个是重点
    /* RAIM FDE *///接收机正值性检测，只对参与的卫星进行检测
    if (!stat&&n>=6&&opt->posopt[4]) { //estpos（）定位不良+>=六颗可见卫星+？，手册里面有提到为什么规定是六个卫星
        stat=raim_fde(obs,n,rs,dts,var,svh,nav,&opt_,sol,azel_,vsat,resp,msg);
    }
    /* estimate receiver velocity with Doppler *///用多普勒计算接收机的速度的钟漂
    //spp
    if (stat) {
        estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat);
    }
    if (azel) {
        for (i=0;i<n*2;i++) azel[i]=azel_[i];
    }

    //输出卫星状态
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].vs=0;
            ssat[i].azel[0]=ssat[i].azel[1]=0.0;
            ssat[i].resp[0]=ssat[i].resc[0]=0.0;
            ssat[i].snr[0]=0;
        }
        for (i=0;i<n;i++) {
            ssat[obs[i].sat-1].azel[0]=azel_[  i*2];
            ssat[obs[i].sat-1].azel[1]=azel_[1+i*2];
            ssat[obs[i].sat-1].snr[0]=obs[i].SNR[0];
            if (!vsat[i]) continue;
            ssat[obs[i].sat-1].vs=1;
            ssat[obs[i].sat-1].resp[0]=resp[i];//输出残差
        }
    }
    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}
