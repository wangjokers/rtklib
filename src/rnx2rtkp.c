/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*           2014/01/27  1.7 fix bug on default output time format
*           2015/05/15  1.8 -r or -l options for fixed or ppp-fixed mode
*           2015/06/12  1.9 output patch level in header
*           2016/09/07  1.10 add option -sys
*-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "./rtklib.h"

#define PROGNAME    "rnx2rtkp"          /* program name */
#define MAXFILE     16                  /* max number of input files */

/* help text -----------------------------------------------------------------*/
static const char *help[]={
"",
" usage: rnx2rtkp [option]... file file [...]",
"",
" Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and ccompute ",
" receiver (rover) positions and output position solutions.",
" The first RINEX OBS file shall contain receiver (rover) observations. For the",
" relative mode, the second RINEX OBS file shall contain reference",
" (base station) receiver observations. At least one RINEX NAV/GNAV/HNAV",
" file shall be included in input files. To use SP3 precise ephemeris, specify",
" the path in the files. The extension of the SP3 file shall be .sp3 or .eph.",
" All of the input file paths can include wild-cards (*). To avoid command",
" line deployment of wild-cards, use \"...\" for paths with wild-cards.",
" Command line options are as follows ([]:default). With -k option, the",
" processing options are input from the configuration file. In this case,",
" command line options precede options in the configuration file.",
"",
" -?        print help",
" -k file   input options from configuration file [off]",
" -o file   set output file [stdout]",
" -ts ds ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
" -te de te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
" -ti tint  time interval (sec) [all]",
" -p mode   mode (0:single,1:dgps,2:kinematic,3:static,4:moving-base,",
"                 5:fixed,6:ppp-kinematic,7:ppp-static) [2]",
" -m mask   elevation mask angle (deg) [15]",
" -sys s[,s...] nav system(s) (s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) [G|R]",
" -f freq   number of frequencies for relative mode (1:L1,2:L1+L2,3:L1+L2+L5) [2]",
" -v thres  validation threshold for integer ambiguity (0.0:no AR) [3.0]",
" -b        backward solutions [off]",
" -c        forward/backward combined solutions [off]",
" -i        instantaneous integer ambiguity resolution [off]",
" -h        fix and hold for integer ambiguity resolution [off]",
" -e        output x/y/z-ecef position [latitude/longitude/height]",
" -a        output e/n/u-baseline [latitude/longitude/height]",
" -n        output NMEA-0183 GGA sentence [off]",
" -g        output latitude/longitude in the form of ddd mm ss.ss' [ddd.ddd]",
" -t        output time in the form of yyyy/mm/dd hh:mm:ss.ss [sssss.ss]",
" -u        output time in utc [gpst]",
" -d col    number of decimals in time [3]",
" -s sep    field separator [' ']",
" -r x y z  reference (base) receiver ecef pos (m) [average of single pos]",
"           rover receiver ecef pos (m) for fixed or ppp-fixed mode",
" -l lat lon hgt reference (base) receiver latitude/longitude/height (deg/m)",
"           rover latitude/longitude/height for fixed or ppp-fixed mode",
" -y level  output soltion status (0:off,1:states,2:residuals) [0]",
" -x level  debug trace level (0:off) [0]"
};
/* show message --------------------------------------------------------------*/
extern int showmsg(const char *format, ...)
{
   va_list arg;
    va_start(arg,format); vfprintf(stderr,format,arg); va_end(arg);
    fprintf(stderr,"\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}

/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
    int i;
    for (i=0;i<(int)(sizeof(help)/sizeof(*help));i++) fprintf(stderr,"%s\n",help[i]);
    exit(0);
}
/* rnx2rtkp main -------------------------------------------------------------*/
int main(int argc, char **argv)
{
    prcopt_t prcopt=prcopt_default;//计算选项设置，设置了一个默认值，默认值仅设置了少部分字段，处理方式，对应界面的处理方式
    solopt_t solopt=solopt_default;//结果输出形式，初始化结算结果输出
    filopt_t filopt={""};          //文件路径选择
    gtime_t ts={0},te={0};         //ts开始时间、te结束时间
    double tint=0.0,es[]={2022,5,30,0,0,0},ee[]={2025,5,1,23,59,59},pos[3];//采样率，pos变量xyz
    int i,j,n,ret;//定义一些变量用于后面的循环
    char *infile[MAXFILE],*outfile="",*p; //读入文件，默认16个，可改MAXFILE定义；输出文件；
                                          //指向字符串的指针，用于循环指向各main函数参数
    prcopt.mode  =PMODE_KINEMA; //定位模式默认动态相对定位kinematic
    prcopt.navsys=0;            //卫星系统，先设置为无，在spp.conf中可以查找得到    
    prcopt.refpos=1;            //基准站坐标，先设为由SPP平均结得到
    prcopt.glomodear=1;         //GLONASS AR mode，先设on
    solopt.timef=0;             //输出时间格式，先设为sssss.s
    sprintf(solopt.prog ,"%s ver.%s %s",PROGNAME,VER_RTKLIB,PATCH_LEVEL);   //项目名称
    sprintf(filopt.trace,"%s.trace",PROGNAME);                              
    
    /* load options from configuration file */
    //从配置文件读取选项参数，后续有需要可以精简一下//
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-k")&&i+1<argc) {          //如果有-k和配置文件输入
            resetsysopts();                             //先重置所有配置
            if (!loadopts(argv[++i],sysopts)) return -1;//读取配置文件修改了sysopts,进而修改了prcopt_
            getsysopts(&prcopt,&solopt,&filopt);//opt_t转到potcopt_t，filopt_t,solopt_赋值到三个结构体里面,获取用户定位的参数设置，用prcopt_修改prcopt
        }
    }   
    //fof循环判断main函数参数
    for (i=1,n=0;i<argc;i++) {
        if      (!strcmp(argv[i],"-o")&&i+1<argc) outfile=argv[++i];//读取输出文件格式路径，赋值给outfile
        else if (!strcmp(argv[i],"-ts")&&i+2<argc) {                //读取开始结算时间
            sscanf(argv[++i],"%lf/%lf/%lf",es,es+1,es+2);           //输入函数sscanf从字符串中提取格式化数据
            sscanf(argv[++i],"%lf:%lf:%lf",es+3,es+4,es+5);         //es是数组用于存储时间格式，
            ts=epoch2time(es);                                      //转为gtime_t，
        }
        else if (!strcmp(argv[i],"-te")&&i+2<argc) {                // 读取结束解算时间
            sscanf(argv[++i],"%lf/%lf/%lf",ee,ee+1,ee+2);
            sscanf(argv[++i],"%lf:%lf:%lf",ee+3,ee+4,ee+5);
            te=epoch2time(ee);                                      // 转为gtime_t
        }
        else if (!strcmp(argv[i],"-ti")&&i+1<argc) tint=atof(argv[++i]);        // 读取解算时间间隔频率
        else if (!strcmp(argv[i],"-k")&&i+1<argc) {++i; continue;}              // 有-k，跳过
        else if (!strcmp(argv[i],"-p")&&i+1<argc) prcopt.mode=atoi(argv[++i]);  // 读取解算模式
        else if (!strcmp(argv[i],"-f")&&i+1<argc) prcopt.nf=atoi(argv[++i]);    // 读取用于计算的频率
        else if (!strcmp(argv[i],"-sys")&&i+1<argc) {           // 读取用于计算的导航系统
            for (p=argv[++i];*p;p++) {
                switch (*p) {
                    case 'G': prcopt.navsys|=SYS_GPS;           //有对应导航系统，就把它的码做与运算加上       
                    case 'R': prcopt.navsys|=SYS_GLO;
                    case 'E': prcopt.navsys|=SYS_GAL;
                    case 'J': prcopt.navsys|=SYS_QZS;
                    case 'C': prcopt.navsys|=SYS_CMP;
                    case 'I': prcopt.navsys|=SYS_IRN;
                }
                if (!(p=strchr(p,','))) break;
            }
        }
        else if (!strcmp(argv[i],"-m")&&i+1<argc) prcopt.elmin=atof(argv[++i])*D2R;     //比较字符串，大于正数，相等0
        else if (!strcmp(argv[i],"-v")&&i+1<argc) prcopt.thresar[0]=atof(argv[++i]);    // 设置整周模糊度Ratio值
        else if (!strcmp(argv[i],"-s")&&i+1<argc) strcpy(solopt.sep,argv[++i]);         // 设置文件路径分隔符
        else if (!strcmp(argv[i],"-d")&&i+1<argc) solopt.timeu=atoi(argv[++i]);         // 设置时间小数位数
        else if (!strcmp(argv[i],"-b")) prcopt.soltype=1;       // 后向滤波
        else if (!strcmp(argv[i],"-c")) prcopt.soltype=2;       // 前后向滤波组合
        else if (!strcmp(argv[i],"-i")) prcopt.modear=2;        // 单历元模糊度固定
        else if (!strcmp(argv[i],"-h")) prcopt.modear=3;        // fix and hold 模糊度固定
        else if (!strcmp(argv[i],"-t")) solopt.timef=1;         // 输出时间格式为 yyyy/mm/dd hh:mm:ss.ss
        else if (!strcmp(argv[i],"-u")) solopt.times=TIMES_UTC; // 输出为 UTC 时间
        else if (!strcmp(argv[i],"-e")) solopt.posf=SOLF_XYZ;   // 输出 XYZ-ecef 坐标
        else if (!strcmp(argv[i],"-a")) solopt.posf=SOLF_ENU;   // 输出 ENU-baseline
        else if (!strcmp(argv[i],"-n")) solopt.posf=SOLF_NMEA;  // 输出 NMEA-0183 GGA
        else if (!strcmp(argv[i],"-g")) solopt.degf=1;          // 输出经纬度格式为 ddd mm ss.ss
        else if (!strcmp(argv[i],"-r")&&i+3<argc) {             // 基站位置E CEF-XYZ (m)
            prcopt.refpos=prcopt.rovpos=0;                      // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) prcopt.rb[j]=atof(argv[++i]);     // 循环存入基准站坐标
            matcpy(prcopt.ru,prcopt.rb,3,1);                    //将 prcopt.rb 数组中的数据复制到 prcopt.ru 数组中，复制的元素个数为3，进行的是1维数组的复制。
        }
        else if (!strcmp(argv[i],"-l")&&i+3<argc) {             // 循环存入基站位置基站位置LLH (deg/m)
            prcopt.refpos=prcopt.rovpos=0;                      // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) pos[j]=atof(argv[++i]);
            for (j=0;j<2;j++) pos[j]*=D2R;                      // 角度转弧度
            pos2ecef(pos,prcopt.rb);                             // LLH 转 XYZ
            matcpy(prcopt.ru,prcopt.rb,3,1);
        }
        else if (!strcmp(argv[i],"-y")&&i+1<argc) solopt.sstat=atoi(argv[++i]); //输出结果信息
        else if (!strcmp(argv[i],"-x")&&i+1<argc) solopt.trace=atoi(argv[++i]); //输出debug trace等级
        else if (*argv[i]=='-') printhelp();                                    //输入-，打印帮助
        else if (n<MAXFILE) infile[n++]=argv[i];                                //循环判断完一遍参数之后，认为参数是文件路径，用infile数组接收
    }//进一步的赋值，比较繁琐




     /*2.0版本用gps进行测试的ppp*/
    //n = 3;
    //infile[0] = "D:\\Desktop\\demo04\\test01\\chan2700.20o";
    //infile[1] = "D:\\Desktop\\demo \\test01\\brdc2700.20n";
    //infile[2] = "D:\\Desktop\\demo04\\test01\\igs21246.sp3";
    ////infile[3] = "D:\\Desktop\\demo04\\test01\\WUM0MGXFIN_20192740000_01D_30S_CLK.CLK";
    //outfile = "D:\\Desktop\\demo04\\test01\\chan_source_cmpbds.pos";
    //3.04版本用混合系统包含北斗的进行测试
    n = 4;
    infile[0] = "D:\\Desktop\\rtk—learn\\rtklib\\data04_text\\202501\\ABPO\\ABPO00MDG_R_20250010000_01D_30S_MO.25o";
    infile[1] = "D:\\Desktop\\rtk—learn\\rtklib\\data04_text\\202501\\BRDC00IGS_R_20250010000_01D_MN.rnx";
    infile[2] = "D:\\Desktop\\rtk—learn\\rtklib\\data04_text\\202501\\COD0MGXFIN_20250010000_01D_05M_ORB.SP3";
    infile[3] = "D:\\Desktop\\rtk—learn\\rtklib\\data04_text\\202501\\COD0MGXFIN_20250010000_01D_30S_CLK.CLK";
    outfile = "D:\\Desktop\\rtk—learn\\rtklib\\data04_text\\202501\\ABPO\\text_noarekf_ABPO_b1c_b2a_2025001_gps_bds.pos";
    //igmas版本的开发
    //用林提供的数据运行发现运行失败，只选中北斗无数据，加了gps发现解算的数据质量不好，中间丢失了大块的时间段
    

    //这里使用rnx代替p文件和视频里面提到的文件格式来解算尝试一下
    // 	 n = 3;
    //infile[0] = "D:\\Desktop\\data\\bshm1500.22o";
    //infile[1] = "D:\\Desktop\\data\\BRDM00DLR_S_20221500000_01D_MN.22p";
    //infile[2] = "D:\\Desktop\\data\\WUM0MGXULA_20221500000_01D_05M_ORB.SP3";
    //infile[3] = "D:\\Desktop\\data\\WUM0MGXULA_20221501000_01D_05M_CLK.CLK";
    //outfile = "D:\\Desktop\\data\\bds_only.pos";


    /*这里进行用北斗来spp的调试看看效果如何*/
     	 //n = 2;
       // infile[0] = "d:\\desktop\\spp_bds\\wuh20320.25o";
       // infile[1] = "d:\\desktop\\spp_bds\\brdm0320.25p";
       // outfile = "d:\\desktop\\spp_bds\\spp_bds_text.pos";

    /*冰菓视频版本进行解算,执行失败，应该是有其他地方还需要更改的*/
    //n = 8;
    //argc = 7;
    //argv[0] = "D:\\Desktop\\rtk—learn\\rtklib\\x64\\Debug\\rnx2rtkp.exe";
    //argv[1] = "-k";
    //argv[2] = "D:\\Desktop\\data2\\bingguo\\bds_only.conf";
    //argv[3] = "D:\\Desktop\\data2\\bingguo\\wuh20320.25o";
    //argv[4] = "D:\\Desktop\\data2\\bingguo\\brdm0320.25p";
    //argv[5] = "D:\\Desktop\\data2\\bingguo\\WUM0MGXFIN_20250320000_01D_05M_ORB.SP3";
    //argv[6] = "D:\\Desktop\\data2\\bingguo\\WUM0MGXFIN_20250320000_01D_30S_CLK.CLK";
    //outfile = "D:\\Desktop\\data2\\bingguo\\bds_bg.pos";
    //以上是错误的，只截取到到视频中的部分代码


    if (!prcopt.navsys) {                                       //如果没设卫星系统，默认为GPS、GLONASS
        prcopt.navsys=SYS_GPS|SYS_GLO;                          
    }
    if (n<=0) {                                                 //如果读入文件数为0,报错，-2退出
        showmsg("error : no input file");
        return -2;
    }
    ret=postpos(ts,te,tint,0.0,&prcopt,&solopt,&filopt,infile,n,outfile,"","");  //后处理定位解算
    /* gtime_t ts       I   processing start time (ts.time==0: no limit)
*        : gtime_t te       I   processing end time   (te.time==0: no limit)
*          double ti        I   processing interval  (s) (0:all)
*          double tu        I   processing unit time (s) (0:all)
*          prcopt_t *popt   I   processing options
*          solopt_t *sopt   I   solution options
*          filopt_t *fopt   I   file options
*          char   **infile  I   input files (see below)
*          int    n         I   number of input files
*          char   *outfile  I   output file ("":stdout, see below)
*          char   *rov      I   rover id list        (separated by " ")
*          char   *base     I   base station id list (separated by " ")*/
    if (!ret) fprintf(stderr,"%40s\r","");
    return ret;
}




//int main() {
//	int i, n, ret;
//	double tint = 0.0;       /* 求解时间间隔(0:默认) */
//	gtime_t ts = { 0 }, te = { 0 }; /* 历元时段始末控制变量 */
//	char* infile[MAXFILE], outfile[MAXSTRPATH] = { '\0' };
//	char resultpath[MAXSTRPATH] = "D:\\Desktop\\demo04"; /* 结果输出路径 */
//	char sep = (char)FILEPATHSEP;
//	prcopt_t prcopt = prcopt_default; /* 默认处理选项设置 */
//	solopt_t solopt = solopt_default; /* 默认求解格式设置 */
//	filopt_t filopt = { /* 参数文件路径设置 */
//		"D:\\Desktop\\demo04\\igs14_2032.atx", /* 卫星天线参数文件 */
//		"D:\\Desktop\\demo04\\igs14_2032.atx", /* 接收机天线参数文件 */
//		"", /* 测站位置文件 */
//		"", /* 扩展大地水准面数据文件 */
//		"D:\\Desktop\\demo04\\CODG3530.18I", /* 电离层数据文件 */
//		"D:\\Desktop\\demo04\\CAS0MGXRAP_20183530000_01D_01D.DCB", /* DCB数据文件 */
//		"", /* 地球自转参数文件 */
//		"", /* 海洋潮汐负荷文件 */
//	};
//	char infile_[MAXFILE][MAXSTRPATH] = {
//		"D:\\Desktop\\demo04\\chan2690.20o",
//		"D:\\Desktop\\demo04\\brdc2690.20n",
//		"D:\\Desktop\\demo04\\igs21245.sp3",
//		"",
//		"",
//		"",
//		"",
//		""
//	};
//	long t1, t2;
//	double eps[] = { 2020,9,25,0,0,0 }, epe[] = { 2020,9,25,23,0,0 }; /* 设置计算的历元时段 */
//	ts = epoch2time(eps); te = epoch2time(epe);
//
//	for (i = 0, n = 0; i < MAXFILE; i++)
//		if (strcmp(infile_[i], "")) infile[n++] = &infile_[i][0];
//
//	sprintf(outfile, "%s%c", resultpath, sep);//设置输出路径
//
//	/* 自定义求解格式 --------------------------------------------------------*/
//	solopt.posf = SOLF_XYZ;   /* 选择输出的坐标格式，经纬度或是XYZ坐标等 */
//	solopt.times = TIMES_UTC; /* 控制输出解的时间系统类型 */
//	solopt.degf = 0;         /* 输出经纬度格式(0:°, 1:°′″) */
//	solopt.outhead = 1;         /* 是否输出头文件(0:否,1:是) */
//	solopt.outopt = 1;         /* 是否输出prcopt变量(0:否,1:是) */
//	solopt.height = 1;         /* 高程(0:椭球高,1:大地高) */
//
//	/* 自定义处理选项设置 ----------------------------------------------------*/
//	prcopt.mode = PMODE_PPP_KINEMA; /* PPP动态处理 */
//	prcopt.modear = 4;     /* 求解模糊度类型 */
//	prcopt.sateph = EPHOPT_PREC;      /* 使用精密星历 */
//	prcopt.ionoopt = IONOOPT_IFLC;     /* 使用双频消电离层组合模型 */
//	prcopt.tropopt = TROPOPT_EST;      /* 使用对流层天顶延迟估计模型 */
//	prcopt.tidecorr = 0; /* 地球潮汐改正选项(0:关闭,1:固体潮,2:固体潮+?+极移) */
//	prcopt.posopt[0] = 0; /* 卫星天线模型 */
//	prcopt.posopt[1] = 0; /* 接收机天线模型 */
//	prcopt.posopt[2] = 0; /* 相位缠绕改正 */
//	prcopt.posopt[3] = 0; /* 排除掩星 */
//	prcopt.posopt[4] = 0; /* 求解接收机坐标出错后的检查选项 */
//	prcopt.navsys = SYS_GPS; /* 处理的导航系统 */
//	sprintf(outfile, "%s%cChan200925.pos", resultpath, sep); /* 输出结果名称 */
//	prcopt.nf = 2;       /* 参与计算的载波频率个数 */
//	prcopt.elmin = 15.0 * D2R;/* 卫星截止高度角 */
//	prcopt.soltype = 0;       /* 求解类型(0:向前滤波,1:向后滤波,2:混合滤波) */
//
//	t1 = clock();
//	ret = postpos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infile, n, outfile, "", "");
//	t2 = clock();
//
//	if (!ret) fprintf(stderr, "%40s\r", "");
//
//	printf("\n * The total time for running the program: %6.3f seconds\n%c", (double)(t2 - t1) / CLOCKS_PER_SEC, '\0');
//	printf("Press any key to exit!\n");
//	getchar();
//	return ret;
//}
