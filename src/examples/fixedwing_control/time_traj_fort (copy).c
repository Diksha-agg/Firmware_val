/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include <math.h>

#include "interpl.c"
static double hover_t=0;
double Tff=3.1;
/*#include "controller.h"
#include "time_trajj.h"*/

/* Function Definitions */

/*
 * Arguments    : double t
 *                double posd[3]
 *                double veld[3]
 *                double rot_des[3]
 *                double omegad[3]
 *                double controld[2]
 * Return Type  : void
 */
void time_traj_fort(double t, double posd[3], double veld[3], double rot_des[3],
                double omegad[3], double controld[2])
{
double t1;
double time[4];
double timev[4];


/*double x_traj[31]={0,4.70677080721227e-05,0.00189691934737655,0.0135601121984784,0.0525899090105499,0.146522775172909,0.325989048911183,0.615844027271135,1.03139383709421,1.57777995619828,2.25351505715142,3.05377690829394,3.97182774659549,5.00036269451869,6.13243397101778,7.36029788191454,8.67417315128395,10.0651563862995,11.5270309701983,13.0547645254691,14.6440179961696,16.2911196403481,17.9930501761850,19.7475312845503,21.5530704998112,23.4086399203417,25.3128285947843,27.2626790980206,29.2530640922719,31.2770443832101,33.3235093115277};
double xdot_traj[31]={0,0.00288771467137894,0.0458264012967156,0.210059481401571,0.597466714652655,1.28578536113746,2.25573027013543,3.42398497434994,4.68934726401309,5.96568843357508,7.20766048137511,8.39146958152477,9.50439198865153,10.5478429332604,11.5231115747598,12.4113214960329,13.2029257165387,13.9211921497962,14.5868791107897,15.2069121023623,15.7879678371132,16.3363633626997,16.8587425715244,17.3628673655951,17.8556529586130,18.3382816952152,18.8015142370104,19.2250514025239,19.5890615070030,19.8779918491735,20};
double z_traj[31]={0,0.00272184131241736,0.0202566777869677,0.0617790522216146,0.135983750382828,0.245543977351801,0.379084877521577,0.518856905379387,0.647876324007861,0.752164733618458,0.822877268922214,0.856622042048778,0.854613495403962,0.821496554358663,0.763938012759586,0.691239334381379,0.614319420099178,0.543313503774697,0.485298119728982,0.442594729526756,0.412734975644948,0.390075253682676,0.368329138314875,0.343077305745454,0.313383126396841,0.281992571399874,0.254118399055730,0.235262934961376,0.227061039076980,0.224824537773180,0.224455603707497};
double zdot_traj[31]={0,0.0796310080057799,0.273883750090314,0.551811692259141,0.903108930890713,1.21264677906662,1.36245458207365,1.33664707645889,1.15669016016864,0.863115096487914,0.510896344555930,0.149355423175048,-0.180595081633762,-0.454894102603020,-0.652467606434272,-0.747620476101966,-0.736072884472000,-0.637164298990295,-0.490744756370352,-0.346510317777500,-0.245503260946774,-0.207098922629750,-0.224509607568382,-0.269611404830547,-0.304969922008601,-0.298561208468750,-0.235776648601905,-0.128364727814322,-0.0410561099062689,-0.00921168805366140,0};*/
double theta_traj[31]={0,0.0101551207797269,0.0710854542031700,0.192954030669188,0.375832797194803,0.605224133205116,0.818900809974507,0.989173275546518,1.11952109278947,1.21767402275617,1.29059286463759,1.34356025643953,1.38035584508448,1.40825034907472,1.42118435594733,1.42152110718377,1.41705004347460,1.41501427595846,1.42014556931006,1.43378516438558,1.45392176607406,1.47624856924096,1.49588427549156,1.50914501371010,1.51472454789371,1.51386004439888,1.50947232258743,1.50746557507440,1.51663997119309,1.52449537913713,1.52592508891013};
double thetadot_traj[31]={0,0.297137176834750,0.891412595894019,1.48592796905070,2.08181976243817,2.24987292936748,1.88296638736592,1.45269490117683,1.10238781336517,0.823637859824534,0.608530814473202,0.425610632220331,0.315180119267708,0.206537448872557,0.0536500700970863,-0.0333529943596615,-0.0418809221336622,0.00998920611827229,0.0922805981940641,0.170359183800042,0.215188572513995,0.212141517735131,0.164726701358337,0.0918001209912934,0.0191763154522304,-0.0310769107981819,-0.0492004412395056,0.0387208985131189,0.105239591096766,0.0413338481412076,0};
double Tfwd_traj[31]={117.720000000000,136.362476628599,144.773942375014,158.360224164153,172.434558122575,175.721747974966,177.845738936319,178.920212826045,173.653571647080,165.634187333590,156.908724946733,147.313381060503,137.959548865010,129.728326646674,121.869634860915,111.489013536377,103.038679116820,98.2968340102702,93.1468656321258,87.2418011049883,81.0016614472189,75.2166787639569,70.7497125639306,68.0931116867177,66.9815450358867,66.2723443537983,64.2387826330516,59.0726748130785,50.0271769361853,39.4665419737548,10.1669632974651};
double Mfwd_traj[31]={0,11.7719994969057,11.7719990764291,11.7719977774213,11.7719851049365,-5.33922690334558,-9.78793442924429,-8.55651653290386,-7.76760229968630,-7.33004249826013,-7.28927745208881,-8.46699682329688,-7.13424468531291,-11.1709869491047,-11.7719935160053,-11.7719975751866,-11.7719980247269,-11.7719982095150,-11.7719984704890,-11.7719987600249,-11.7719990117666,-11.7719991924732,-11.7719992955964,-11.7719993176311,-11.7719992285013,-11.7719988866128,-11.7719973628719,-8.49405279728121,-11.6617396749766,-11.7719984973046,-10.1021163711460};

double coeffx[120]={0,0,-0.0147216079753913,0.185125296138183,-0.00136805242756360,0.0374122431611924,-0.354465174422847,1.20910325361837,-0.0172445441716103,0.260000233143256,-1.39338829004203,2.82330901173908,-0.0894975522834053,0.923825641511982,-3.42057589517445,4.88030140443201,-0.218207349894711,1.75598822950993,-5.18704644616266,6.10582117710170,-0.406365164143791,2.55379724790692,-6.16076669022796,6.36601085510855,-0.841976975501425,4.01818207541186,-7.48491276731751,6.51997921106251,-1.68866864363845,6.39914143795994,-9.21343022589093,6.59619397089270,-3.63955369706096,11.7198435579304,-13.5217924629409,7.47538399688397,-7.08701993609393,20.3312968977235,-20.0831342306787,8.85999198555280,-12.1724186029634,31.7326022942751,-27.8623763087624,10.3196574064128,-19.3971523107500,46.4950474590861,-37.0617999430443,11.9065661611853,-29.1994168797182,64.8930087263889,-47.6108050393024,13.5896157994564,-41.1421750990868,85.1734093565280,-57.9504792761864,14.9747959593532,-54.6987314994027,105.762996986861,-66.9826226156788,15.8577113586925,-73.4165632286195,133.307626479473,-79.1511371794449,17.2698189764605,-97.1029051921788,166.429813855256,-93.2227854985460,18.9048422756822,-122.526639175408,198.803582273480,-105.364756262746,20.0158197609285,-151.958538913853,234.024565934216,-117.714842978725,21.0488234726010,-185.776124367661,272.221146753159,-130.305385462132,22.0210000775834,-224.324987701245,313.450175574264,-143.127980731080,22.9396549909975,-268.216886908572,358.124076361314,-156.338928467031,23.8359444561220,-317.904204722688,406.384994583035,-169.954205095450,24.7159166473330,-373.129608202083,457.454446473128,-183.599121640608,25.5296593079528,-432.864880332312,509.710612018354,-196.603928598469,26.1934468646302,-495.970109219620,561.593436052628,-208.403473033623,26.6470488066417,-562.731162621726,613.314336750445,-219.158331383369,26.9272921198888,-636.410183627925,668.324206506396,-230.195960681504,27.2090674365660,-716.536380092899,725.834754195268,-241.235455296517,27.4627623473999,-735.969685198914,718.412102918067,-229.789329298719,25.1918743924381};
double coeffz[120]={0,0,-0.00590137367436631,2.52395292086833,0.000879232024115145,-0.0298367760951336,0.324557927947273,1.32338644095141,-0.0107871116122378,0.114686347585218,-0.254616738830504,2.06377982222975,-0.0241916132388925,0.171143690978226,-0.200402878515220,1.75136242095162,0.0619113707673093,-0.620931183180244,2.12026341673164,-0.446236297659723,-0.0269395802517202,-0.396902287147508,2.25158784728704,-0.895431895087386,-0.341680153647970,0.740711621493184,1.03821759097666,-0.577217180220771,-0.739478791203081,1.95579486487784,-0.0405525283463714,-0.357078169473465,-1.36200125735887,3.79039109022427,-1.74624406823785,0.123612738731280,-1.99754072740051,5.48614974544216,-3.18859889374408,0.504011752190909,-2.41995058852019,6.47825732701566,-3.92201562288414,0.667472831894117,-2.56924117548024,6.80244438152304,-4.14578573900799,0.715077430103842,-2.06383277401932,5.67744351234361,-3.31736680613714,0.513577671965557,-1.98270102959891,5.75788211469937,-3.57200555794974,0.625090736669010,-1.44792527909310,5.07903305373612,-3.40008195138418,0.653985503912990,0.0357937298780756,2.73617087203568,-2.22975282074616,0.475677058088859,2.64378067652807,-1.44477671657981,-0.0343934669786271,0.100290009135643,6.62818065556965,-7.74443733233454,3.26437868990963,-0.471133093469344,10.8228769814778,-14.1039930335278,6.46517017757133,-1.00562538861987,13.7762799698666,-18.3201030091875,8.46145883848599,-1.31889412987783,14.1790540940361,-18.6875089633999,8.53410712527382,-1.31365533938699,11.4301342108508,-14.6938154624542,6.60459701791355,-1.00357723445110,6.06919556611604,-7.39283449493771,3.29341983112178,-0.503462986953868,-0.176376751294874,0.783299205331666,-0.269829445042369,0.0135538566635169,-4.85310314975469,6.79161824629734,-2.83406644553182,0.377231909118750,-5.75428682829689,8.19821778593619,-3.51793606921659,0.483457599827553,3.09640303660929,-1.33717920468617,-0.0981260199885610,0.0752113354319711,26.7989435410794,-26.6953857650860,8.94481678568210,-0.999689821851997,18.3188323492405,-17.9376265386224,5.93003230397492,-0.653757373150525,6.14206071475746,-5.68173559509324,1.81830856986766,-0.193956903853148};

t1=t-hover_t;
if (t1>3.1)
    {t1=3.1;}

for(int i=0;i<4;i++)
{time[i]=pow(t1,i);
 timev[i]=i*pow(t1,i-1);
}
int n=4;
int j;

double h=Tff/30;
  if (t1<0.00001)
    {j=1;}
  else
    {j=ceil(t1/h);}

for(int i=0;i<4;i++)
{posd[0] += coeffx[i+(n*(j-1))]*time[i];
 veld[0] += coeffx[i+(n*(j-1))]*timev[i];
 posd[2] += coeffz[i+(n*(j-1))]*time[i];
 veld[2] += coeffz[i+(n*(j-1))]*timev[i];
}
double tl1 = (j-1)*h;
double tl2 = (j)*h;

double time_new=t1;
double theta_lim[2]={theta_traj[j], theta_traj[j+1]};      //linear interpol of theta
double time_lim[2]={tl1,tl2};
rot_des[1]=interp1(time_lim, theta_lim, time_new);
rot_des[0]=0;
rot_des[2]=0;

double thetadot_lim[2]={thetadot_traj[j], thetadot_traj[j+1]};      //linear interpol of thetadot
omegad[1]=interp1(time_lim, thetadot_lim, time_new);

omegad[0]=0.0;
omegad[2]=0.0;

double Tfwd_lim[2]={Tfwd_traj[j], Tfwd_traj[j+1]};      //linear interpol of feed forward thrust
controld[0]=interp1(time_lim, Tfwd_lim, time_new);

double Mfwd_lim[2]={Mfwd_traj[j], Mfwd_traj[j+1]};      //linear interpol of feed forward thrust
controld[1]=interp1(time_lim, Mfwd_lim, time_new);

posd[1]=0;
veld[1]=0;

}
