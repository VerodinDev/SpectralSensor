#pragma once

namespace
{

// Candle SPD, 380-780, 2nm steps
// Thank you Charles!
//
// clang-format off
// 
// photometry:
//lumens = 0,158   W (360 - 830 nm) = 1,01 mW   CCT = 2019 (495 Mired)   LER = 156
//x = 0,5250   y = 0,4141   u' = 0,3035   v' = 0,5386   Duv = 0,0002   Du'v' = 0,0002   
//CCT (F,10) = 2018   lm (F,10) = 0,165   x (F,10) = 0,5310   y (F,10) = 0,4115   u' (F,10) = 0,3089   v' (F,10) = 0,5386   Duv (F,10) = 0,0001   Du'v' (F,10) = 0,0002   
//dom WL = 588,7nm   peak cent = 648,8nm   FWHM = 89,8nm   purity = 81,9%   FCI = 126,1   FSI = 11,99   FSCI = 38,84   WBMR = 0,327   Scopotic lm = 0,150   S/P = 0,95   Photons/s (360-830nm) = 3,300e+15   
//TM-30-18: Rf = 100   Rg = 100   Rf,skin = 100   
//CRI=100  R1=100  R2=100  R3=99  R4=100  R5=100  R6=100  R7=100  R8=99  R9=99  R10=99  R11=100  R12=99  R13=100  R14=99  R15=100  
//CQS=99  Q1=99  Q2=99  Q3=99  Q4=99  Q5=99  Q6=99  Q7=99  Q8=100  Q9=100  Q10=100  Q11=100  Q12=100  Q13=100  Q14=100  Q15=100  
//TM-30-18: Rf=100  Rfh,1=100  Rfh,2=100  Rfh,3=99  Rfh,4=100  Rfh,5=100  Rfh,6=99  Rfh,7=100  Rfh,8=99  Rfh,9=100  Rfh,10=100  Rfh,11=100  Rfh,12=100  Rfh,13=99  Rfh,14=99  Rfh,15=100  Rfh,16=100  
//Rhs,h1=0.00  Rhs,h2=0.00  Rhs,h3=0.00  Rhs,h4=0.00  Rhs,h5=0.00  Rhs,h6=0.00  Rhs,h7=0.00  Rhs,h8=0.00  Rhs,h9=0.00  Rhs,h10=0.00  Rhs,h11=0.00  Rhs,h12=0.00  Rhs,h13=-0.01  Rhs,h14=0.00  Rhs,h15=0.00  Rhs,h16=0.00  
//Rcs,h1=0%  Rcs,h2=0%  Rcs,h3=0%  Rcs,h4=0%  Rcs,h5=0%  Rcs,h6=0%  Rcs,h7=0%  Rcs,h8=0%  Rcs,h9=0%  Rcs,h10=0%  Rcs,h11=0%  Rcs,h12=0%  Rcs,h13=0%  Rcs,h14=0%  Rcs,h15=0%  Rcs,h16=0%  
//
// clang-format on
//
// wl.com
// xy : (0.525, 0.4141) | CCT : 2, 027 K | Duv : 0.0003 | CRI(Ra) : 100 | R9 : 100
// R1=100
// R2 = 100
// R3 = 99
// R4 = 100
// R5 = 100
// R6 = 100
// R7 = 100
// R8 = 100
// R9 = 100
// R10 = 99
// R11 = 100
// R12 = 99
// R13 = 100
// R14 = 99
// R15 = 100
//
// double candleSpd[] = {
std::vector<double> candleSpd = {
    0.00005835, 0.00005835, 0.00005835, 0.00005835, 0.00005835, 0.00005835, 0.00005835, 0.00005835, 0.00005835,
    0.00005835, 0.00005836, 0.00006948, 0.00007319, 0.00008403, 0.00008592, 0.00009159, 0.00009374, 0.00009445,
    0.00010000, 0.00010300, 0.00011200, 0.00012450, 0.00012870, 0.00013780, 0.00014060, 0.00014890, 0.00016320,
    0.00016810, 0.00018630, 0.00018930, 0.00019840, 0.00021260, 0.00021730, 0.00023870, 0.00024350, 0.00025800,
    0.00027080, 0.00027510, 0.00029280, 0.00029850, 0.00031570, 0.00033630, 0.00034320, 0.00037220, 0.00037970,
    0.00040220, 0.00042530, 0.00043300, 0.00046900, 0.00047820, 0.00050580, 0.00053110, 0.00053960, 0.00057330,
    0.00058330, 0.00061340, 0.00064900, 0.00066080, 0.00070270, 0.00071230, 0.00074110, 0.00077360, 0.00078440,
    0.00083300, 0.00084650, 0.00088680, 0.00092300, 0.00093510, 0.00098260, 0.00099720, 0.00104100, 0.00109200,
    0.00110900, 0.00118200, 0.00119800, 0.00124700, 0.00129200, 0.00130700, 0.00137400, 0.00139600, 0.00146100,
    0.00153200, 0.00155600, 0.00164000, 0.00165700, 0.00171000, 0.00176000, 0.00177700, 0.00186200, 0.00188800,
    0.00196500, 0.00204200, 0.00206800, 0.00215700, 0.00217600, 0.00223200, 0.00228800, 0.00230700, 0.00240900,
    0.00244100, 0.00253600, 0.00263600, 0.00267000, 0.00278900, 0.00281100, 0.00287900, 0.00294800, 0.00297100,
    0.00307700, 0.00311000, 0.00321000, 0.00332100, 0.00335800, 0.00349800, 0.00352800, 0.00361800, 0.00370800,
    0.00373800, 0.00386700, 0.00390500, 0.00401600, 0.00413300, 0.00417200, 0.00432900, 0.00436700, 0.00448000,
    0.00459300, 0.00463100, 0.00476700, 0.00480000, 0.00489900, 0.00500800, 0.00504500, 0.00521300, 0.00525900,
    0.00539600, 0.00552900, 0.00557400, 0.00573800, 0.00577300, 0.00587900, 0.00599100, 0.00602900, 0.00619400,
    0.00623400, 0.00635400, 0.00648100, 0.00652300, 0.00671200, 0.00676200, 0.00691200, 0.00706200, 0.00711200,
    0.00730000, 0.00733800, 0.00745000, 0.00756200, 0.00759900, 0.00778100, 0.00783000, 0.00797900, 0.00813900,
    0.00819200, 0.00843200, 0.00849400, 0.00867900, 0.00886300, 0.00892400, 0.00912600, 0.00916900, 0.00930000,
    0.00945400, 0.00950600, 0.00975400, 0.00731600, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000};

// White 2700K (Osram ColorCalc)
std::vector<double> white_2700_osram = {
    0.00000586, 0.00000518, 0.00000591, 0.00000592, 0.00000523, 0.00000555, 0.00000484, 0.00000553, 0.00000547,
    0.00000562, 0.00000546, 0.00000625, 0.00000638, 0.00000707, 0.00000742, 0.00000827, 0.00000839, 0.00000982,
    0.00001035, 0.00001152, 0.00001325, 0.00001444, 0.00001628, 0.00001844, 0.00002120, 0.00002478, 0.00002938,
    0.00003552, 0.00004264, 0.00005337, 0.00006570, 0.00008290, 0.00010350, 0.00013160, 0.00016730, 0.00020840,
    0.00025850, 0.00031420, 0.00037590, 0.00043220, 0.00048100, 0.00051690, 0.00053260, 0.00052990, 0.00051210,
    0.00048710, 0.00046150, 0.00043850, 0.00042020, 0.00040370, 0.00038720, 0.00036710, 0.00034620, 0.00032460,
    0.00030700, 0.00029600, 0.00028490, 0.00028240, 0.00028080, 0.00028230, 0.00028630, 0.00029270, 0.00030060,
    0.00031240, 0.00032680, 0.00033850, 0.00035500, 0.00036950, 0.00038620, 0.00040270, 0.00042120, 0.00043790,
    0.00045750, 0.00047650, 0.00049370, 0.00051190, 0.00053040, 0.00054720, 0.00056830, 0.00058860, 0.00060820,
    0.00062780, 0.00064870, 0.00066710, 0.00068810, 0.00071110, 0.00073150, 0.00075250, 0.00077650, 0.00079880,
    0.00082000, 0.00084510, 0.00086670, 0.00089170, 0.00091600, 0.00093850, 0.00095930, 0.00098250, 0.00100300,
    0.00102900, 0.00105300, 0.00107100, 0.00109200, 0.00111200, 0.00112800, 0.00114500, 0.00116000, 0.00117300,
    0.00118400, 0.00119400, 0.00120500, 0.00121100, 0.00121700, 0.00122300, 0.00122000, 0.00121700, 0.00121400,
    0.00121200, 0.00120600, 0.00119400, 0.00118500, 0.00117300, 0.00116000, 0.00114100, 0.00112400, 0.00110800,
    0.00108800, 0.00107500, 0.00105100, 0.00102700, 0.00100600, 0.00098040, 0.00095670, 0.00093340, 0.00090680,
    0.00088470, 0.00085790, 0.00083020, 0.00080620, 0.00078100, 0.00075500, 0.00073660, 0.00071030, 0.00068680,
    0.00065910, 0.00063550, 0.00061690, 0.00059250, 0.00057120, 0.00055040, 0.00052880, 0.00050760, 0.00048720,
    0.00047000, 0.00045130, 0.00043090, 0.00041390, 0.00039420, 0.00037990, 0.00036280, 0.00034850, 0.00033210,
    0.00031710, 0.00030370, 0.00028990, 0.00027680, 0.00026470, 0.00025170, 0.00024200, 0.00022840, 0.00021840,
    0.00020800, 0.00019840, 0.00018820, 0.00018030, 0.00017130, 0.00016410, 0.00015650, 0.00014930, 0.00014150,
    0.00013470, 0.00012960, 0.00012220, 0.00011630, 0.00011200, 0.00010570, 0.00010130, 0.00009744, 0.00009264,
    0.00008817, 0.00008375, 0.00007976, 0.00007637, 0.00007194, 0.00006979, 0.00006714, 0.00006421, 0.00006050,
    0.00005782, 0.00005549, 0.00000000};

std::vector<double> e21mix_mule = {
    -0.00247979, -0.00250449, -0.00252992, -0.00255554, -0.00258133, -0.00260751, -0.00263383,  -0.00266058,
    -0.00268727, -0.00271449, -0.00274189, -0.00276955, -0.00279747, -0.00282559, -0.00285394,  -0.00288198,
    -0.00290932, -0.00293598, -0.00296047, -0.00298256, -0.00300043, -0.00301345, -0.00301918,  -0.00301644,
    -0.00300287, -0.00297701, -0.00293686, -0.00288033, -0.00280642, -0.00271324, -0.00259998,  -0.00246577,
    -0.00230971, -0.00213229, -0.00193292, -0.00171207, -0.00147045, -0.00120905, -0.000928513, -0.00063049,
    -0.00031625, 1.26808e-05, 0.000354564, 0.00070783,  0.00106996,  0.00143959,  0.0018145,    0.00219254,
    0.00257178,  0.00294992,  0.00332493,  0.00369477,  0.00405738,  0.00441094,  0.00475274,   0.00508247,
    0.0053961,   0.00569428,  0.00597336,  0.00623272,  0.00647102,  0.00668683,  0.0068784,    0.00704582,
    0.007187,    0.00730331,  0.00739242,  0.00745442,  0.00749067,  0.00750068,  0.00748477,   0.00744419,
    0.00738144,  0.00729562,  0.00718963,  0.00706579,  0.00692548,  0.00677034,  0.0066033,    0.00642632,
    0.0062415,   0.00605108,  0.00585744,  0.00566226,  0.00546772,  0.00527542,  0.00508769,   0.0049055,
    0.00473066,  0.00456408,  0.00440738,  0.00426127,  0.00412676,  0.00400437,  0.00389498,   0.00379881,
    0.00371608,  0.00364725,  0.00359244,  0.00355155,  0.00352469,  0.00351151,  0.00351226,   0.00352634,
    0.00355338,  0.00359304,  0.00364506,  0.00370875,  0.0037838,   0.00386926,  0.00396485,   0.00406985,
    0.00418357,  0.00430523,  0.00443436,  0.00456976,  0.00471119,  0.00485775,  0.00500884,   0.00516334,
    0.00532083,  0.00548026,  0.00564129,  0.00580287,  0.00596454,  0.00612559,  0.00628515,   0.0064428,
    0.00659799,  0.00674999,  0.00689908,  0.00704388,  0.00718469,  0.00732088,  0.00745256,   0.00757928,
    0.00770085,  0.00781753,  0.00792882,  0.00803503,  0.00813629,  0.00823235,  0.00832362,   0.00840981,
    0.00849131,  0.0085683,   0.00864046,  0.0087083,   0.00877199,  0.00883114,  0.00888583,   0.00893661,
    0.00898344,  0.00902643,  0.00906531,  0.00910025,  0.00913167,  0.00915978,  0.0091845,    0.00920521,
    0.00922299,  0.00923792,  0.00924973,  0.00925885,  0.0092651,   0.00926919,  0.00927094,   0.00927064,
    0.00926855,  0.00926522,  0.0092602,   0.00925396,  0.00924698,  0.00923953,  0.00923144,   0.00922305,
    0.00921546,  0.00920762,  0.00920059,  0.00919466,  0.00918983,  0.00918643,  0.00918483,   0.00918539,
    0.00918807,  0.00919364,  0.00920236,  0.00921372,  0.00922917,  0.00924846,  0.00927164,   0.00929914,
    0.00933138,  0.00936885,  0.00941167,  0.00945988,  0.00951402,  0.00957446,  0.0096405,    0.00971411,
    0.00979415,  0.0098815,   0.00997531,  0.0100772,   0.0101865,   0.0103041,   0.0104286,    0.0105613,
    0.0107016,   0.0108496,   0.0110055,   0.0111689,   0.0113398,   0.0115177,   0.0117021,    0.0118929,
    0.0120892,   0.0122906,   0.012497,    0.0127072,   0.0129208,   0.0131363,   0.0133541,    0.0135731,
    0.0137924,   0.0140111,   0.0142287,   0.0144442,   0.0146575,   0.0148671,   0.0150727,    0.0152731,
    0.0154688,   0.0156577,   0.0158398,   0.0160144,   0.0161805,   0.0163388,   0.016487,     0.0166258,
    0.0167537,   0.0168717,   0.0169781,   0.0170725,   0.0171556,   0.0172265,   0.0172848,    0.0173301,
    0.0173633,   0.0173829,   0.0173903,   0.0173839,   0.0173652,   0.0173339,   0.0172892,    0.0172327,
    0.0171638,   0.0170827,   0.0169909,   0.0168872,   0.0167734,   0.0166482,   0.0165136,    0.0163693,
    0.016216,    0.0160541,   0.0158837,   0.015706,    0.0155208,   0.0153291,   0.0151309,    0.0149277,
    0.0147185,   0.0145049,   0.0142872,   0.014066,    0.0138418,   0.0136155,   0.0133867,    0.013157,
    0.0129264,   0.0126958,   0.0124657,   0.0122372,   0.0120107,   0.0117863,   0.0115651,    0.0113473,
    0.0111334,   0.0109239,   0.0107192,   0.0105192,   0.0103247,   0.0101359,   0.00995241,   0.00977468,
    0.00960317,  0.00943765,  0.00927831,  0.00912535,  0.00897849,  0.00883842,  0.0087044,    0.00857699,
    0.00845586,  0.00834182,  0.00823368,  0.00813279,  0.00803716,  0.00794872,  0.00786556,   0.00778894,
    0.00771792,  0.0076522,   0.00759273,  0.00753855,  0.00748923,  0.00744424,  0.00740507,   0.00736964,
    0.00733911,  0.00731214,  0.00728919,  0.00726976,  0.00725366,  0.00724065,  0.00723055,   0.00722328,
    0.00721846,  0.00721632,  0.00721569,  0.00721741,  0.00722112,  0.00722539,  0.0072303,    0.0072375,
    0.00724588,  0.00725537,  0.00726488,  0.00727594,  0.00728699,  0.00729888,  0.00731021,   0.00732312,
    0.0073352,   0.00734778,  0.00736154,  0.00737506,  0.00738838,  0.00740116,  0.00741502,   0.00742837,
    0.00744121,  0.00745565,  0.00746862,  0.00748181,  0.00749509,  0.00750779,  0.00752079,   0.00753362,
    0.00754617,  0.00755787,  0.00756943,  0.00758114,  0.00759178,  0.00760216,  0.00761217,   0.0076217,
    0.00763046,  0.00763936,  0.00764649,  0.0076541,   0.00766036,  0.00766716,  0.00767185,   0.00767655,
    0.00768053,  0.00768361,  0.00768718,  0.00768848,  0.00768981,  0.00769007,  0.00769104,   0.00768988,
    0.00768947,  0.00768753,  0.00768466,  0.00768153,  0.00767769,  0.00767342,  0.00766813,   0.00766284,
    0.00765625,  0.00764817,  0.00764125,  0.00763214,  0.00762386,  0.00761449,  0.0076038,    0.00759161,
    0.00758036,
};

//FW3A 2700K: 752 lm 
//
// clang-format off
//lumens = 752   W (360 - 830 nm) = 2909 mW   CCT = 2581 (387 Mired)   LER = 259
//x = 0,481328   y = 0,432033   u' = 0,266599   v' = 0,538416   Duv = 0,006048   Du'v' = 0,008919   
//CCT (F,10) = 2534   lm (F,10) = 792   x (F,10) = 0,490702   y (F,10) = 0,428482   u' (F,10) = 0,274121   v' (F,10) = 0,538566   Duv (F,10) = 0,005565   Du'v' (F,10) = 0,008226   
//dom WL = 583,1nm   peak cent = 624,3nm   FWHM = 231,9nm   purity = 74,2%   FCI = 118,5   FSI = 9,71   FSCI = 50,49   WBMR = 0,428   Scopotic lm = 902   S/P = 1,20   Photons/s (360-830nm) = 9,142e+18   
//TM-30-18: Rf = 90   Rg = 94   Rf,skin = 92   
//CRI=95  R1=97  R2=96  R3=94  R4=97  R5=96  R6=99  R7=95  R8=85  R9=63  R10=90  R11=97  R12=84  R13=97  R14=95  R15=90  
//CQS=87  Q1=88  Q2=89  Q3=79  Q4=74  Q5=80  Q6=84  Q7=91  Q8=96  Q9=98  Q10=99  Q11=99  Q12=95  Q13=87  Q14=94  Q15=94  
//TM-30-18: Rf=90  Rfh,1=91  Rfh,2=94  Rfh,3=90  Rfh,4=86  Rfh,5=90  Rfh,6=88  Rfh,7=87  Rfh,8=90  Rfh,9=91  Rfh,10=93  Rfh,11=93  Rfh,12=91  Rfh,13=89  Rfh,14=90  Rfh,15=89  Rfh,16=87  
//Rhs,h1=0.01  Rhs,h2=-0.02  Rhs,h3=-0.03  Rhs,h4=-0.04  Rhs,h5=0.02  Rhs,h6=0.07  Rhs,h7=0.06  Rhs,h8=0.07  Rhs,h9=0.05  Rhs,h10=0.04  Rhs,h11=0.04  Rhs,h12=-0.03  Rhs,h13=-0.09  Rhs,h14=-0.08  Rhs,h15=0.01  Rhs,h16=-0.07  
//Rcs,h1=-4%  Rcs,h2=-3%  Rcs,h3=-4%  Rcs,h4=-9%  Rcs,h5=-8%  Rcs,h6=-5%  Rcs,h7=-5%  Rcs,h8=0%  Rcs,h9=0%  Rcs,h10=0%  Rcs,h11=3%  Rcs,h12=3%  Rcs,h13=-2%  Rcs,h14=-1%  Rcs,h15=-5%  Rcs,h16=-4%  
// clang-format on
std::vector<double> FW3A_2700 = {
    0,           0,           0,           0,          0,          0,          0,          0,          0,
    0,           0,           0,           0,          0,          0,          0,          0,          0,
    0,           0,           0,           0,          0,          0,          0,          0,          0,
    0,           0,           0,           0,          0,          0,          0,          0,          0,
    0,           0,           0,           0,          0,          0,          0,          0,          0,
    0,           0,           0,           0,          0,          0,          0,          0,          0.000199208,
    0.000414522, 0.000625038, 0.000828597, 0.00102562, 0.0012142,  0.00139396, 0.00156423, 0.00172431, 0.0018732,
    0.00201103,  0.0021366,   0.00225073,  0.00235211, 0.0024408,  0.00251743, 0.0025817,  0.0026338,  0.00267416,
    0.00270421,  0.00272319,  0.00273269,  0.00273379, 0.00272705, 0.00271321, 0.00269375, 0.00266964, 0.00264169,
    0.00261114,  0.0025791,   0.0025463,   0.00251383, 0.00248232, 0.00245317, 0.0024266,  0.00240361, 0.00238449,
    0.00237019,  0.002361,    0.00235743,  0.00235966, 0.00236827, 0.00238328, 0.00240471, 0.00243294, 0.00246802,
    0.00250982,  0.00255847,  0.00261374,  0.00267594, 0.00274463, 0.00281963, 0.00290081, 0.00298806, 0.00308107,
    0.0031797,   0.00328341,  0.00339215,  0.00350556, 0.00362329, 0.00374493, 0.00387034, 0.00399872, 0.00413021,
    0.0042643,   0.00440073,  0.00453879,  0.00467844, 0.004819,   0.00496054, 0.00510226, 0.00524418, 0.00538589,
    0.00552682,  0.00566687,  0.00580578,  0.00594311, 0.00607935, 0.00621335, 0.00634559, 0.00647561, 0.00660365,
    0.00672937,  0.00685262,  0.00697376,  0.00709228, 0.00720848, 0.00732242, 0.00743384, 0.00754304, 0.00764966,
    0.00775397,  0.00785597,  0.00795527,  0.00805217, 0.00814674, 0.00823838, 0.00832704, 0.00841313, 0.0084963,
    0.00857667,  0.00865375,  0.00872753,  0.00879833, 0.00886622, 0.00893096, 0.00899185, 0.00904978, 0.00910487,
    0.00915661,  0.00920545,  0.00925108,  0.00929419, 0.00933453, 0.00937233, 0.00940783, 0.00944156, 0.009473,
    0.00950261,  0.00953093,  0.00955817,  0.00958414, 0.00960924, 0.00963459, 0.00965909, 0.00968386, 0.00970923,
    0.0097352,   0.00976214,  0.00979046,  0.00982055, 0.00985242, 0.00988687, 0.00992422, 0.00996395, 0.0100077,
    0.010055,    0.0101062,   0.0101616,   0.0102218,  0.0102873,  0.0103582,  0.0104347,  0.0105172,  0.0106063,
    0.0107011,   0.0108039,   0.0109133,   0.0110304,  0.0111543,  0.0112866,  0.0114268,  0.0115755,  0.0117316,
    0.0118961,   0.0120686,   0.012249,    0.0124376,  0.0126339,  0.0128377,  0.0130486,  0.0132658,  0.0134892,
    0.0137177,   0.0139508,   0.0141881,   0.0144284,  0.0146711,  0.0149143,  0.0151585,  0.0154021,  0.0156443,
    0.0158838,   0.0161201,   0.0163518,   0.0165787,  0.0167991,  0.0170126,  0.0172175,  0.0174145,  0.0176011,
    0.0177773,   0.017942,    0.0180943,   0.0182346,  0.0183604,  0.0184725,  0.0185692,  0.0186513,  0.0187173,
    0.0187663,   0.0187992,   0.0188151,   0.0188134,  0.0187937,  0.0187569,  0.0187014,  0.0186288,  0.0185373,
    0.0184284,   0.018302,    0.0181573,   0.0179959,  0.0178175,  0.0176222,  0.0174121,  0.0171857,  0.0169454,
    0.0166899,   0.0164217,   0.0161406,   0.0158477,  0.0155435,  0.0152285,  0.0149044,  0.014571,   0.0142297,
    0.0138809,   0.0135265,   0.0131657,   0.0128005,  0.0124315,  0.0120596,  0.0116859,  0.0113114,  0.0109364,
    0.0105627,   0.0101908,   0.00982183,  0.00945697, 0.00909764, 0.00874453, 0.00839824, 0.00806032, 0.00773122,
    0.00741171,  0.00710253,  0.00680434,  0.00651715, 0.00624183, 0.00597873, 0.00572758, 0.00548895, 0.00526324,
    0.00505033,  0.00485039,  0.00466363,  0.00448987, 0.00432966, 0.00418239, 0.00404846, 0.00392752, 0.00382025,
    0.00372559,  0.00364458,  0.00357538,  0.00351948, 0.00347511, 0.00344325, 0.00342281, 0.00341349, 0.00341567,
    0.00342854,  0.00345135,  0.00348345,  0.00352592, 0.00357648, 0.00363603, 0.00370322, 0.00377792, 0.00385948,
    0.00394756,  0.00404147,  0.00414087,  0.00424526, 0.00435408, 0.00446728, 0.00458336, 0.0047029,  0.00482534,
    0.00494896,  0.00507385,  0.00520107,  0.00532947, 0.00545887, 0.00558794, 0.00571791, 0.00584741, 0.00597682,
    0.0061048,   0.00623332,  0.00635997,  0.00648577, 0.00661152, 0.0067357,  0.00685824, 0.00697878, 0.00709886,
    0.00721692,  0.00733286,  0.00744882,  0.00756166, 0.00767307, 0.0077829,  0.00789047, 0.00799666, 0.00810099,
    0.00820336,  0.00830323,  0.00840124,  0.00849779, 0.00859159, 0.00868351, 0.00877337, 0.0088612,  0.00894667,
    0.00903065,  0.00911133,  0.0091909,   0.00926756, 0.00934331, 0.00941537, 0.00948595, 0.00955436, 0.00962039,
    0.00968544,  0.00974676,  0.0098067,   0.00986412, 0.00992081, 0.00997404, 0.0100266,  0.0100763,  0.0101236,
    0.0101693,   0.010213,    0.0102548,   0.0102942,  0.0103324,  0.0103679,  0.0104006,  0.010433,   0.010462,
    0.0104905,   0.0105165,   0.01054,     0.0105606,  0.0105808,
};

} // namespace
