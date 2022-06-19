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
    0.00000000, 0.00000000, 0.00000000,
};

} // namespace
