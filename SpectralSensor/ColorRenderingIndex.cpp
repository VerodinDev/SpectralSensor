#include "ColorRenderingIndex.h"
#include "Spectrum.h"
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std;

ColorRenderingIndex::ColorRenderingIndex()
{
    // init TCS table
    uint16_t rows = VISIBLE_WAVELENGTHS;
    uint8_t cols = MAX_TCS;
    m_TCS.resize(rows);

    for (uint16_t i = 0; i < rows; i++)
    {
        m_TCS[i].resize(cols);
    }
}

void ColorRenderingIndex::loadTCSTable()
{
    readCSV("../data/TCS values 1nm.csv", m_TCS);

    // printf("\nTCS table\n");
    // for (int row = 0; row < 10; row++)
    //{
    //     for (int col = 0; col < MAX_TCS; col++)
    //     {
    //         printf("%f, ", m_TCS[row][col]);
    //     }

    //    printf("\n");
    //}
}

// calculate CRI
//
// Wikipedia
// 1. Using the 2° standard observer, find the chromaticity coordinates of the test source in the CIE 1960 color
// space.[14]
// 2. Determine the correlated color temperature(CCT) of the test source by finding the closest point to the Planckian
// locus on the(u, v) chromaticity diagram.
// 3. If the test source has a CCT < 5000 K, use a black body for reference, otherwise use CIE standard illuminant
// D.Both sources should have the same CCT.
// 4. Ensure that the chromaticity distance(DC) of the test source to the Planckian locus is under 5.4×10-3 in the CIE
// 1960 UCS.This ensures the meaningfulness of the result, as the CRI is only defined for light sources that are
// approximately white.
// 5. Illuminate the first eight standard samples, from the fifteen listed below, alternately using both sources.
// 6. Using the 2° standard observer, find the coordinates of the light reflected by each sample in the CIE 1964 color
// space.
// 7. Chromatically adapt each sample by a von Kries transform.
// 8. For each sample, calculate the Euclidean distance between the pair of coordinates.
// 9. Calculate the special(i.e., particular) CRI
// 10. Find the general CRI(Ra) by calculating the arithmetic mean of the special CRIs.
//
// Notes:
// CCT < 5000K -> use Planckian radiator as the reference illuminant
void ColorRenderingIndex::calculateCRI2(vector<double> &spd, uint8_t Ri[])
{
    // init
    for (uint8_t i = 0; i < MAX_TCS; i++)
    {
        Ri[i] = 0;
    }

    // 1nm or 2nm resolution
    uint8_t stepsize = spd.size() > 201 ? 1 : 2;
    const uint16_t noOfWavelengths(VISIBLE_WAVELENGTHS / stepsize);

    // get SPD XYZ
    Tristimulus spdXYZ;
    Spectrum::spectrumToXYZ(spd, spdXYZ);
    printf("XYZ = %f, %f, %f\n", spdXYZ.X, spdXYZ.Y, spdXYZ.Z);

    double YspdNormal = 100 / spdXYZ.Y;
    //double Yspd = spdXYZ.Y * YspdNormal;

    // get SPD u and v
    double uSpd, vSpd;
    convertToCIE1960(spdXYZ, uSpd, vSpd);
    printf("convertToCIE1960: u = %f, v = %f\n", uSpd, vSpd);

    // setup reference
    prepareReference2(spd);

    //
    double tcsSpdY[MAX_TCS];
    double tcsRefY[MAX_TCS];
    double uTcsSpd[MAX_TCS];
    double vTcsSpd[MAX_TCS];
    double uTcsRef[MAX_TCS];
    double vTcsRef[MAX_TCS];

    printf("\n*** TCS reflexivity ***\n");

    // get u and v for each TCS
    for (uint8_t s = 0; s < MAX_TCS; s++)
    {
        printf("\nTCS%d:\n", s + 1);

        // SPD
        vector<double> spdRefl(noOfWavelengths);
        for (uint16_t k = 0; k < noOfWavelengths; k++)
        {
            spdRefl[k] = spd[k] * m_TCS[k * stepsize][s];
        }

        Tristimulus tcsSpdXYZ;
        Spectrum::spectrumToXYZ(spdRefl, tcsSpdXYZ);
        tcsSpdY[s] = tcsSpdXYZ.Y * YspdNormal;

        convertToCIE1960(tcsSpdXYZ, uTcsSpd[s], vTcsSpd[s]);

        printf("spd XYZ: %f, %f, %f\n", tcsSpdXYZ.X, tcsSpdXYZ.Y, tcsSpdXYZ.Z);
        printf("spd uv : %f, %f\n", uTcsSpd[s], vTcsSpd[s]);

        // reference
        m_reference.refl.resize(noOfWavelengths);
        for (uint16_t k = 0; k < noOfWavelengths; k++)
        {
            m_reference.refl[k] = m_reference.reference[k] * m_TCS[k * stepsize][s];
        }

        Tristimulus tcsRefXYZ;
        Spectrum::spectrumToXYZ(m_reference.refl, tcsRefXYZ);
        tcsRefY[s] = tcsRefXYZ.Y * m_reference.YNormal;

        convertToCIE1960(tcsRefXYZ, uTcsRef[s], vTcsRef[s]);

        printf("ref XYZ: %f, %f, %f\n", tcsRefXYZ.X, tcsRefXYZ.Y, tcsRefXYZ.Z);
        printf("ref uv : %f, %f\n", uTcsRef[s], vTcsRef[s]);
    }

    printf("\n");

    // check chromaticity distance
    double DC = sqrt(pow(m_reference.u - uSpd, 2) + pow(m_reference.v - vSpd, 2));

    if (DC > 0.0054)
    {
        printf("Chromaticity distance too high\n");
        printf("SPD uv = %f %f, ref uv = %f %f\n", uSpd, vSpd, m_reference.u, m_reference.v);

        return;
    }

    // cromatic transform on SPD and reference
    double cSpd, dSpd;
    chromaticTransform(uSpd, vSpd, cSpd, dSpd);
    chromaticTransform(m_reference.u, m_reference.v, m_reference.c, m_reference.d);

    double cTcsSpd[MAX_TCS];
    double dTcsSpd[MAX_TCS];

    //  Von Kries
    double vku[MAX_TCS];
    double vkv[MAX_TCS];

    double Wref[MAX_TCS];
    double Uref[MAX_TCS];
    double Vref[MAX_TCS];

    double Wtest[MAX_TCS];
    double Utest[MAX_TCS];
    double Vtest[MAX_TCS];

    // colour difference
    double Euc[MAX_TCS];

    for (uint8_t s = 0; s < MAX_TCS; s++)
    {
        chromaticTransform(uTcsSpd[s], vTcsSpd[s], cTcsSpd[s], dTcsSpd[s]);

        // von Kries chromatic transform
        double cn = m_reference.c / cSpd * cTcsSpd[s];
        double dn = m_reference.d / dSpd * dTcsSpd[s];
        vku[s] = (10.872 + 0.404 * cn - 4 * dn) / (16.518 + 1.481 * cn - dn);
        vkv[s] = 5.520 / (16.518 + 1.481 * cn - dn);

        // transformation into 1964
        Wref[s] = 25 * pow(tcsRefY[s], 1 / 3) - 17;
        Uref[s] = 13 * Wref[s] * (uTcsRef[s] - m_reference.u);
        Vref[s] = 13 * Wref[s] * (vTcsRef[s] - m_reference.v);

        Wtest[s] = 25 * pow(tcsSpdY[s], 1 / 3) - 17;
        Utest[s] = 13 * Wtest[s] * (vku[s] - m_reference.u);
        Vtest[s] = 13 * Wtest[s] * (vkv[s] - m_reference.v);

        // colour difference
        Euc[s] = sqrt(pow(Wref[s] - Wtest[s], 2) + pow(Uref[s] - Utest[s], 2) + pow(Vref[s] - Vtest[s], 2));

        // CRI value
        Ri[s] = static_cast<uint8_t>(100 - 4.6 * Euc[s]);
    }
}

void ColorRenderingIndex::normalize(vector<double> &values)
{
    double highestValue(0);

    for (uint16_t i = 0; i < values.size(); i++)
    {
        if (values[i] > highestValue)
        {
            highestValue = values[i];
        }
    }

    for (uint16_t i = 0; i < values.size(); i++)
    {
        values[i] /= highestValue;
    }
}

void ColorRenderingIndex::normalizeTCS(Tristimulus XYZ[], double Ynorm)
{
    const double refNorm = 100 / Ynorm;

    for (uint8_t q = 0; q < MAX_TCS; q++)
    {
        XYZ[q].X = XYZ[q].X * refNorm;
        XYZ[q].Y = XYZ[q].Y * refNorm;
        XYZ[q].Z = XYZ[q].Z * refNorm;
    }
}

void ColorRenderingIndex::convertToCIE1960(const Tristimulus &XYZ, double &u, double &v)
{
    // from XYZ
    // https://en.wikipedia.org/wiki/CIE_1960_color_space#Relation_to_CIE_XYZ
    double n = XYZ.X + 15 * XYZ.Y + 3 * XYZ.Z;
    u = 4 * XYZ.X / n;
    v = 6 * XYZ.Y / n;

    // debug E21A mix
    // Osram CC: u = 0.2558 v = 0.5100

    // debug FW3A 2700K
    // Osram CC: u = 0.2666   v = 0.5384
    // this:     u = 0.266605 v = 0.358944

    // printf("convertToCIE1960 (XYZ):\t\t u = %f, v = %f\n", u, v);
}

void ColorRenderingIndex::convertToCIE1960(const Chromaticity &xy, double &u, double &v)
{
    // MacAdam simplified Judd's
    double n = 12 * xy.y - 2 * xy.x + 3;
    u = (4 * xy.x) / n;
    v = (6 * xy.y) / n;
    // printf("convertToCIE1960: u = %f, v = %f\n", u, v);

    // using Judd results in very different values, e.g.
    // convertToCIE1960(MacAdam xy) : u = 0.266605, v = 0.358944
    // convertToCIE1960(Judd xy) xy : u = 0.489688, v = 0.474900
    // convertToCIE1960(XYZ) : u = 0.266605, v = 0.358944

    //// Judd 1
    // double m = xy.y - 0.15735 * xy.x + 0.2424;
    // u = (0.4661 * xy.x + 0.1593 * xy.y) / m;
    // v = (0.6581 * xy.y) / m;

    // printf("convertToCIE1960 (Judd xy) xy:\t u = %f, v = %f\n", u, v);

    //// Judd 2
    // double p = 12 * xy.y - 1.882 * xy.x + 2.9088;
    // u = (5.5932 * xy.x + 1.9116 * xy.y) / p;
    // v = (7.8972 * xy.y) / p;

    // printf("convertToCIE1960 (Judd xy) xy:\t u = %f, v = %f\n", u, v);
}

void ColorRenderingIndex::convertToCIE1976(const Chromaticity &xy, double &u, double &v)
{
    double n = 12 * xy.y - 2 * xy.x + 3;
    u = (4 * xy.x) / n;
    v = (9 * xy.y) / n;
    // printf("convertToCIE1976: u' = %f, v' = %f\n", u, v);
}

void ColorRenderingIndex::chromaticTransform(double u, double v, double &c, double &d)
{
    c = (4 - u - 10 * v) / v;
    d = (1.708 * v - 1.481 * u + 0.404) / v;

    // printf("chromaticTransform c = %f, d = %f\n", c, d);
}

uint16_t ColorRenderingIndex::getCCT(const std::vector<double> &spd)
{
    Tristimulus XYZ;
    Chromaticity xy;

    Spectrum::spectrumToXYZ(spd, XYZ);
    Spectrum::XYZtoXy(XYZ, xy);

    // debug
    uint16_t CCTwp = Spectrum::CIE1931_xy_to_CCT_wikipedia(xy);
    printf("CCT = %dK (Wikipedia)\n", CCTwp);

    return Spectrum::CIE1931_xy_to_CCT(xy);
}

// TODO use D65 as ref for >5000K
void ColorRenderingIndex::prepareReference2(const vector<double> &spd)
{
    printf("*** Preparing reference ***\n");

    uint16_t CCT = getCCT(spd);

    printf("CCT = %dK\n", CCT);

    if (CCT > 5000)
    {
        throw runtime_error("CCT > 5000K not supported yet");
    }

    // 1nm or 2nm resolution
    uint8_t stepsize = spd.size() > 201 ? 1 : 2;

    const uint16_t noOfWavelengths(VISIBLE_WAVELENGTHS / stepsize);

    m_reference.reference.resize(noOfWavelengths);
    m_reference.refl.resize(noOfWavelengths);

    for (int i = 0; i < noOfWavelengths; i++)
    {
        m_reference.reference[i] = getPlanckianLocus(i * stepsize + 380, CCT);
    }

    normalize(m_reference.reference);

    Spectrum::spectrumToXYZ(m_reference.reference, m_reference.XYZ);
    m_reference.YNormal = 100 / m_reference.XYZ.Y;
    //double Yref = m_reference.XYZ.Y * m_reference.YNormal;

    convertToCIE1960(m_reference.XYZ, m_reference.u, m_reference.v);

    // u = 0.26312380320845397
    // v = 0.35174945096434301
    // CCT	x (black body)	y (black body)	u' (black body)	v' (black body)	y (daylight)	y (TM30)
    // 2686	0.461007723031611	0.410854252624604	0.263123416678460	0.527620430070558		0.410854252624604
}

//bool ColorRenderingIndex::verifyChromaticityDistance()
//{
//    double DC = sqrt(pow(m_reference.u - m_spd.u, 2) + pow(m_reference.v - m_spd.v, 2));
//
//    if (DC > 0.0054)
//    {
//        return false;
//    }
//    else
//    {
//        return true;
//    }
//}

double ColorRenderingIndex::getPlanckianLocus(uint16_t wavelength, uint16_t T)
{
    // c1 = 2 * pi * h * c ^ 2;
    // c2 = h * c / k;

    double c1 = 3.74183e-16;                // 1st radiation constant
    double c2 = 1.4388e-2;                  // 2nd radiation constant
    double wavelengthM = wavelength * 1e-9; // wavelength in meter

    return (c1 * pow(wavelengthM, -5.0)) / (exp(c2 / (wavelengthM * T)) - 1.0);
}

// TODO merge with one in spectrumdata. FileOps.cpp?
void ColorRenderingIndex::readCSV(const string &filename, TCSTable &table)
{
    ifstream csvFile;

    csvFile.open(filename, ifstream::in);
    if (!csvFile.is_open())
    {
        throw runtime_error("Error opening data file " + filename);
    }

    string line;
    uint16_t row(0);

    // skip header
    getline(csvFile, line);

    // read values
    // 380 0.2190 0.0700 0.0650 0.0740 0.2950 0.1500 0.3780 0.1040 0.0660 0.0500 0.1110 0.1200 0.1040 0.0360
    while (getline(csvFile, line))
    {
        replace(line.begin(), line.end(), ';', ' ');

        stringstream ss(line);
        float value(0);
        uint8_t column = 0;

        // skip wavelength column
        ss >> value;

        // note that there is NO faulty input data check whatsoever!!!
        while (ss >> value)
        {
            table[row][column] = value;
            column++;
        }

        row++;
    }

    csvFile.close();
}
