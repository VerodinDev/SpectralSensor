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
// TODO use D65 as ref for >5000K
void ColorRenderingIndex::calculateCRI(vector<double> &spd, uint8_t Ri[])
{
    const uint16_t noOfWavelengths(VISIBLE_WAVELENGTHS / 2);
    uint16_t CCT = getCCT(spd);

    printf("CCT = %dK\n", CCT);

    prepareReference(CCT);
    prepareTestSPD(spd);

    if (!verifyChromaticityDistance())
    {
        // TODO handle better
        printf("Chromaticity distance too high");
        return;
    }

    printf("*** Calculating CRI ***\n");

    // u and v coordinates for each TCS in 1960
    double urefTCS[MAX_TCS];
    double vrefTCS[MAX_TCS];
    double utestTCS[MAX_TCS];
    double vtestTCS[MAX_TCS];

    // c and d constants for TCS samples
    double crefTCS[MAX_TCS];
    double drefTCS[MAX_TCS];
    double ctestTCS[MAX_TCS];
    double dtestTCS[MAX_TCS];

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

    // for each TCS
    for (uint8_t s = 0; s < MAX_TCS; s++)
    {
        // find coordinates of the light reflected in CIE 1964 color space
        // convert chromaticities to the CIE 1960
        convertToCIE1960(m_reference.TCSXYZ[s], urefTCS[s], vrefTCS[s]);
        convertToCIE1960(m_spd.TCSXYZ[s], utestTCS[s], vtestTCS[s]);

        chromaticTransform(urefTCS[s], vrefTCS[s], crefTCS[s], drefTCS[s]);
        chromaticTransform(utestTCS[s], vtestTCS[s], ctestTCS[s], dtestTCS[s]);

        // find corresponding color (uc,i, vc,i)
        // von Kries chromatic transform
        vku[s] =
            (10.872 + 0.404 * (m_reference.c / m_spd.c) * ctestTCS[s] - 4 * (m_reference.d / m_spd.d) * dtestTCS[s]) /
            (16.518 + 1.481 * (m_reference.c / m_spd.c) * ctestTCS[s] - (m_reference.d / m_spd.d) * dtestTCS[s]);
        vkv[s] =
            5.52 / (16.518 + 1.481 * (m_reference.c / m_spd.c) * ctestTCS[s] - (m_reference.d / m_spd.d) * dtestTCS[s]);

        // CIE 1964 (U*, V*, W*) color space
        Wref[s] = 25 * pow(m_reference.TCSXYZ[s].Y, 1 / 3) - 17;
        Uref[s] = 13 * Wref[s] * (urefTCS[s] - m_reference.u);
        Vref[s] = 13 * Wref[s] * (vrefTCS[s] - m_reference.v);

        Wtest[s] = 25 * pow(m_spd.TCSXYZ[s].Y, 1 / 3) - 17;
        Utest[s] = 13 * Wtest[s] * (vku[s] - m_reference.u);
        Vtest[s] = 13 * Wtest[s] * (vkv[s] - m_reference.v);

        // colour difference
        Euc[s] = sqrt(pow(Wref[s] - Wtest[s], 2) + pow(Uref[s] - Utest[s], 2) + pow(Vref[s] - Vtest[s], 2));

        // CRI value
        Ri[s] = static_cast<uint8_t>(100 - 4.6 * Euc[s]);
    }
}

void ColorRenderingIndex::normalize(vector<double> values)
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

    // printf("convertToCIE1960 u = %f, v = %f\n", u, v);
}

void ColorRenderingIndex::chromaticTransform(double u, double v, double &c, double &d)
{
    c = (4 - u - 10 * v) / v;
    d = (1.708 * v - 1.481 * u + 0.404) / v;

    // printf("chromaticTransform c = %f, d = %f\n", c, d);
}

uint16_t ColorRenderingIndex::getCCT(std::vector<double> &spd)
{
    Tristimulus XYZ;
    Chromaticity xy;

    //{X = 0.096242674146876345 Y = 0.31466154678143538 Z = 0.15518062272351676 }
    //{x=0.17001457506972328 y=0.55585580555654046 }
    // CIE 1931 xy Result : (0.525, 0.4141)
    Spectrum::spectrumToXYZ(spd, XYZ);
    Spectrum::XYZtoXy(XYZ, xy);
    return Spectrum::CIE1931_xy_to_CCT(xy);
}

void ColorRenderingIndex::prepareTestSPD(vector<double> &spd)
{
    printf("*** Preparing test SPD ***\n");

    // visible spectrum (in 2nm steps)
    // TODO handle difference in steps
    const uint16_t noOfWavelengths(VISIBLE_WAVELENGTHS / 2);

    // get XYZ values
    Spectrum::spectrumToXYZ(spd, m_spd.XYZ);

    // ?
    m_spd.refl.resize(noOfWavelengths);

    for (uint8_t t = 0; t < MAX_TCS; t++)
    {
        for (uint16_t k = 0; k < noOfWavelengths; k++)
        {
            m_spd.refl[k] = spd[k] * m_TCS[k * 2][t]; // stepsize
        }

        //
        Spectrum::spectrumToXYZ(m_spd.refl, m_spd.TCSXYZ[t]);
    }

    // normalize XYZ values based on original
    const double norm = 100 / m_spd.XYZ.Y;
    m_spd.XYZ.X = m_spd.XYZ.X * norm;
    m_spd.XYZ.Y = m_spd.XYZ.Y * norm;
    m_spd.XYZ.Z = m_spd.XYZ.Z * norm;

    normalizeTCS(m_spd.TCSXYZ, norm);

    // XYZ again, reuse vars. TODO, expensive
    Spectrum::spectrumToXYZ(spd, m_spd.XYZ);

    // DEBUG
    Chromaticity xy;
    Spectrum::XYZtoXy(m_spd.XYZ, xy);
    float duv = Spectrum::CIE1931_xy_to_duv(xy);
    printf("XYZ = %f, %f, %f\n", m_spd.XYZ.X, m_spd.XYZ.Y, m_spd.XYZ.Z);
    printf("xy = %f, %f\n", xy.x, xy.y);
    printf("Duv = %f\n", duv);

    // u and v coordinates for test SPD in 1960
    convertToCIE1960(m_spd.XYZ, m_spd.u, m_spd.v);
    chromaticTransform(m_spd.u, m_spd.v, m_spd.c, m_spd.d);

    // xy coreect, duv too high, v incorrect
}

void ColorRenderingIndex::prepareReference(uint16_t CCT)
{
    printf("*** Preparing reference ***\n");

    if (CCT > 5000)
    {
        throw runtime_error("CCT > 5000K not supported yet");
    }

    // visible spectrum (in 2nm steps)
    const uint16_t noOfWavelengths(VISIBLE_WAVELENGTHS / 2);

    m_reference.reference.resize(noOfWavelengths);
    m_reference.refl.resize(noOfWavelengths);

    for (int i = 0; i < noOfWavelengths; i++)
    {
        m_reference.reference[i] = bb_spectrum(i * 2 + 380, CCT);
    }

    normalize(m_reference.reference);

    Spectrum::spectrumToXYZ(m_reference.reference, m_reference.XYZ);

    // ?
    for (uint8_t t = 0; t < MAX_TCS; t++)
    {
        for (uint16_t k = 0; k < noOfWavelengths; k++)
        {
            m_reference.refl[k] = m_reference.reference[k] * m_TCS[k * 2][t]; // k*2? stepsize
        }

        Spectrum::spectrumToXYZ(m_reference.refl, m_reference.TCSXYZ[t]);
    }

    // normalize the XYZ values based on original
    const double refNorm = 100 / m_reference.XYZ.Y;
    m_reference.XYZ.X = m_reference.XYZ.X * refNorm;
    m_reference.XYZ.Y = m_reference.XYZ.Y * refNorm;
    m_reference.XYZ.Z = m_reference.XYZ.Z * refNorm;

    normalizeTCS(m_reference.TCSXYZ, refNorm);

    // XYZ again, reuse vars. TODO, expensive
    normalize(m_reference.reference);
    Spectrum::spectrumToXYZ(m_reference.reference, m_reference.XYZ);

    convertToCIE1960(m_reference.XYZ, m_reference.u, m_reference.v);
    chromaticTransform(m_reference.u, m_reference.v, m_reference.c, m_reference.d);
}

bool ColorRenderingIndex::verifyChromaticityDistance()
{
    double DC = sqrt(pow(m_reference.u - m_spd.u, 2) + pow(m_reference.v - m_spd.v, 2));

    return (DC > 0.0054) ? false : true;
}

double ColorRenderingIndex::bb_spectrum(uint16_t wavelength, uint16_t CCT)
{
    // wavelength in meters
    double wavelengthMeter = wavelength * 1e-9;
    double radiationConstant = 1.4388e-2; // 2nd radiation constant (International Temperature Scale)

    return (3.74183e-16 * pow(wavelengthMeter, -5.0)) / (exp(radiationConstant / (wavelengthMeter * CCT)) - 1.0);
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
