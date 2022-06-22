#pragma once

#include "Spectrum.h"
#include <cstdint>
#include <string>
#include <vector>

// only TCS1 to TCS14 supported, need values for TCS15
// TODO obtain TSC15 in 1mn steps
const uint8_t MAX_TCS = 14;

class ColorRenderingIndex
{
  public:
    ColorRenderingIndex();

    void loadTCSTable();

    // calculate CRI values
    void calculateCRI2(std::vector<double> &spd, uint8_t Ri[]);

  private:
    typedef std::vector<std::vector<float>> TCSTable;

    void normalize(std::vector<double> &values);

    // normalize TCS XYZ values based on original
    void normalizeTCS(Tristimulus XYZ[], double Ynorm);

    // convert chromaticities to the CIE 1960 / CIE 1976
    void convertToCIE1960(const Tristimulus &XYZ, double &u, double &v);
    void convertToCIE1960(const Chromaticity &xy, double &u, double &v);
    void convertToCIE1976(const Chromaticity &xy, double &u, double &v);

    // use von Kries chromatic transform equation to find the corresponding color
    // c and d constants for use in Von Kries
    void chromaticTransform(double u, double v, double &c, double &d);

    // get CCT to determine reference illuminant
    uint16_t getCCT(const std::vector<double> &spd);

    //
    void prepareTestSPD(const std::vector<double> &spd);

    // setup reference since those values won't change
    void prepareReference2(const std::vector<double> &spd);

    // make sure chromaticity distance (DC) is under 5.4x10-3 in CIE 1960
    // CRI is only defined for light sources that are approximately white
    // bool verifyChromaticityDistance();

    // Planckian locus
    double getPlanckianLocus(uint16_t wavelength, uint16_t T);

    void readCSV(const std::string &filename, TCSTable &table);

    // TCS table
    TCSTable m_TCS;

    struct Reference
    {
        std::vector<double> reference;
        std::vector<double> refl;
        Tristimulus XYZ;
        Tristimulus TCSXYZ[MAX_TCS];
        double YNormal = 0;
        double u = 0; //< u and v coordinates for reference and test in 1960
        double v = 0;
        double c = 0; //< von Kries chromatic transform
        double d = 0;
    };

    // reference illuminant
    Reference m_reference;
};
