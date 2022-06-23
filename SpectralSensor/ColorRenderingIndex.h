#pragma once

#include "Spectrum.h"

// only TCS1 to TCS14 supported, need values for TCS15
// TODO obtain TSC15 in 1mn steps
const uint8_t MAX_TCS = 14;

class ColorRenderingIndex
{
  public:
    ColorRenderingIndex();

    void loadTCSTable();

    // calculate CRI values
    void calculateCRI(Spectrum &spd, uint8_t Ri[]);

  private:
    typedef std::vector<std::vector<float>> TCSTable;

    void prepareReference(uint16_t cct, uint8_t stepsize);

    // normalize TCS XYZ values
    void normalizeTCS(Tristimulus XYZ[], double Ynorm);

    // convert chromaticities to CIE 1960 / CIE 1976
    void convertToCIE1960(const Tristimulus &XYZ, double &u, double &v);
    void convertToCIE1960(const Chromaticity &xy, double &u, double &v);
    void convertToCIE1976(const Chromaticity &xy, double &u, double &v);

    // use von Kries chromatic transform equation to find the corresponding color
    void chromaticTransform(double u, double v, double &c, double &d);

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
        Spectrum reference;
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
