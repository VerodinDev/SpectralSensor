#pragma once

#include "Spectrum.h"
#include <cstdint>
#include <string>
#include <vector>

//struct CRI
//{
//    uint8_t R1;
//    uint8_t R2;
//    uint8_t R3;
//    uint8_t R4;
//    uint8_t R5;
//    uint8_t R6;
//    uint8_t R7;
//    uint8_t R8;
//    uint8_t R9;
//    uint8_t R10;
//    uint8_t R11;
//    uint8_t R12;
//    uint8_t R13;
//    uint8_t R14;
//    uint8_t R15;
//};

// only TCS1 to TCS14 supported, need values for TCS15
const uint8_t MAX_TCS = 14;

class ColorRenderingIndex
{
  public:
    ColorRenderingIndex();

    void loadTCSTable();

    // calculate CRI values
    void calculateCRI(std::vector<double> &spd, uint8_t Ri[]);

  private:
    typedef std::vector<std::vector<float>> TCSTable;

    void normalize(std::vector<double> values);

    // normalize TCS XYZ values based on original
    void normalizeTCS(Tristimulus XYZ[], double Ynorm);

    // convert chromaticities to the CIE 1960
    void convertToCIE1960(const Tristimulus &XYZ, double &u, double &v);

    // use von Kries chromatic transform equation to find the corresponding color
    // c and d constants for use in Von Kries
    void chromaticTransform(double u, double v, double &c, double &d);

    // get CCT to determine reference illuminant
    uint16_t getCCT(std::vector<double>& spd);

    // 
    void prepareTestSPD(std::vector<double>& spd);

    // setup reference since those values won't change
    void prepareReference(uint16_t CCT);

    // Ensure that the chromaticity distance (DC) of the test source to the Planckian locus is under 5.4x10-3 in the CIE 1960 UCS
    bool verifyChromaticityDistance();

    // Planckian locus approximation
    double bb_spectrum(uint16_t wavelength, uint16_t CCT);

    void readCSV(const std::string &filename, TCSTable& table);

    // TCS1 to TCS14
    // TODO obtain TSC15 in 1mn steps
    // float m_TCS[visibleWavelengths][14];
    TCSTable m_TCS;

    struct TestSPD
    {
        std::vector<double> spd;
        std::vector<double> refl;
        Tristimulus XYZ;
        Tristimulus TCSXYZ[MAX_TCS];
        double u;                       //< u and v coordinates for reference and test in 1960
        double v;
        double c;                       //< von Kries chromatic transform
        double d;
    };

    struct Reference
    {
        std::vector<double> reference;
        std::vector<double> refl;
        Tristimulus XYZ;
        Tristimulus TCSXYZ[MAX_TCS];
        double u;                       //< u and v coordinates for reference and test in 1960
        double v;
        double c;                       //< von Kries chromatic transform
        double d;
    };

    // SPD
    TestSPD m_spd;

    // reference illuminant
    Reference m_reference;
};
