#pragma once

#include <cstdint>
#include <string>
#include <vector>

const uint16_t ALL_WAVELENGTHS = 1000 - 380 + 1;
const uint16_t VISIBLE_WAVELENGTHS = 780 - 380 + 1;

struct Tristimulus
{
    Tristimulus() : X(0), Y(0), Z(0)
    {
    }

    double X;
    double Y;
    double Z;
};

struct Chromaticity
{
    Chromaticity() : x(0), y(0)
    {
    }

    double x;
    double y;
};

class Spectrum
{
  public:
    typedef std::vector<std::vector<double>> CIE1931Table;

    Spectrum();
    Spectrum(const std::vector<double> &spd);

    std::vector<double> &getSpd();
    uint16_t getNoOfWavelengths() const;
    void normalize();
    void getXYZ(Tristimulus &XYZ) const;
    uint16_t getCCT() const;
    void resize(uint16_t newSize);

    // save to CSV
    void saveToCsv(const std::string &filename);

    // XYZ to xy
    void XYZtoXy(const Tristimulus &XYZ, Chromaticity &xy) const;

    // CIE 1931 xy to CCT
    // McCamy's approximation
    uint16_t xyToCCT(const Chromaticity &xy) const;
    uint16_t xyToCCT_wikipedia(const Chromaticity &xy) const;

    // CIE 1931 xy to Duv
    float xyToDuv(const Chromaticity &xy) const;

    double &operator[](int i);

  private:
    void loadCIE1931Table();
    void readCSV(const std::string &filename, CIE1931Table &table);

    std::vector<double> m_spd;
    CIE1931Table m_cie1931Table;
};
