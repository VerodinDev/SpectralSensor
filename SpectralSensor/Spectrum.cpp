#include "Spectrum.h"

#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std;

Spectrum::Spectrum()
{
    loadCIE1931Table();
}

Spectrum::Spectrum(const std::vector<double> &spd) : m_spd(spd)
{
    loadCIE1931Table();
}

std::vector<double> &Spectrum::getSpd()
{
    return m_spd;
}

uint16_t Spectrum::getNoOfWavelengths() const
{
    return static_cast<uint16_t>(m_spd.size());
}

void Spectrum::normalize()
{
    double highestValue(0);

    for (uint16_t i = 0; i < m_spd.size(); i++)
    {
        if (m_spd[i] > highestValue)
        {
            highestValue = m_spd[i];
        }
    }

    for (uint16_t i = 0; i < m_spd.size(); i++)
    {
        m_spd[i] /= highestValue;
    }
}

void Spectrum::resize(uint16_t newSize)
{
    m_spd.resize(newSize);
}

void Spectrum::getXYZ(Tristimulus &XYZ) const
{
    uint8_t stepsize = m_spd.size() > 201 ? 1 : 2;

    for (uint16_t i = 0; i < m_spd.size(); i++)
    {
        XYZ.X += m_spd[i] * m_cie1931Table[i * stepsize][0];
        XYZ.Y += m_spd[i] * m_cie1931Table[i * stepsize][1];
        XYZ.Z += m_spd[i] * m_cie1931Table[i * stepsize][2];
    }
}

void Spectrum::XYZtoXy(const Tristimulus &XYZ, Chromaticity &xy) const
{
    double XYZsum = XYZ.X + XYZ.Y + XYZ.Z;

    xy.x = XYZ.X / XYZsum;
    xy.y = XYZ.Y / XYZsum;
}

uint16_t Spectrum::xyToCCT(const Chromaticity &xy) const
{
    // See below for the formula used to approximate CCT using CIE 1931 xy values :
    // McCamy's approximation
    // Source: AMS excel
    // Note 1: differs from wikipedia: CCT(x,y)=-449n^{3}+3525n^{2}-6823.3n+5520.33
    // Note 2: waveformlighting.com uses same formula as AMS

    double n = (xy.x - 0.3320) / (0.1858 - xy.y);
    uint16_t cct = static_cast<uint16_t>(437 * pow(n, 3) + 3601 * pow(n, 2) + 6861 * n + 5517);

    return cct;
}

uint16_t Spectrum::xyToCCT_wikipedia(const Chromaticity &xy) const
{
    double n = (xy.x - 0.325) / (xy.y - 0.154);
    uint16_t cct = static_cast<uint16_t>(-449 * pow(n, 3) + 3525 * pow(n, 2) - 6823.3 * n + 5520.33);

    return cct;
}

uint16_t Spectrum::getCCT() const
{
    Tristimulus XYZ;
    Chromaticity xy;

    getXYZ(XYZ);
    XYZtoXy(XYZ, xy);

    // debug
    uint16_t CCTwp = xyToCCT_wikipedia(xy);
    printf("CCT = %dK (Wikipedia)\n", CCTwp);

    return xyToCCT(xy);
}

float Spectrum::xyToDuv(const Chromaticity &xy) const
{
    // convert chromaticities to CIE 1960
    double u = (4 * xy.x) / (-2 * xy.x + 12 * xy.y + 3);
    double v = (6 * xy.y) / (-2 * xy.x + 12 * xy.y + 3);

    // I have no idea what all these magic numbers mean...
    const double k6 = -0.00616793;
    const double k5 = 0.0893944;
    const double k4 = -0.5179722;
    const double k3 = 1.5317403;
    const double k2 = -2.4243787;
    const double k1 = 1.925865;
    const double k0 = -0.471106;

    double Lfp = sqrt(pow((u - 0.292), 2) + pow((v - 0.24), 2));
    double a = acos((u - 0.292) / Lfp);
    double Lbb = k6 * pow(a, 6) + k5 * pow(a, 5) + k4 * pow(a, 4) + k3 * pow(a, 3) + k2 * pow(a, 2) + k1 * a + k0;

    return static_cast<float>(Lfp - Lbb);
}

double &Spectrum::operator[](int i)
{
    if (i > m_spd.size())
    {
        // cout << "Index out of bounds" << endl;
        return m_spd[0];
    }

    return m_spd[i];
}

void Spectrum::saveToCsv(const std::string &filename)
{
    std::ofstream cvsFile;

    //__DATE__ __TIME__

    cvsFile.open(filename);

    // header
    // cvsFile << "nm  value" << std::endl;

    // values
    for (uint16_t wavelength = 0; wavelength < m_spd.size(); wavelength++)
    {
        cvsFile << wavelength + 380 << " " << m_spd[wavelength] << std::endl;
    }

    cvsFile.close();
}

void Spectrum::loadCIE1931Table()
{
    // init CIE1931 table
    uint16_t rows = VISIBLE_WAVELENGTHS;
    uint8_t cols = 3;
    m_cie1931Table.resize(rows);

    for (uint16_t i = 0; i < rows; i++)
    {
        m_cie1931Table[i].resize(cols);
    }

    readCSV("../data/CIE1931 2degree.csv", m_cie1931Table);
}

void Spectrum::readCSV(const string &filename, CIE1931Table &table)
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
    // wl X Y Z
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
