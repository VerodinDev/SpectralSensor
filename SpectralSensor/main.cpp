#include "AS7341.h"
#include "SpectralSensor.h"
#include <stdexcept>

#include "ColorRenderingIndex.h"
#include "Spectrum.h"
#include "TestData.h"

#include <string>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

////  TODO merge with other CSV reading. FileOps.cpp?
// typedef std::vector<std::vector<double>> CIE1931Table;
// void readCSV(const string& filename, CIE1931Table& table)
//{
//     ifstream csvFile;
//
//     csvFile.open(filename, ifstream::in);
//     if (!csvFile.is_open())
//     {
//         throw runtime_error("Error opening data file " + filename);
//     }
//
//     string line;
//     uint16_t row(0);
//
//     // skip header
//     getline(csvFile, line);
//
//     // read values
//     // wl X Y Z
//     while (getline(csvFile, line))
//     {
//         replace(line.begin(), line.end(), ';', ' ');
//
//         stringstream ss(line);
//         float value(0);
//         uint8_t column = 0;
//
//         // skip wavelength column
//         ss >> value;
//
//         // note that there is NO faulty input data check whatsoever!!!
//         while (ss >> value)
//         {
//             table[row][column] = value;
//             column++;
//         }
//
//         row++;
//     }
//
//     csvFile.close();
// }

int main()
{
    printf("*** Spectral sensor v0.2 ***\n\n");

    //// init CIE1931 table
    // CIE1931Table cie1931Table;
    // uint16_t rows = VISIBLE_WAVELENGTHS;
    // uint8_t cols = 3;
    // cie1931Table.resize(rows);

    // for (uint16_t i = 0; i < rows; i++)
    //{
    //     cie1931Table[i].resize(cols);
    // }

    // readCSV("../data/CIE1931 2degree.csv", cie1931Table);

#ifdef VERIFY_CALCS

    printf("*** VERIFICATION MODE ***\n\n");

    SpectralSensor sensor;
    sensor.loadCorrectionMatrix();

    //sensor.checkChannelDataCalcs();
    //sensor.checkCIE1931Calcs(10);
    //sensor.checkCIE1931Calcs(CHANNEL_F8);
    //sensor.verifySpectralReconstruction();

    try
    {
        printf("\n*** verify CRI calculation ***\n");

        // TCS table
        uint8_t Ri[MAX_TCS];

        ColorRenderingIndex cri;
        cri.loadTCSTable();

        Spectrum testSpd(/*FW3A_2700*/ white_2700_osram);
        cri.calculateCRI(testSpd, Ri);

        printf("Result:\n");
        for (uint8_t i = 0; i < MAX_TCS; i++)
        {
            printf("R%d: %d | ", i + 1, Ri[i]);
        }
        printf("\n");

        // Ra based on R1 to R8
        // uint16_t Rsum = accumulate(vector.begin(), vector.end(), ecltype(vector)::value_type(0));
        // uint8_t Ra = Rsum / 8;
    }
    catch (const std::exception &e)
    {
        printf("Exception: %s\n", e.what());
    }

#else

    try
    {
        SpectralSensor sensor;

        // setup
        sensor.setupI2C();
        sensor.setupAS7341();
        sensor.loadCorrectionMatrix();

        // take n readings
        /*for (uint8_t readings = 0; readings < 5; readings++)
        {*/
        sensor.takeReading();
        //}
    }
    catch (const std::exception &e)
    {
        printf("Exception: %s\n", e.what());
    }

#endif
}
