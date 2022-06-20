#include "SpectrumData.h"
#include <fstream>
#include <algorithm>
#include <vector>
#include <sstream>

using namespace std;

void SpectrumData::getXYZCalibrationMatrix()
{
    // TODO use vector
    double calibrationMatrix[3][10];

    readCSV("../data/XYZCorrectionMatrix_v3.0.csv", calibrationMatrix);

    printf("\nresult\n");
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 10; col++)
        {
            printf("%f, ", calibrationMatrix[row][col]);
        }

        printf("\n");
    }
}

void SpectrumData::getSpectralCalibrationMatrix()
{
}

void SpectrumData::getCIE1931Table()
{

}

void SpectrumData::readCSV(string filename, double matrix[][10])
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
    while (getline(csvFile, line))
    {
        replace(line.begin(), line.end(), ';', ' ');

        stringstream ss(line);
        double value(0);
        uint8_t column = 0;

        while (ss >> value)
        {
            matrix[row][column] = value;
            column++;
        }

        row++;
    }

    csvFile.close();
}