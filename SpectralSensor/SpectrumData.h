#pragma once

#include <string>

class SpectrumData
{
  public:
    void getXYZCalibrationMatrix();
    void getSpectralCalibrationMatrix();
    void getCIE1931Table();

  private:
    void readCSV(std::string filename, double matrix[][10]);
};
