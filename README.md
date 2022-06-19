# DIY spectrometer

DIY spectrometer based on AMS AS7341 sensor.

Uncomment this line in SpectralSensor.cpp to use XYZ calibration matrix. Otherwise spectral calibration matrix is used.

    #define USE_XYZ_CALIBRATION_MATRIX

### Notes on verification

To test implemented calculations against AMS example calculations uncomment the following define in SpectralSensor.h

    #define VERIFY_CALCS

To test AS7341 code against AMS example calculations, using uncomment the following define in AS7341.h. This replaces actual readings with the raw counts provided by AMS.

    #define VERIFY_CALCS_AS7341
