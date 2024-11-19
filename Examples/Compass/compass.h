#ifndef COMPASS_H
#define COMPASS_H

//https://github.com/mprograms/QMC5883LCompass
#include <QMC5883LCompass.h>

class Compass {
  QMC5883LCompass compass;

 public:
    //Do we need to pass in SCL and SDA in to make a construction of this class?
    //   Compass() : compass(SCL, SDA, U8X8_PIN_NONE) {}
    Compass() : compass() {}

  void setup() {
        compass.init();
    }

    void setSmoothing(byte steps, bool advanced) {
        compass.setSmoothing(steps, advanced);
    }

    void read() {
        compass.read();
    }

    int getAzimuth() {
        return compass.getAzimuth();
    }
};

#endif
