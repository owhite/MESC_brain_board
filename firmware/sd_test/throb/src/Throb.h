#ifndef Throb_h
#define Throb_h

#include "Arduino.h"

class Throb {
  public:
    Throb(int pin);
    void pulse();
    void stop();
    void goDark();
    void fullOn();

  private:
    boolean _blinkOn = false;
    uint32_t _blinkDelta = 0;
    uint32_t _blinkInterval = 2000; 
    uint32_t _blinkNow;
    uint32_t _blinkDuty;
    uint32_t _blinkUpTime;
    boolean _LEDstate;
    int _loopCounter;
    int _pwmCount;

    // int _pwmArray[3] = {1, 2, 4};

    int _pwmArray[100] = {1, 2, 4,  6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 98, 96, 94, 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0};

    int _pin;
    void pwmLED(int pin, int val);
};

#endif
