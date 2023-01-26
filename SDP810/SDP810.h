#ifndef SDP810_H
#define SDP810_H


#include <Wire.h>
#include <Arduino.h>

class SDP810
{
private:
    /* data */

double flow;
int16_t sensorRaw;
int16_t conversionFactor;

TwoWire *SDP810wire;
char SDP810address = 0;


byte buffer[9];


public:

    SDP810(char sensorAddress);
    ~SDP810();
    void begin(TwoWire *wire);
    void read();
    int16_t getRaw();
    float getVolume();

};

#endif // SDP810




