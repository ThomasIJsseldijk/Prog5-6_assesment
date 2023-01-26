/**
 * @file pinHelper.h
 * @author Thomas IJsseldijk (fhm.ijsseldijk@student.han.nl)
 * @brief a helper file with all helperfunctions and pin definitions for the ventilation controller
 * @version 0.1
 * @date 23-01-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef PINHELPER_H
#define PINHELPER_H


#include <Wire.h>
#include <SPI.h>
#include <wiring_private.h>
#include <Adafruit_FRAM_SPI.h>

//module address
#define ventilatieControllerAdress 0x01


//debug macros. can be used to only run pieces of code when debug mode is enabled.
#define enableDebug true
#define DEBUG(x) if(enableDebug){x;}


//all defines for the fram
#define FRAM_CS 21
#define FRAM_SCK 3
#define FRAM_MISO 2
#define FRAM_MOSI 4

Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

void initFram(){
    
    if (fram.begin()) {
    DEBUG(Serial.println("Found SPI FRAM"));
  } else {
    DEBUG(Serial.println("No SPI FRAM found ... check your connections\r\n"));
    while (1);
  }
    }


//all defines for the sensor B port on the sensorHub
#define sensorBSDA 28
#define sensorBSCL 39
const EPioType sensorBsercomType = PIO_SERCOM;
SERCOM   *sensorBsercom = &sercom2;
TwoWire sensorB(sensorBsercom, sensorBSDA, sensorBSCL);

void initSensorB(){
    sensorB.begin();
    pinPeripheral(sensorBSDA, sensorBsercomType);
    pinPeripheral(sensorBSCL, sensorBsercomType);
}

//all defines for the sensor A port on the sensorHub
#define sensorASDA 13
#define sensorASCL 11
const EPioType sensorAsercomType = PIO_SERCOM;
SERCOM   *sensorAsercom = &sercom1;
TwoWire sensorA(sensorAsercom, sensorASDA, sensorASCL);

void initSensorA(){
    sensorA.begin();
    pinPeripheral(sensorASDA, sensorAsercomType);
    pinPeripheral(sensorASCL, sensorAsercomType);
}



//all defines for the backbone port on the sensorHub
#define BackboneSDA 26
#define BackboneSCL 27
const EPioType BackboneSercomType = PIO_SERCOM;
SERCOM *BackboneSercom = &sercom3;
TwoWire Backbone(BackboneSercom, BackboneSDA, BackboneSCL);

void initBackbone(){

    Backbone.begin();
    pinPeripheral(BackboneSDA, BackboneSercomType);
    pinPeripheral(BackboneSCL, BackboneSercomType);
}

//led pins
#define ledHB 14
#define ledHealth 5

//sensor addresses
#define pressureSensorAddress 0x25






#endif
