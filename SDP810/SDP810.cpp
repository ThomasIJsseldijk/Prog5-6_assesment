/**
 * @file SDP810.cpp
 * @author Thomas IJsseldijk (fhm.ijsseldijk@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 07-10-2022
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "SDP810.h"



/**
 * @brief Construct a new SDP810::SDP810 object
 * 
 * @param sensorAddress 
 */
SDP810::SDP810(char sensorAddress)
{
    SDP810address = sensorAddress;
}


/**
 * @brief Destroy the SDP810::SDP810 object
 * 
 */
SDP810::~SDP810()
{
}


/**
 * @brief Initialize communication with the SDP810 sensor
 * @param[in] wire TwoWire object to use for communication with the sensor
 * This function initializes communication with the SDP810 sensor by setting up
 * the I2C communication object and configuring the sensor settings.
*/
void SDP810::begin(TwoWire *wire)
{
    byte setting[2] = {0x36, 0x15};
    SDP810wire = wire;

    SDP810wire->beginTransmission(SDP810address);
    SDP810wire->write(setting, sizeof(setting));
    SDP810wire->endTransmission(true);
}



/**
 * @brief Read data from the SDP810 sensor
 * This function reads the data from the SDP810 sensor by sending a request to the sensor
 * to send the data, and then read the data buffer. It also converts the sensor data into
 * the appropriate format.
 */
void SDP810::read()
{

    SDP810wire->requestFrom(SDP810address, sizeof(buffer), true);
    SDP810wire->readBytes(buffer, sizeof(buffer));

    conversionFactor = buffer[6] << 8 | buffer[7];
    sensorRaw = (buffer[0] << 8 | buffer[1]);
    sensorRaw = sensorRaw / conversionFactor; 
}

/**
 * @brief Get the raw sensor data
 * @return int16_t the raw sensor data in int16_t format
 * This function returns the raw sensor data in int16_t format.
 */
int16_t SDP810::getRaw()
{
    return sensorRaw;
}

/**
 * @brief Get the volume from the sensor data
 * @return float the volume from the sensor data
 * This function returns the volume from the sensor data.
 */
float SDP810::getVolume()
{
return sqrt(sensorRaw*2);
}
