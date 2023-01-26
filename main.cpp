#include <Arduino.h>
#include <SDP810.h>
#include <FreeRTOS_SAMD21.h>
#include <Adafruit_FRAM_SPI.h>
#include <SPI.h>
#include "pinHelper.h"

SDP810 flowSensor(pressureSensorAddress);
SDP810 pressureSensor(pressureSensorAddress);

void TaskHeartbeat(void *ptr);
TaskHandle_t heartbeatHandle;

void TaskReadPressureSensor(void *ptr);
TaskHandle_t ReadPressureSensorHandle;

void TaskReadFlowSensor(void *ptr);
TaskHandle_t ReadFlowSensorHandle;

void TaskWriteFram(void *ptr);
TaskHandle_t WriteFramHandle;

int pressure;
static SemaphoreHandle_t pressureMutex = xSemaphoreCreateMutex();

int flow;
static SemaphoreHandle_t flowMutex = xSemaphoreCreateMutex();

bool state = true;

void setup()
{

  DEBUG(
      Serial.begin(115200);
      while (!Serial);
      Serial.println("connected"))

  pinMode(ledHealth, OUTPUT);

  xTaskCreate(TaskHeartbeat, "heartbeat", configMINIMAL_STACK_SIZE + 100, NULL, 1, &heartbeatHandle);
  xTaskCreate(TaskReadFlowSensor, "readFlowSensor", configMINIMAL_STACK_SIZE + 500, NULL, 2, &ReadFlowSensorHandle);
  xTaskCreate(TaskReadPressureSensor, "readPressureSensor", configMINIMAL_STACK_SIZE + 500, NULL, 2, &ReadPressureSensorHandle);
  // xTaskCreate(TaskWriteFram, "writeFram", configMINIMAL_STACK_SIZE + 100, NULL, 2, &WriteFramHandle);

  vTaskStartScheduler();
}

/**
 * @brief arduino loop function. does nothing in this code.
 *
 */
void loop()
{

  // vTaskDelay(500/ portTICK_PERIOD_MS);
}

/**
 * @brief Task to blink an LED (Heartbeat)
 * @param[in] ptr void pointer to pass data to task (unused)
 * This task will blink an LED at a specific rate to indicate that the system is running.
 */
void TaskHeartbeat(void *ptr)
{

  pinMode(ledHB, OUTPUT);
  while (1)
  {
    digitalWrite(ledHB, LOW);
    vTaskDelay(900 / portTICK_PERIOD_MS);
    digitalWrite(ledHB, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
* @brief Task to read the pressure sensor
* @param[in] ptr void pointer to pass data to task (unused)
* This task will initialize and read the pressure sensor at a regular interval,
* and store the result in a global variable protected by a mutex.
 */
void TaskReadPressureSensor(void *ptr)
{
  initSensorB();
  pressureSensor.begin(&sensorB);
  while (1)
  {

    pressureSensor.read();
    if (xSemaphoreTake(pressureMutex, 10 / portTICK_PERIOD_MS) == pdTRUE)
    {
      pressure = pressureSensor.getRaw();
      xSemaphoreGive(pressureMutex);
      DEBUG(Serial.print(pressure / 50));
      DEBUG(Serial.print("   "));
      DEBUG(Serial.println(flow));
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

/**
* @brief Task to read the flow sensor
* @param[in] ptr void pointer to pass data to task (unused)
* This task will initialize and read the flow sensor at a regular interval,
* and store the result in a global variable protected by a mutex.
 */

void TaskReadFlowSensor(void *ptr)
{
  initSensorA();
  flowSensor.begin(&sensorA);

  while (1)
  {
    flowSensor.read();
    if (xSemaphoreTake(flowMutex, 10 / portTICK_PERIOD_MS) == pdTRUE)
    {
      flow = flowSensor.getRaw();
      xSemaphoreGive(flowMutex);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

void TaskWriteFram(void *ptr)
{

  initFram();

  uint32_t currentAddress = 0x00000000;
  uint8_t writeBuffer[4];

  while (1)
  {

    writeBuffer[0] = flow;
    writeBuffer[1] = flow << 8;
    writeBuffer[2] = pressure;
    writeBuffer[3] = pressure << 8;

    fram.writeEnable(true);
    fram.write(currentAddress, writeBuffer, sizeof(writeBuffer));
    fram.writeEnable(false);
    currentAddress += 4;

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}
