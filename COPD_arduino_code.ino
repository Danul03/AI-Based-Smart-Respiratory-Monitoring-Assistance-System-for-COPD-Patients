#include <Arduino.h>
#include <Wire.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================

// MAX30102
const int MAX30102_SDA_PIN = 21;
const int MAX30102_SCL_PIN = 22;

// DS18B20
const int DS18B20_PIN = 4;

// Relay module
const int RELAY_PIN = 5;

// L298N right-side channel
const int MOTOR_IN3_PIN = 18;
const int MOTOR_IN4_PIN = 19;
const int MOTOR_ENB_PIN = 23;

// Most relay modules are active LOW
const int RELAY_ON_LEVEL = LOW;
const int RELAY_OFF_LEVEL = HIGH;

// =====================================================
// TIMING SETTINGS
// =====================================================

// Send SpO2 and temperature every 100 ms
const unsigned long SEND_INTERVAL_MS = 100;

// Stop actuators when LabVIEW communication disappears
const unsigned long COMMAND_TIMEOUT_MS = 5000;

// Temperature update interval
const unsigned long TEMPERATURE_INTERVAL_MS = 1000;

// DS18B20 conversion time at 10-bit resolution
const unsigned long TEMPERATURE_CONVERSION_MS = 200;

// Keep previous valid SpO2 briefly
const unsigned long SPO2_HOLD_TIME_MS = 5000;

// MAX30102 finger detection level
const uint32_t FINGER_IR_THRESHOLD = 10000;

const unsigned long MOTOR_60_DEGREE_TIME_MS = 1000;

// MAX30102 VARIABLES

MAX30105 max30102;

const int32_t SAMPLE_COUNT = 100;
const int32_t NEW_SAMPLE_COUNT = 25;

uint32_t redBuffer[SAMPLE_COUNT];
uint32_t irBuffer[SAMPLE_COUNT];

int32_t calculatedSpO2 = -1;
int32_t calculatedHeartRate = -1;

int8_t validSpO2 = 0;
int8_t validHeartRate = 0;

int32_t storedSpO2 = -1;

bool max30102Found = false;

unsigned long lastValidSpO2Time = 0;

// DS18B20 VARIABLES

OneWire oneWire(DS18B20_PIN);
DallasTemperature temperatureSensor(&oneWire);

bool ds18b20Found = false;
bool temperatureConversionRunning = false;

float storedTemperatureC = -127.0;

unsigned long temperatureRequestTime = 0;
unsigned long previousTemperatureTime = 0;

// ACTUATOR VARIABLES

bool relayState = false;
bool bedMotorState = false;

unsigned long lastValidCommandTime = 0;
unsigned long previousSendTime = 0;

bool motorMovementActive = false;
unsigned long motorMovementStartTime = 0;

int previousMotorValue = 0;

// RELAY CONTROL

void setAirPumpRelay(bool turnOn)
{
  relayState = turnOn;

  if (turnOn)
  {
    digitalWrite(RELAY_PIN, RELAY_ON_LEVEL);
  }
  else
  {
    digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
  }
}

// BED MOTOR CONTROL

void stopBedMotor()
{
  bedMotorState = false;
  motorMovementActive = false;

  digitalWrite(MOTOR_ENB_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}

// Motor moves forward for approximately 60 degrees
void moveBedForward60Degrees()
{
  bedMotorState = true;
  motorMovementActive = true;
  motorMovementStartTime = millis();

  digitalWrite(MOTOR_IN3_PIN, HIGH);
  digitalWrite(MOTOR_IN4_PIN, LOW);
  digitalWrite(MOTOR_ENB_PIN, HIGH);
}

// Motor moves backward for approximately 60 degrees
void moveBedBackward60Degrees()
{
  bedMotorState = true;
  motorMovementActive = true;
  motorMovementStartTime = millis();

  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, HIGH);
  digitalWrite(MOTOR_ENB_PIN, HIGH);
}

// Automatically stops the motor after the calibrated time
void updateBedMotorMovement()
{
  if (
    motorMovementActive &&
    millis() - motorMovementStartTime >=
    MOTOR_60_DEGREE_TIME_MS
  )
  {
    stopBedMotor();
  }
}

void stopAllActuators()
{
  setAirPumpRelay(false);
  stopBedMotor();
}

// PROCESS COMMA-SEPARATED LABVIEW STRING

void processLabVIEWCommand(String inputString)
{

  // Removes newline, carriage return and spaces
  inputString.trim();

  // Find the comma position
  int commaIndex = inputString.indexOf(',');

  // Ignore invalid strings without a comma
  if (commaIndex < 0)
  {
    return;
  }

  // Separate the string into two parts
  String firstPart =
    inputString.substring(0, commaIndex);

  String secondPart =
    inputString.substring(commaIndex + 1);

  // Convert text into floating-point values
  float firstFloat = firstPart.toFloat();
  float secondFloat = secondPart.toFloat();

  int relayValue = 0;
  int motorValue = 0;

  if (firstFloat >= 0.5)
  {
    relayValue = 1;
  }

  if (secondFloat >= 0.5)
  {
    motorValue = 1;
  }

  // First value controls the relay
  if (relayValue == 1)
  {
    setAirPumpRelay(true);
  }
  else
  {
    setAirPumpRelay(false);
  }


  if (motorValue != previousMotorValue)
  {
    if (motorValue == 1)
    {
      moveBedForward60Degrees();
    }
    else
    {
      moveBedBackward60Degrees();
    }

    previousMotorValue = motorValue;
  }

  // Record the latest valid LabVIEW command time
  lastValidCommandTime = millis();
}

// READ STRING FROM LABVIEW

void readLabVIEWCommand()
{

  while (Serial.available() > 0)
  {
    // Read characters until newline
    String inputString =
      Serial.readStringUntil('\n');

    // Remove spaces and carriage return
    inputString.trim();

    if (inputString.length() > 0)
    {
      processLabVIEWCommand(inputString);
    }
  }
}

// SEND SENSOR VALUES TO LABVIEW

void sendSensorValues()
{
  unsigned long currentTime = millis();

  if (
    currentTime - previousSendTime >=
    SEND_INTERVAL_MS
  )
  {
    previousSendTime = currentTime;


    Serial.print(storedSpO2);
    Serial.print(",");
    Serial.println(storedTemperatureC, 2);
  }
}

// TEMPERATURE FUNCTIONS

void startTemperatureConversion()
{
  if (!ds18b20Found)
  {
    return;
  }

  temperatureSensor.requestTemperatures();

  temperatureRequestTime = millis();
  temperatureConversionRunning = true;
}

void updateTemperature()
{
  if (!ds18b20Found)
  {
    storedTemperatureC = -127.0;
    return;
  }

  unsigned long currentTime = millis();

  // Read the completed DS18B20 conversion
  if (
    temperatureConversionRunning &&
    currentTime - temperatureRequestTime >=
    TEMPERATURE_CONVERSION_MS
  )
  {
    float newTemperature =
      temperatureSensor.getTempCByIndex(0);

    if (
      newTemperature != DEVICE_DISCONNECTED_C &&
      newTemperature >= -55.0 &&
      newTemperature <= 125.0
    )
    {
      storedTemperatureC = newTemperature;
    }
    else
    {
      storedTemperatureC = -127.0;
    }

    temperatureConversionRunning = false;
  }

  // Start the next temperature measurement
  if (
    !temperatureConversionRunning &&
    currentTime - previousTemperatureTime >=
    TEMPERATURE_INTERVAL_MS
  )
  {
    previousTemperatureTime = currentTime;
    startTemperatureConversion();
  }
}

// MAX30102 SAMPLE FUNCTIONS

void waitForMax30102Sample()
{
  while (!max30102.available())
  {
    max30102.check();

    // Continue receiving LabVIEW commands
    readLabVIEWCommand();

    // Continue controlling timed motor movement
    updateBedMotorMovement();

    // Continue temperature measurement
    updateTemperature();

    // Continue sending stored sensor values
    sendSensorValues();

    delay(1);
  }
}

void readOneMax30102Sample(
  uint32_t &redValue,
  uint32_t &irValue
)
{
  waitForMax30102Sample();

  redValue = max30102.getRed();
  irValue = max30102.getIR();

  max30102.nextSample();
}

// CALCULATE SPO2

void calculateSpO2()
{
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    SAMPLE_COUNT,
    redBuffer,
    &calculatedSpO2,
    &validSpO2,
    &calculatedHeartRate,
    &validHeartRate
  );

  uint32_t latestIR =
    irBuffer[SAMPLE_COUNT - 1];

  bool fingerDetected =
    latestIR >= FINGER_IR_THRESHOLD;

  // No finger
  if (!fingerDetected)
  {
    storedSpO2 = -1;
    return;
  }

  // Valid SpO2
  if (
    validSpO2 == 1 &&
    calculatedSpO2 >= 70 &&
    calculatedSpO2 <= 100
  )
  {
    storedSpO2 = calculatedSpO2;
    lastValidSpO2Time = millis();
  }
  else
  {
    // Remove an old value after five seconds
    if (
      millis() - lastValidSpO2Time >
      SPO2_HOLD_TIME_MS
    )
    {
      storedSpO2 = -1;
    }
  }
}

// COLLECT INITIAL MAX30102 SAMPLES

void collectInitialSamples()
{
  for (int32_t i = 0; i < SAMPLE_COUNT; i++)
  {
    readOneMax30102Sample(
      redBuffer[i],
      irBuffer[i]
    );
  }

  calculateSpO2();
}
// UPDATE MAX30102

void updateSpO2()
{
  if (!max30102Found)
  {
    storedSpO2 = -1;
    return;
  }

  // Keep the newest 75 samples
  for (
    int32_t i = NEW_SAMPLE_COUNT;
    i < SAMPLE_COUNT;
    i++
  )
  {
    redBuffer[i - NEW_SAMPLE_COUNT] =
      redBuffer[i];

    irBuffer[i - NEW_SAMPLE_COUNT] =
      irBuffer[i];
  }

  // Collect 25 new samples
  for (
    int32_t i = SAMPLE_COUNT - NEW_SAMPLE_COUNT;
    i < SAMPLE_COUNT;
    i++
  )
  {
    readOneMax30102Sample(
      redBuffer[i],
      irBuffer[i]
    );
  }

  calculateSpO2();
}

// SETUP

void setup()
{
  Serial.begin(115200);

  Serial.setTimeout(50);

  delay(500);

  // Actuator pins

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);
  pinMode(MOTOR_ENB_PIN, OUTPUT);

  stopAllActuators();

  lastValidCommandTime = millis();

  // Initial LabVIEW motor state is OFF
  previousMotorValue = 0;

  // DS18B20 setup

  temperatureSensor.begin();

  ds18b20Found =
    temperatureSensor.getDeviceCount() > 0;

  if (ds18b20Found)
  {
    temperatureSensor.setResolution(10);
    temperatureSensor.setWaitForConversion(false);

    previousTemperatureTime = millis();
    startTemperatureConversion();
  }
  else
  {
    storedTemperatureC = -127.0;
  }

  // MAX30102 setup

  Wire.begin(
    MAX30102_SDA_PIN,
    MAX30102_SCL_PIN
  );

  max30102Found =
    max30102.begin(
      Wire,
      I2C_SPEED_STANDARD
    );

  if (max30102Found)
  {
    byte ledBrightness = 60;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;

    max30102.setup(
      ledBrightness,
      sampleAverage,
      ledMode,
      sampleRate,
      pulseWidth,
      adcRange
    );

    max30102.setPulseAmplitudeRed(0x3C);
    max30102.setPulseAmplitudeIR(0x3C);
    max30102.setPulseAmplitudeGreen(0);

    collectInitialSamples();
  }
  else
  {
    storedSpO2 = -1;
  }
}

// MAIN LOOP

void loop()
{
  // Receive actuator values from LabVIEW
  readLabVIEWCommand();

  // Stop motor automatically after approximately 60°
  updateBedMotorMovement();

  // Update temperature
  updateTemperature();

  // Send SpO2 and temperature to LabVIEW
  sendSensorValues();

  // Update SpO2
  if (max30102Found)
  {
    updateSpO2();
  }
  else
  {
    /*
      Continue LabVIEW communication even when
      the MAX30102 is disconnected.
    */

    unsigned long waitStart = millis();

    while (millis() - waitStart < 1000)
    {
      readLabVIEWCommand();
      updateBedMotorMovement();
      updateTemperature();
      sendSensorValues();

      delay(5);
    }
  }


  if (
    millis() - lastValidCommandTime >
    COMMAND_TIMEOUT_MS
  )
  {
    if (relayState || bedMotorState)
    {
      stopAllActuators();
    }
  }

  delay(1);
}