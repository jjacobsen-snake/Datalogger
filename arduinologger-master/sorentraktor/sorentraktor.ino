#include <DS1302.h> // hardware timer (clock) http://www.rinkydinkelectronics.com/library.php?id=5
#include "Adafruit_MAX31855.h"
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <SD.h>

//---------------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------------
const int DELAY_IN_MILLIS = 500;
const bool SHOULD_PRINT_TO_MONITOR = true;

const String FILE_NAME_EXHAUST = "exhaust.txt";
const String FILE_NAME_MISC_TEMP = "misc.txt";
const String FILE_NAME_PRESSURE = "pressure.txt";

const double R_REF = 430.0;
const double R_NOMINAL = 100.0;

const double PRESSURE_REF_200_PSI = 3.5;
const double PRESSURE_REF_300_PSI = 5.15;
const double PRESSURE_REF_1600_PSI = 27;

const int CLK = 24;
const int CS = 23;
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Global objects
//---------------------------------------------------------------------------------
Adafruit_MAX31855 cardExhaust30(CLK, CS, 30);
Adafruit_MAX31855 cardExhaust31(CLK, CS, 31);
Adafruit_MAX31855 cardExhaust32(CLK, CS, 32);
Adafruit_MAX31855 cardExhaust33(CLK, CS, 33);
Adafruit_MAX31855 cardExhaust34(CLK, CS, 34);
Adafruit_MAX31855 cardExhaust35(CLK, CS, 35);
Adafruit_MAX31855 cardTempBeforeInt(CLK, CS, 36);
Adafruit_MAX31855 cardTempAfterInt(CLK, CS, 37);

DS1302 rtc(28, 27, 26); // hardware timer (clock)  RST, DAT, CLK
File fileExhaust;
File fileMiscTemp;
File filePressure;
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Arduino Lifecycle
//---------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  setupHardwareTime();

  if (SHOULD_PRINT_TO_MONITOR)
  {
    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Started: ");
    Serial.println(rtc.getDateStr());
    Serial.println("-------------------------------------------------------------------------------");
    Serial.println();
  }

  setupSDCard();
}

bool shouldLogToSDCard()
{
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);

  return voltage == 5.00;
}

void loop()
{
  double t30 = cardExhaust30.readCelsius();
  double t31 = cardExhaust31.readCelsius();
  double t32 = cardExhaust32.readCelsius();
  double t33 = cardExhaust33.readCelsius();
  double t34 = cardExhaust34.readCelsius();
  double t35 = cardExhaust35.readCelsius();
  double t36 = cardTempBeforeInt.readCelsius();
  double t37 = cardTempAfterInt.readCelsius();

  // pressure
  float exhaustPressure = getPressureInBar(A11, PRESSURE_REF_300_PSI);
  float turboPressure1 = getPressureInBar(A9, PRESSURE_REF_300_PSI);
  float turboPressure2 = getPressureInBar(A10, PRESSURE_REF_300_PSI);
  float dieselPressure = getPressureInBar(A7, PRESSURE_REF_300_PSI);
  float waterInjectPressure = getPressureInBar(A8, PRESSURE_REF_1600_PSI);
  float oilPressure = getPressureInBar(A6, PRESSURE_REF_200_PSI);

  if (shouldLogToSDCard() == true)
  {
    writeExhaustTemperatures(t30, t31, t32, t33, t34, t35);
    writeMiscTemperatures(t36, t37);
    writePressure(exhaustPressure, turboPressure1, turboPressure2, dieselPressure, waterInjectPressure, oilPressure);
  }

  if (SHOULD_PRINT_TO_MONITOR)
  {
    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Time: ");
    Serial.println(rtc.getTimeStr());
    Serial.println("-------------------------------------------------------------------------------");

    Serial.print("30: " + String(t30) + "C");
    Serial.print(" | 31: " + String(t31) + "C");
    Serial.print(" | 32: " + String(t32) + "C");
    Serial.print(" | 33: " + String(t33) + "C");
    Serial.print(" | 34: " + String(t34) + "C");
    Serial.println(" | 35: " + String(t35) + "C");

    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Before int: " + String(t36) + "C");
    Serial.println(" | After int: " + String(t37) + "C");

    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Exhaust: " + String(exhaustPressure) + "bar");
    Serial.print(" | Turbo 1: " + String(turboPressure1) + "bar");
    Serial.print(" | Turbo 2: " + String(turboPressure2) + "bar");
    Serial.println(" | Diesel: " + String(dieselPressure) + "bar");
    Serial.print(" | Water Inject: " + String(waterInjectPressure) + "bar");
    Serial.println(" | Oil: " + String(oilPressure) + "bar");
    Serial.println("-------------------------------------------------------------------------------");
    Serial.println();
  }

  delay(DELAY_IN_MILLIS);
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------------
float getPressureInBar(int pin, double psiFactor)
{
  int sensorVal = analogRead(pin);

  float voltage = (sensorVal * 5.0) / 1024.0;
  float pressurePascal = (psiFactor * ((float)voltage - 0.469)) * 1000000.0;
  float pressureBar = pressurePascal / 10e5;

  return pressureBar;
}

String getTimestamp()
{
  return String(rtc.getDateStr()) + " " + String(rtc.getTimeStr());
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// SD card
//---------------------------------------------------------------------------------

void writeExhaustTemperatures(double t31, double t32, double t33, double t34, double t35, double t36)
{
  fileExhaust = SD.open(FILE_NAME_EXHAUST, FILE_WRITE);

  if (fileExhaust)
  {
    fileExhaust.println(getTimestamp() + "," + String(t31) + "," + String(t32) + "," + String(t33) + "," + String(t34) + "," + String(t35) + "," + String(t36));
    fileExhaust.close();
  }
}

void writeMiscTemperatures(double beforeInt, double afterInt)
{
  fileMiscTemp = SD.open(FILE_NAME_MISC_TEMP, FILE_WRITE);

  if (fileMiscTemp)
  {
    fileMiscTemp.println(getTimestamp() + "," + String(beforeInt) + "," + String(afterInt));
    fileMiscTemp.close();
  }
}

void writePressure(double exhaustPressure, double turboPressure1, double turboPressure2, double dieselPressure, double waterInjectPressure, double oilPressure)
{
  filePressure = SD.open(FILE_NAME_PRESSURE, FILE_WRITE);

  if (filePressure)
  {
    filePressure.println(getTimestamp() + "," + String(exhaustPressure) + "," + String(turboPressure1) + "," + String(turboPressure2) + "," + String(dieselPressure) + "," + String(waterInjectPressure) + "," + String(oilPressure));
    filePressure.close();
  }
}

void setupSDCard()
{
  if (!SD.begin(10))
  {
    Serial.println("Initialization of SD card failed!");
    while (1)
      ; // halt program (infinite loop)
  }
  else
  {
    Serial.println("SD card initialization complete.");
  }

  // Create files: open and immediately close.
  fileExhaust = SD.open(FILE_NAME_EXHAUST, FILE_WRITE);
  fileExhaust.println();
  fileExhaust.close();

  fileMiscTemp = SD.open(FILE_NAME_MISC_TEMP, FILE_WRITE);
  fileMiscTemp.println();
  fileMiscTemp.close();

  filePressure = SD.open(FILE_NAME_PRESSURE, FILE_WRITE);
  filePressure.println();
  filePressure.close();
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Date and time
//---------------------------------------------------------------------------------

void setupHardwareTime()
{
  return;

  // Set the clock to run-mode, and disable the write protection
  rtc.halt(false);
  rtc.writeProtect(false);

  rtc.setDOW(SATURDAY);
  rtc.setTime(15, 9, 0);
  rtc.setDate(11, 5, 2019);
}
//---------------------------------------------------------------------------------
