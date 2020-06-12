//#include <DS1302.h> // hardware timer (clock) http://www.rinkydinkelectronics.com/library.php?id=5
#include <TimeLib.h>
#include "Adafruit_MAX31855.h"
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <SD.h>
#include <Nextion.h>

//---------------------------------------------------------------------------------
// Pin definitions
//---------------------------------------------------------------------------------
#define pressureBeforeIntPin      A7    // Pressure Before Intercooler
#define pressureAfterIntPin       A8    // Pressure After Intercooler
#define pressureOilEnginePin      A9    // Pressure Engine Oil
#define pressureOilTurboPin       A10    // Pressure Turbo Oil
#define pressureWaterInjectPin    A4    // Pressure Water Injection
#define pressureManifoilPin       A5    // Pressure Manifold
#define pressureFuelPin           A6    // Pressure Fuel
#define CLK1                      24    // CLK for MAX31855 & MAX31865
#define MISO1                     25    // MISO for MAX31855 & MAX31865
#define MOSI1                     26    // MOSI for MAX31865
#define Ext1Pin                   30    // CS for MAX31855
#define Ext2Pin                   31    // CS for MAX31855
#define Ext3Pin                   32    // CS for MAX31855
#define Ext4Pin                   33    // CS for MAX31855
#define Ext5Pin                   34    // CS for MAX31855
#define Ext6Pin                   35    // CS for MAX31855
#define Ext7Pin                   22    // CS for MAX31855
#define Ext8Pin                   23    // CS for MAX31855
#define Ext9Pin                   27    // CS for MAX31855
#define Ext10Pin                  28    // CS for MAX31855
#define Ext11Pin                  29    // CS for MAX31855
#define Ext12Pin                  40    // CS for MAX31855
#define TempBeforeIntPin          36    // CS for MAX31855
#define TempAfterIntPin           37    // CS for MAX31855
#define WaterTempPin              38    // CS for MAX31865
#define OilTempPin                39    // CS for MAX31865
#define DS1302rstPin              16    // RST for DS1302
#define DS1302datPin              15    // DATA for DS1302
#define DS1302clkPin              14    // CLK for DS1302
#define VinPin                    A0    // Input Voltage
#define IntTempPin                A1    // Internal Temperature
#define SDcsPin                   10    // CS SD
#define logBTNpin                 4     // Enable logging

//---------------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------------
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

const int DELAY_IN_MILLIS = 500;
const bool SHOULD_PRINT_TO_MONITOR = true;
bool shouldLogToSDCard = false;

const double R_REF = 430.0;
const double R_NOMINAL = 100.0;

const double PRESSURE_REF_200_PSI = 3.5;
const double PRESSURE_REF_300_PSI = 5.15;
const double PRESSURE_REF_1600_PSI = 27;

double maxValues[10];
// float mp[10];
double maxPressureOilTurbo;
double maxOilTemp;

//---------------------------------------------------------------------------------
// Global objects
//---------------------------------------------------------------------------------
Adafruit_MAX31855 cardExt1(CLK1, Ext1Pin, MISO1); // Adafruit_MAX31855(int8_t _sclk, int8_t _cs, int8_t _miso);
Adafruit_MAX31855 cardExt2(CLK1, Ext2Pin, MISO1);
Adafruit_MAX31855 cardExt3(CLK1, Ext3Pin, MISO1);
Adafruit_MAX31855 cardExt4(CLK1, Ext4Pin, MISO1);
Adafruit_MAX31855 cardExt5(CLK1, Ext5Pin, MISO1);
Adafruit_MAX31855 cardExt6(CLK1, Ext6Pin, MISO1);
Adafruit_MAX31855 cardExt7(CLK1, Ext7Pin, MISO1);
Adafruit_MAX31855 cardExt8(CLK1, Ext8Pin, MISO1);
Adafruit_MAX31855 cardExt9(CLK1, Ext9Pin, MISO1);
Adafruit_MAX31855 cardExt10(CLK1, Ext10Pin, MISO1);
Adafruit_MAX31855 cardExt11(CLK1, Ext11Pin, MISO1);
Adafruit_MAX31855 cardExt12(CLK1, Ext12Pin, MISO1);

Adafruit_MAX31855 cardTempBeforeInt(CLK1, TempBeforeIntPin, MISO1);
Adafruit_MAX31855 cardTempAfterInt(CLK1, TempAfterIntPin, MISO1);

Adafruit_MAX31865 cardWaterTemp = Adafruit_MAX31865(WaterTempPin, MOSI1, MISO1, CLK1);
Adafruit_MAX31865 cardOilTemp = Adafruit_MAX31865(OilTempPin, MOSI1, MISO1, CLK1);

//DS1302 rtc(DS1302rstPin, DS1302datPin, DS1302clkPin); // hardware timer (clock)  RST, DAT, CLK
File fileExhaust;
File fileMiscTemp;
File filePressure;

#define nexSerial Serial3

// Page 0                nex, page, id, name
NexNumber nPressureTurbo(0, 12, "x0");
NexNumber nPressureFuel(0, 13, "x1");
NexNumber nPressureOilEngine(0, 14, "x2");
NexNumber nRPM(0, 11, "n3");
NexGauge gPressureTurbo(0, 6, "z0");
NexGauge gPressureFuel(0, 7, "z1");
NexGauge gPressureOilEngine(0, 8, "z2");
NexGauge gTempWater(0, 9, "z3");
NexGauge gTempOil(0, 10, "z4");

// Page 1                nex, page, id, name
NexNumber nPressureBeforeInt(1, 11, "x0");
NexNumber nPressureManifoil(1, 12, "x1");
NexNumber nPressureAfterInt(1, 13, "x2");
NexGauge gT36(1, 9, "z8");
NexGauge gT37(1, 10, "z9");

// Page 2                nex, page, id, name
NexNumber nExt1(2, 9, "n7");
NexNumber nExt2(2, 10, "n8");
NexNumber nExt3(2, 11, "n9");
NexNumber nExt4(2, 12, "n10");
NexNumber nExt5(2, 13, "n11");
NexNumber nExt6(2, 14, "n12");
NexGauge gExt1(2, 3, "z10");
NexGauge gExt2(2, 4, "z11");
NexGauge gExt3(2, 5, "z12");
NexGauge gExt4(2, 6, "z13");
NexGauge gExt5(2, 7, "z14");
NexGauge gExt6(2, 8, "z15");

// Page 3                nex, page, id, name
NexNumber nMaxExt1(3, 36, "n26");
NexNumber nMaxExt2(3, 37, "n27");
NexNumber nMaxExt3(3, 38, "n28");
NexNumber nMaxExt4(3, 39, "n29");
NexNumber nMaxExt5(3, 40, "n30");
NexNumber nMaxExt6(3, 41, "n31");
NexNumber nMaxPressureOil(3, 24, "n14");
NexNumber nMaxTempOil(3, 27, "n17");
NexNumber nMaxPressureFuel(3, 25, "n15");
NexNumber nMaxTempWater(3, 29, "n19");
NexNumber nMaxPressureBeforeInt(3, 30, "n20");
NexNumber nMaxTempBeforeInt(3, 33, "n23");
NexNumber nMaxPressureAfterInt(3, 31, "n21");
NexNumber nMaxTempAfterInt(3, 34, "n24");
NexNumber nMaxPressureManifold(3, 32, "n22");

NexButton bResetMax(3, 5, "b8");



NexTouch *nex_listen_list[] = {&bResetMax, NULL};

//---------------------------------------------------------------------------------
// Arduino Lifecycle
//---------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  nexInit(115200);

  setupHardwareTime();

  bResetMax.attachPush(bResetMaxPushCallback);
  pinMode(logBTNpin, INPUT_PULLUP);

  cardWaterTemp.begin(MAX31865_3WIRE);
  cardOilTemp.begin(MAX31865_3WIRE);
  //  cardTempBeforeInt.begin(MAX31865_3WIRE);
  //  cardTempAfterInt.begin(MAX31865_3WIRE);

  if (SHOULD_PRINT_TO_MONITOR)
  {
    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Started: ");
    Serial.println(getTimestamp());
    Serial.println("-------------------------------------------------------------------------------");
    Serial.println();
  }

  setupSDCard();
}

float valToAngle(float val, float lower, float upper)   
{
  val = val * 100;
  int  angle = map(val, (lower*100), (upper*100), 0, (216*100));
  angle = (angle/100) -18;
  return angle;
}

void setDashboardValues(double ext1, double ext2, double ext3, double ext4, double ext5, double ext6, double t36, double t37, double tempWater, double tempOil,
                        float pressureBeforeInt, float pressureAfterInt, float pressureOilEngine, float pressureOilTurbo, float pressureWaterInject,
                        float pressureManifoil, float pressureFuel, int RPM)
{
  // Page 0
  gPressureTurbo.setValue((valToAngle(pressureBeforeInt,0.00,12.00)));
  nPressureTurbo.setValue(pressureBeforeInt*10);
  gPressureFuel.setValue((valToAngle(pressureFuel,0.00,6.00)));
  nPressureFuel.setValue(pressureFuel*10);
  gPressureOilEngine.setValue((valToAngle(pressureOilEngine,0.00,12.00)));
  nPressureOilEngine.setValue(pressureOilEngine*10);
  gTempWater.setValue((valToAngle(tempWater,0.00,150.00)));
  nRPM.setValue(RPM);
  gTempOil.setValue((valToAngle(tempOil,0.00,150.00)));
  
  nExt1.setValue(ext1);
  nExt2.setValue(ext2);
  nExt3.setValue(ext3);
  nExt4.setValue(ext4);
  nExt5.setValue(ext5);
  nExt6.setValue(ext6);

  if (pressureOilTurbo > maxPressureOilTurbo)
  {
    maxPressureOilTurbo = pressureOilTurbo;
    nMaxPressureOil.setValue(pressureOilTurbo);
  }
  else if (tempOil > maxOilTemp)
  {
    maxOilTemp = tempOil;
    nMaxTempOil.setValue(tempOil);
  }

  // gPressureBeforeInt.setValue(pressureToAngle(pressureBeforeInt * 50));

  //  dtostrf(val, width, precision, buffer);

//  char buffer[10];
//
//  dtostrf(pressureOilTurbo, 5, 2, buffer);
//  tMaxPressureOil.setText(buffer);
//
//  dtostrf(pressureFuel, 5, 2, buffer);
//  tMaxPressureFuel.setText(buffer);
//
//  dtostrf(pressureBeforeInt, 5, 2, buffer);
//  tMaxPressureBeforeInt.setText(buffer);
//
//  dtostrf(pressureAfterInt, 5, 2, buffer);
//  tMaxPressureAfterInt.setText(buffer);
//
//  dtostrf(pressureManifoil, 5, 2, buffer);
//  tMaxPressureManifold.setText(buffer);
}

void loop()
{
  double ext1 = cardExt1.readCelsius();
  double ext2 = cardExt2.readCelsius();
  double ext3 = cardExt3.readCelsius();
  double ext4 = cardExt4.readCelsius();
  double ext5 = cardExt5.readCelsius();
  double ext6 = cardExt6.readCelsius();

  double t36 = cardTempBeforeInt.readCelsius();
  double t37 = cardTempAfterInt.readCelsius();

  //  double tempBeforeInt = cardTempBeforeInt.temperature(R_NOMINAL, R_REF);
  //  double tempAfterInt = cardTempAfterInt.temperature(R_NOMINAL, R_REF);
  double tempWater = cardWaterTemp.temperature(R_NOMINAL, R_REF);
  double tempOil = cardWaterTemp.temperature(R_NOMINAL, R_REF);

  // pressure
  float pressureBeforeInt = getPressureInBar(pressureBeforeIntPin, PRESSURE_REF_200_PSI);
  float pressureAfterInt = getPressureInBar(pressureAfterIntPin, PRESSURE_REF_200_PSI);
  float pressureOilEngine = getPressureInBar(pressureOilEnginePin, PRESSURE_REF_200_PSI);
  float pressureOilTurbo = getPressureInBar(pressureOilTurboPin, PRESSURE_REF_200_PSI);
  float pressureWaterInject = getPressureInBar(pressureWaterInjectPin, PRESSURE_REF_1600_PSI);
  float pressureManifoil = getPressureInBar(pressureManifoilPin, PRESSURE_REF_200_PSI);
  float pressureFuel = getPressureInBar(pressureFuelPin, PRESSURE_REF_200_PSI);
  int RPM;

  setDashboardValues(ext1, ext2, ext3, ext4, ext5, ext6, t36, t37, tempWater, tempOil, pressureBeforeInt,
                     pressureAfterInt, pressureOilEngine, pressureOilTurbo, pressureWaterInject,
                     pressureManifoil, pressureFuel, RPM);
  Serial.println(shouldLogToSDCard);
  if (shouldLogToSDCard == true)
  {
    writeExhaustTemperatures(ext1, ext2, ext3, ext4, ext5, ext6);
    writeMiscTemperatures(tempWater, tempOil, t36, t37);
    writePressure(pressureBeforeInt, pressureAfterInt, pressureOilEngine, pressureOilTurbo, pressureWaterInject, pressureManifoil, pressureFuel);
  }

  if (SHOULD_PRINT_TO_MONITOR)
  {
    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Time: ");
    Serial.println(getTimestamp());
    Serial.println("-------------------------------------------------------------------------------");

    Serial.print("ext1: " + String(ext1) + "C");
    Serial.print(" | ext2: " + String(ext2) + "C");
    Serial.print(" | ext3: " + String(ext3) + "C");
    Serial.print(" | ext4: " + String(ext4) + "C");
    Serial.print(" | ext5: " + String(ext5) + "C");
    Serial.println(" | ext6: " + String(ext6) + "C");

    //    Serial.println("-------------------------------------------------------------------------------");
    //    Serial.print("Before int: " + String(t36) + "C");
    //    Serial.print(" | After int: " + String(t37) + "C");
    //    Serial.print(" | Water: " + String(tempWater) + "C");
    //    Serial.println(" | Oil: " + String(tempOil) + "C");

    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("Before int: " + String(pressureBeforeInt) + "bar");
    Serial.print(" | After int: " + String(pressureAfterInt) + "bar");
    Serial.println(" | Oil eng: " + String(pressureOilEngine) + "bar");
    //    Serial.println(" | Oil tur: " + String(pressureOilTurbo) + "bar");
    //    Serial.print(" | Water inject: " + String(pressureWaterInject) + "bar");
    //    Serial.print(" | Manifoil: " + String(pressureManifoil) + "bar");
    //    Serial.println(" | Fuel: " + String(pressureFuel) + "bar");
    Serial.println("-------------------------------------------------------------------------------");
    Serial.println();
  }

if(digitalRead(logBTNpin) == LOW){
  shouldLogToSDCard = true;
}
else{
  shouldLogToSDCard = false;
}

  nexLoop(nex_listen_list);

  // delay(DELAY_IN_MILLIS);
  
}

//---------------------------------------------------------------------------------
// RTC Timer
//---------------------------------------------------------------------------------
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//---------------------------------------------------------------------------------
// Nextion callbacks
//---------------------------------------------------------------------------------
void bResetMaxPushCallback(void *ptr)
{
  Serial.println("BUTTON PRESS");
  maxValues[0] = 0;
  maxPressureOilTurbo = 0;
  maxOilTemp = 0;
  nMaxPressureOil.setValue(0);
  nMaxTempOil.setValue(0);
}

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
  unsigned long t = now();
  return String(String(day(t)) + "/" + String(month(t)) + "/" + String(year(t)) + " " + String(hour(t)) + ":" + String(minute(t)) + ":" + String(second(t)));
}

//---------------------------------------------------------------------------------
// SD card
//---------------------------------------------------------------------------------

void writeExhaustTemperatures(double ext1, double ext2, double ext3, double ext4, double ext5, double ext6)
{
  fileExhaust = SD.open("exhaust.txt", FILE_WRITE);
  Serial.println(fileExhaust);
  if (fileExhaust)
  {
    fileExhaust.println(getTimestamp() + "," + String(ext1) + "," + String(ext2) + "," + String(ext3) + "," + String(ext4) + "," + String(ext5) + "," + String(ext6));
    fileExhaust.close();
    Serial.println("Written Exhaust values");
  }
  else{
    Serial.println("Problem Writing Exhaust values");
  }

  //  file = SD.open(FILE_NAME); // open for read;
  //  while (file.available()) {
  //    Serial.write(file.read());
  //  }
}

void writeMiscTemperatures(double water, double oil, double beforeInt, double afterInt)
{
  fileMiscTemp = SD.open("misc.txt", FILE_WRITE);

  if (fileMiscTemp)
  {
    fileMiscTemp.println(getTimestamp() + "," + String(water) + "," + String(oil) + "," + String(beforeInt) + "," + String(afterInt));
    fileMiscTemp.close();
    Serial.println("Written Misc values");
  }
  else{
    Serial.println("Problem Writing Misc values");
  }
}

void writePressure(double beforeInt, double afterInt, double oilEngine, double oilTurbo, double waterInject, double manifoil, double fuel)
{
  filePressure = SD.open("pressure.txt", FILE_WRITE);

  if (filePressure)
  {
    filePressure.println(getTimestamp() + "," + String(beforeInt) + "," + String(afterInt) + "," + String(oilEngine) + "," + String(oilTurbo) + "," + String(waterInject) + "," + String(manifoil) + "," + String(fuel));
    filePressure.close();
    Serial.println("Written Pressure values");
  }
  else{
    Serial.println("Problem Writing Pressure values");
  }
}

void setupSDCard()
{
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("Initialization of SD card failed!");
    while (1); // halt program (infinite loop)
  }
  else
  {
    Serial.println("SD card initialization complete.");
  }

  // Create files: open and immediately close.
  fileExhaust = SD.open("exhaust.txt", FILE_WRITE);
  fileExhaust.println();
  fileExhaust.close();

  fileMiscTemp = SD.open("misc.txt", FILE_WRITE);
  fileMiscTemp.println();
  fileMiscTemp.close();

  filePressure = SD.open("pressure.txt", FILE_WRITE);
  filePressure.println();
  filePressure.close();
}

//---------------------------------------------------------------------------------
// Date and time
//---------------------------------------------------------------------------------

void setupHardwareTime()
{
  setSyncProvider(getTeensy3Time);  // set the Time library to use Teensy 3.0's RTC to keep time
  return;

  // Set the clock to run-mode, and disable the write protection
  setTime(23, 49, 00, 07, 06, 2020);
}
