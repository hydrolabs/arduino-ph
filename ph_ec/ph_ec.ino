#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // knihovna z https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

//////////////////// nastaveni ////////////////////

const byte phSensorPin = A1;
const byte ecSensorPin = A0;
const byte ds18b20Pin = 10; //1-wire temp sensor

const byte btnPinOK = 4;
const byte btnPinCalPH = 2;
const byte btnPinCalEC = 5;
const byte btnPinCancel = 3;

const unsigned voltageReferenceMiliVolts = 5000;
const unsigned printIntervalMilliSeconds = 500;

unsigned long printTimestamp;

//////////////////// stav programu

enum deviceState {
  stateMeasuring = 1,
  stateCalibratingPH = 2,
  stateCalibratingEC = 3,
} deviceState = stateMeasuring;

enum buttonPressed {
  btnNone,
  btnOK,
  btnCancel,
  btnCalibratePH,
  btnCalibrateEC,
} buttonPressed = btnNone;

unsigned long debounceTimestamp;
const unsigned debounceInterval = 50;

//////////////////// promenne k ph ////////////////////

enum phCalibrationState {
  phcsDone,
  phcsUse7,
  phcsCalibrating7,
  phcsUse4,
  phcsCalibrating4,
} phCalibrationState;

const unsigned phSampleCount = 30;           // sum of sample point
int phSampleBuffer[phSampleCount];    //store the sample voltage
int phSampleBufferIndex = 0;

const unsigned phEepromSlopeAddress = 0;     // (slope of the ph probe)store at the beginning of the EEPROM. The slope is a float number,occupies 4 bytes.
const unsigned phEepromInterceptAddress = phEepromSlopeAddress+4; //phEepromSlopeAddress+sizeof(float)?
float phSlope = 3.5;
float phIntercept = 0.0;
float phAverageVoltage; //FIXME proc je to float?

int phCalVoltage7;
int phCalVoltage4;
const unsigned phCalibrationWaitInterval = 15 * 1000; //wait 15 seconds before reading buffer solution voltage
unsigned long phCalibrationWaitStart;

const unsigned phSampleInterval = 40;
unsigned long phSampleTimestamp;


//////////////////// promenne k ec+temp ////////////////////
enum {
  tempCommandStartConvert = 0,
  tempCommandReadTemperature = 1,
};

enum ecCalibrationState {
  eccsDone,
  eccsUseA,
  eccsCalibratingA,
  eccsUseB,
  eccsCalibratingB,
} ecCalibrationState;

const byte ecSampleCount = 20;     //the number of sample times
unsigned int ecSampleBuffer[ecSampleCount];      // the readings from the analog input
byte ecSampleBufferIndex = 0;                  // the index of the current reading
unsigned long ecAnalogValueTotal = 0;                  // the running total
unsigned int ecAnalogAverage = 0, ecAverageVoltage = 0;             // the average
float tempCurrent, ecCurrent;

const unsigned ecSampleInterval = 25;
const unsigned tempSampleInterval = 850;
unsigned long ecSampleTimestamp, tempSampleTime;

const unsigned ecEepromSlopeAddress = phEepromInterceptAddress+4;
const unsigned ecEepromInterceptAddress = ecEepromSlopeAddress+4;
const unsigned ecEepromCalibratedTempAddress = ecEepromInterceptAddress+4;
float ecSlope = 6.84;
float ecIntercept = -64.32;
float ecCalibratedTemp = 25.0;

const unsigned ecCalConductivityA = 1413;  // us/cm
const unsigned ecCalConductivityB = 12880; // us/cm
int ecCalVoltageA;
int ecCalVoltageB;
const unsigned ecCalibrationWaitInterval = 15 * 1000;
unsigned long ecCalibrationWaitStart;

//Temperature chip i/o
OneWire tempSensor(ds18b20Pin);

// vytvoří objekt lcd a nastaví jeho adresu
// 0x20 a 16 zanků na 2 řádcích
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);

//////////////////// funkce k ph ////////////////////

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void SamplePH() {
  phSampleBuffer[phSampleBufferIndex] = analogRead(phSensorPin) / 1024.0 * voltageReferenceMiliVolts; //read the voltage and store into the buffer,every 40ms
  phSampleBufferIndex++;
  if (phSampleBufferIndex == phSampleCount) {
    phSampleBufferIndex = 0;
  }
  phAverageVoltage = getMedianNum(phSampleBuffer, phSampleCount);  // read the stable value by the median filtering algorithm
}

void SetupPH()
{
    EEPROM_read(phEepromSlopeAddress, phSlope);
    EEPROM_read(phEepromInterceptAddress, phIntercept);
    if(EEPROM.read(phEepromSlopeAddress)==0xFF && EEPROM.read(phEepromSlopeAddress+1)==0xFF && EEPROM.read(phEepromSlopeAddress+2)==0xFF && EEPROM.read(phEepromSlopeAddress+3)==0xFF)
    {
      phSlope = 3.5; //but why?
      EEPROM_write(phEepromSlopeAddress, phSlope);
    }
    if(EEPROM.read(phEepromInterceptAddress)==0xFF && EEPROM.read(phEepromInterceptAddress+1)==0xFF && EEPROM.read(phEepromInterceptAddress+2)==0xFF && EEPROM.read(phEepromInterceptAddress+3)==0xFF)
    {
      phIntercept = 0;
      EEPROM_write(phEepromInterceptAddress, phIntercept);
    }
}

void PHCalibrationRefresh()
{
  switch(phCalibrationState)
  {
  case phcsUse7:
    lcd.clear();
    lcd.print("put into pH=7");
    lcd.setCursor(0, 1);
    lcd.print("and press OK");
    break;
  case phcsCalibrating7:
    if (waited_for(phCalibrationWaitInterval, &phCalibrationWaitStart))
    {
      phCalVoltage7 = phAverageVoltage;
      phCalibrationState = phcsUse4;
    }
    else
    {
      lcd.clear();
      lcd.print("U=");
      lcd.print(phAverageVoltage);
      lcd.print(" mV");
      lcd.setCursor(0, 1);
      lcd.print("wait ...");
    }
    break;
  case phcsUse4:
    lcd.clear();
    lcd.print("put into pH=4");
    lcd.setCursor(0, 1);
    lcd.print("and press OK");
    break;
  case phcsCalibrating4:
    if (waited_for(phCalibrationWaitInterval, &phCalibrationWaitStart))
    {
      phCalVoltage4 = phAverageVoltage;

      // spocitat parametry a ulozit do pameti
      phSlope = -3.0 / ((phCalVoltage4 - phCalVoltage7)/1000.0); //(4 - 7) / (U_4 - U_7)
      phIntercept = 7 - phSlope * (phCalVoltage7/1000.0);
      EEPROM_write(phEepromSlopeAddress, phSlope);
      EEPROM_write(phEepromInterceptAddress, phIntercept);

      phCalibrationState = phcsDone;
      phCalibrationWaitStart = millis();
    }
    else
    {
      lcd.clear();
      lcd.print("U=");
      lcd.print(phAverageVoltage);
      lcd.print(" mV");
      lcd.setCursor(0, 1);
      lcd.print("wait ...");
    }
    break;
  case phcsDone:
    // po pul seknude se vratit to rezimu mereni
    if (waited_for(2500, &phCalibrationWaitStart))
    {
      deviceState = stateMeasuring;
    }
    else
    {
      lcd.clear();
      lcd.print("Calibration done");
      lcd.setCursor(0, 1);
      lcd.print("sl: ");
      lcd.print(phSlope);
      lcd.print(" in: ");
      lcd.print(phIntercept);
    }
    break;
  }
}

void PHCalibrationButtonPressed(int btn) //why the FUCK can't i use enum buttonPressed instead of int?
{
  if (btn == btnCancel)
  {
    deviceState = stateMeasuring;
    return;
  }

  if (btn != btnOK)
  {
    return;
  }

  switch(phCalibrationState)
  {
  case phcsUse7:
    phCalibrationState = phcsCalibrating7;
    phCalibrationWaitStart = millis();
    break;
  case phcsUse4:
    phCalibrationState = phcsCalibrating4;
    phCalibrationWaitStart = millis();
    break;
  }
}

//////////////////// funkce k ec+temp ////////////////////
void SetupEC() {
  for (byte thisReading = 0; thisReading < ecSampleCount; thisReading++)
    ecSampleBuffer[thisReading] = 0;
  TempProcess(tempCommandStartConvert);   //let the DS18B20 start the convert
}

void SampleEC() {    // subtract the last reading:
  ecAnalogValueTotal = ecAnalogValueTotal - ecSampleBuffer[ecSampleBufferIndex];
  // read from the sensor:
  ecSampleBuffer[ecSampleBufferIndex] = analogRead(ecSensorPin);
  Serial.println(ecSampleBuffer[ecSampleBufferIndex]);
  // add the reading to the total:
  ecAnalogValueTotal = ecAnalogValueTotal + ecSampleBuffer[ecSampleBufferIndex];
  // advance to the next position in the array:
  ecSampleBufferIndex = ecSampleBufferIndex + 1;
  // if we're at the end of the array...
  if (ecSampleBufferIndex >= ecSampleCount)
    // ...wrap around to the beginning:
    ecSampleBufferIndex = 0;
  // calculate the average:
  ecAnalogAverage = ecAnalogValueTotal / ecSampleCount;
  ecAverageVoltage = ecAnalogAverage * (float)5000 / 1024;
}

void SampleTemp() {
  tempCurrent = TempProcess(tempCommandReadTemperature);  // read the current temperature from the  DS18B20
  TempProcess(tempCommandStartConvert);                   //after the reading,start the convert for next reading
}

void ComputeEC() {
  float TempCoefficient = 1.0 + 0.0185 * (tempCurrent - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
  float CoefficientVolatge = (float)ecAverageVoltage / TempCoefficient;
  if (CoefficientVolatge < 150)Serial.println("No solution!"); //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
  else if (CoefficientVolatge > 3300)Serial.println("Out of the range!"); //>20ms/cm,out of the range
  else
  {
    if (CoefficientVolatge <= 448)ecCurrent = 6.84 * CoefficientVolatge - 64.32; //1ms/cm<EC<=3ms/cm
    else if (CoefficientVolatge <= 1457)ecCurrent = 6.98 * CoefficientVolatge - 127; //3ms/cm<EC<=10ms/cm
    else ecCurrent = 5.3 * CoefficientVolatge + 2278;                     //10ms/cm<EC<20ms/cm
    //ecCurrent /= 1000;  //convert us/cm to ms/cm
    Serial.print(ecCurrent, 2); //two decimal
    Serial.println("us/cm");
  }
}

/*
  ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess(int command)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if (command == tempCommandStartConvert) {
    if ( !tempSensor.search(addr)) {
      Serial.println("no more sensors on chain, reset search!");
      tempSensor.reset_search();
      return 0;
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0;
    }
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized!");
      return 0;
    }
    tempSensor.reset();
    tempSensor.select(addr);
    tempSensor.write(0x44, 1); // start conversion, with parasite power on at the end
  }
  else if(command == tempCommandReadTemperature) {
    byte present = tempSensor.reset();
    tempSensor.select(addr);
    tempSensor.write(0xBE); // Read Scratchpad
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = tempSensor.read();
    }
    tempSensor.reset_search();
    byte MSB = data[1];
    byte LSB = data[0];
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    TemperatureSum = tempRead / 16;
  } else {
    Serial.println("Unknown command!");
  }
  return TemperatureSum;
}

void ECCalibrationRefresh()
{
  switch(ecCalibrationState)
  {
  case eccsUseA:
    lcd.clear();
    lcd.print("put into ");
    lcd.print(ecCalConductivityA);
    lcd.print("us");
    lcd.setCursor(0, 1);
    lcd.print("and press OK");
    break;
  case eccsCalibratingA:
    if (waited_for(ecCalibrationWaitInterval, &ecCalibrationWaitStart))
    {
      ecCalVoltageA = ecAverageVoltage;
      ecCalibrationState = eccsUseB;
    }
    else
    {
      lcd.clear();
      lcd.print("U=");
      lcd.print(ecAverageVoltage);
      lcd.print(" mV");
      lcd.setCursor(0, 1);
      lcd.print("wait ...");
    }
    break;
  case eccsUseB:
    lcd.clear();
    lcd.print("put into ");
    lcd.print(ecCalConductivityB);
    lcd.print("us");
    lcd.setCursor(0, 1);
    lcd.print("and press OK");
    break;
  case eccsCalibratingB:
    if (waited_for(ecCalibrationWaitInterval, &ecCalibrationWaitStart))
    {
      ecCalVoltageB = ecAverageVoltage;

      // spocitat parametry a ulozit do pameti
      ecSlope = (ecCalConductivityB - ecCalConductivityA) / ((ecCalVoltageB - ecCalVoltageA)/1000.0);
      ecIntercept = ecCalConductivityA - ecSlope * (ecCalVoltageA/1000.0);
      ecCalibratedTemp = tempCurrent;
      EEPROM_write(ecEepromSlopeAddress, ecSlope);
      EEPROM_write(ecEepromInterceptAddress, ecIntercept);
      EEPROM_write(ecEepromCalibratedTempAddress, ecCalibratedTemp);

      ecCalibrationState = eccsDone;
      ecCalibrationWaitStart = millis();
    }
    else
    {
      lcd.clear();
      lcd.print("U=");
      lcd.print(ecAverageVoltage);
      lcd.print(" mV");
      lcd.setCursor(0, 1);
      lcd.print("wait ...");
    }
    break;
  case eccsDone:
    // po pul seknude se vratit to rezimu mereni
    if (waited_for(2500, &ecCalibrationWaitStart))
    {
      deviceState = stateMeasuring;
    }
    else
    {
      lcd.clear();
      lcd.print("Calibration done");
      lcd.setCursor(0, 1);
      lcd.print("sl: ");
      lcd.print(ecSlope);
      lcd.print(" in: ");
      lcd.print(ecIntercept);
      lcd.print(" t: ");
      lcd.print(ecCalibratedTemp);
    }
    break;
  }
}

void ECCalibrationButtonPressed(int btn)
{
  if (btn == btnCancel)
  {
    deviceState = stateMeasuring;
    return;
  }

  if (btn != btnOK)
  {
    return;
  }

  switch(ecCalibrationState)
  {
  case eccsUseA:
    ecCalibrationState = eccsCalibratingA;
    ecCalibrationWaitStart = millis();
    break;
  case eccsUseB:
    ecCalibrationState = eccsCalibratingB;
    ecCalibrationWaitStart = millis();
    break;
  }
}

//////////////////// dalsi funkce ////////////////////
void PrintValues() {
  float phCurrent = phAverageVoltage / 1000.0 * phSlope + phIntercept;
  ComputeEC();
  
  Serial.print("Voltage:");
  Serial.print(phAverageVoltage);
  Serial.println("mV");

  Serial.print("pH:");              // in normal mode, print the ph value to user
  Serial.println(phCurrent);

  lcd.clear();
  lcd.print("pH");
  lcd.print(phCurrent);
  lcd.print(" t=");
  lcd.print(tempCurrent);
  lcd.setCursor(0, 1);
  lcd.print("EC");
  lcd.print(ecCurrent);
  lcd.print("uS/cm");
}


bool waited_for(const unsigned interval, unsigned long *timestamp)
{
  if (*timestamp == 0)
  {
    *timestamp = millis();
  }

  if (millis() - *timestamp >= interval)
  {
    *timestamp = millis();
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16,2);
  lcd.backlight();
  pinMode(btnPinOK, INPUT_PULLUP);
  pinMode(btnPinCalPH, INPUT_PULLUP);
  pinMode(btnPinCalEC, INPUT_PULLUP);
  pinMode(btnPinCancel, INPUT_PULLUP);
  ///////////////////// inicializovat ph /////////////////////
  SetupPH();

  ///////////////////// inicializovat ec+teplotu /////////////////////
  SetupEC();
}


void loop() {
  //////////////////// nacist ph ////////////////////
  if(waited_for(phSampleInterval, &phSampleTimestamp))
  {
    SamplePH();
  }

  //////////////////// nacist ec+teplotu ////////////////////
  /*
    Every once in a while,sample the analog value and calculate the average.
  */
  if(waited_for(ecSampleInterval, &ecSampleTimestamp))
  {
    SampleEC();
  }
  /*
    Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
    Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
  if(waited_for(tempSampleInterval, &tempSampleTime))
  {
    SampleTemp();
  }

  buttonPressed = btnNone;
  // TODO zjistit jestli bylo stisknuto tlacitko a ulozit do buttonPressed
  if (digitalRead(btnPinOK) == LOW) {
    buttonPressed = btnOK;
  } else if (digitalRead(btnPinCalPH) == LOW) {
    buttonPressed = btnCalibratePH;
  } else if (digitalRead(btnPinCalEC) == LOW) {
    buttonPressed = btnCalibrateEC;
  } else if (digitalRead(btnPinCancel) == LOW) {
    buttonPressed = btnCancel;
  }

  if (buttonPressed != btnNone)
  {
    switch(deviceState)
    {
    case stateMeasuring:
      if (buttonPressed == btnCalibratePH) {
        deviceState = stateCalibratingPH;
        phCalibrationState = phcsUse7;
        PHCalibrationRefresh();
      } else if (buttonPressed == btnCalibrateEC) {
        deviceState = stateCalibratingEC;
        ecCalibrationState = eccsUseA;
        ECCalibrationRefresh();
      }
      break;
    case stateCalibratingPH:
      PHCalibrationButtonPressed(buttonPressed);
      break;
    case stateCalibratingEC:
      ECCalibrationButtonPressed(buttonPressed);
      break;
    }
  }
  else
  {
    switch(deviceState)
    {
    case stateMeasuring:
      //////////////////// vypsat na seriak ////////////////////
      if(waited_for(printIntervalMilliSeconds, &printTimestamp))
      {
        PrintValues();
      }
      break;
    case stateCalibratingPH:
      if(waited_for(printIntervalMilliSeconds, &printTimestamp))
      {
        PHCalibrationRefresh();
      }
      break;
    case stateCalibratingEC:
      if(waited_for(printIntervalMilliSeconds, &printTimestamp))
      {
        ECCalibrationRefresh();
      }
      break;
    }
  }
}
