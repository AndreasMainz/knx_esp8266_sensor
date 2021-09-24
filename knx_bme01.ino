// Sendet die Temp, Feuchte und Luftdruck alle 11 sec auf 3 verschiedenen GA's
// Sendeinterval wird mit Delay_Mask eingestellt

#include <Wire.h>
#include "knx_bme.h"
#include <ESP8266WebServer.h>
#include <KonnektingDevice.h>
#define KNX_SERIAL  Serial  // swaped Serial on D7(GPIO13)=RX/GPIO15(D8)=TX 19200 baud

// BME280 I2C address is 0x76(108)
#define Addr 0x76

byte b1[24];
byte data[8];
unsigned int dig_T1,dig_P1,dig_H1;
int dig_T2,dig_T3;
int dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
int dig_H2,dig_H4,dig_H5,dig_H6;
unsigned int dig_H3;

// ################################################
// ### DEBUG Configuration
// ################################################

// Define KONNEKTING Device related IDs
#define MANUFACTURER_ID 57005
#define DEVICE_ID 255
#define REVISION 0
#define KONNEKTING_SYSTEM_TYPE_SIMPLE

byte virtualEEPROM[128];

struct knx_data_out
{
  word BME_Temp;
  word BME_Hum;
  word BME_Pres;
  
};
knx_data_out Knx_Out;

unsigned char *p;
const byte Obj_out = sizeof (Knx_Out) / 2; // 2 bytes for 1 GA

word individualAddress = P_ADDR(1, 2, 116);

// Definition of the Communication Objects attached to the device
KnxComObject KnxDevice::_comObjectsList[] = {
  /* Input */
  /* Suite-Index 0 : Temperatur*/       KnxComObject(KNX_DPT_9_001, COM_OBJ_SENSOR),
  /* Suite-Index 1 : Humitity  */       KnxComObject(KNX_DPT_9_007, COM_OBJ_SENSOR),
  /* Suite-Index 2 : Luftdruck */       KnxComObject(KNX_DPT_9_006, COM_OBJ_SENSOR)
};
const byte KnxDevice::_numberOfComObjects = sizeof (_comObjectsList) / sizeof (KnxComObject); // do no change this code
// Definition of parameter size
byte KonnektingDevice::_paramSizeList[] = {
  /* Param Index 0 */ PARAM_UINT16
};
const int KonnektingDevice::_numberOfParams = sizeof (_paramSizeList); // do no change this code

void Knx_Init(void) {
  Knx_Out.BME_Temp    = G_ADDR(1, 5, 9);
  Knx_Out.BME_Hum     = G_ADDR(1, 5, 10);
  Knx_Out.BME_Pres    = G_ADDR(1, 5, 11);
};

//we do not need a ProgLED, but this function muss be defined
void progLed (bool state) {}; //nothing to do here

void knxEvents(byte index) {/*nothing to receive */};

byte readMemory(int index) {
  return virtualEEPROM[index];
}
void writeMemory(int index, byte val) {
  virtualEEPROM[index] = val;
}
void updateMemory(int index, byte val) {
  if (readMemory(index) != val) {
    writeMemory(index, val);
  }
}
void commitMemory() {
}



void setup()
{
  WiFi.forceSleepBegin();  // WLan aus
  delay(5000); //Warten auf Supercap
  //////////////////////////////////////////// KNX Part /////////////////////////////////////////
  memset(virtualEEPROM, 0xFF, 128); // check data size of Knx Struct!!

  Konnekting.setMemoryReadFunc(&readMemory);
  Konnekting.setMemoryWriteFunc(&writeMemory);
  Konnekting.setMemoryUpdateFunc(&updateMemory);
  Konnekting.setMemoryCommitFunc(&commitMemory);

  Knx_Init(); // Write GA's to RAM
  /******************************************************* PA ************************************************/
  // write hardcoded PA and GAs
  writeMemory(0,  0x7F);  //Set "not factory" flag
  writeMemory(1,  (byte)(individualAddress >> 8));
  writeMemory(2,  (byte)(individualAddress));
  /******************************************************* OUT ************************************************/
  // Write all send GA to Eeprom
  p = (unsigned char*)&Knx_Out;
  for (int index = 0; index < (Obj_out); index++) {
    virtualEEPROM[(3 * index) + 11] = *p++; // Order is fliped in data structure..
    virtualEEPROM[(3 * index) + 10] = *p++;
    virtualEEPROM[(3 * index) + 12] = 0x80;
  }

  //////////////////////////////////////////////// BME Setup ////////////////////////////////////
  // Initialise I2C communication as MASTER
  Wire.begin(12, 14);
  // Initialise Serial communication, set baud rate = 9600
  Serial.begin(115200);
  dig_H1 = 0;
  // read calibration data
  for (int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((0x88 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 24 bytes of data
    // if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
      // Serial.println(b1[i]);
    }
    //else
    //Serial.println("Not_connected");
  }

  // Convert the data
  // temp coefficients
  dig_T1 = b1[0] + (unsigned int)(b1[1] * 256);
  // unsigned int dig_T1 = (b1[0] & 0xff) + ((b1[1] & 0xff) * 256);
  dig_T2 = b1[2] + (b1[3] * 256);
  dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  dig_P1 = (b1[6] & 0xff) + (unsigned int)((b1[7] & 0xff ) * 256);
  dig_P2 = b1[8] + (int)(b1[9] * 256);
  dig_P3 = b1[10] + (int)(b1[11] * 256);
  dig_P4 = b1[12] + (int)(b1[13] * 256);
  dig_P5 = b1[14] + (int)(b1[15] * 256);
  dig_P6 = b1[16] + (int)(b1[17] * 256);
  dig_P7 = b1[18] + (int)(b1[19] * 256);
  dig_P8 = b1[20] + (int)(b1[21] * 256);
  dig_P9 = b1[22] + (int)(b1[23] * 256);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0xA1);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);

  // Read 1 byte of data
  if (Wire.available() == 1)
  {
    dig_H1 = Wire.read();
  }

  for (int i = 0; i < 7; i++)
  { //////////////////////////////// Calib26.. ///////////////////////////////
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((0xE1 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 7 bytes of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
    else Serial.print("*");
  }

  // Convert the data
  // humidity coefficients
  dig_H2 = b1[0] + (b1[1] * 256);
  dig_H3 = b1[2] & 0xFF ;
  dig_H4 = (b1[3] * 16) + (b1[4] & 0xF);
  dig_H5 = (b1[4] / 16) + (b1[5] * 16);
  dig_H6 = b1[6];

  ///////////////////////////////////////////////// Schreibe Oversampling und Einstellungen ///////////////////////
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control humidity register
  Wire.write(0xF2);
  // Humidity over sampling rate = 1
  Wire.write(0x01);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Initialize KNX
  Konnekting.init(KNX_SERIAL, &progLed, MANUFACTURER_ID, DEVICE_ID, REVISION);
  // Now serial is swiched to RX2/TX2!
}

void loop()
{
   Main_Clock++;
  /////////////////////////////////////////////////////////////////////////////////////
  Knx.task();
  /////////////////////////////////////////////////////////////////////////////////////
   if ((Main_Clock & Delay_Mask) == 0){
  /////////////////////// Lese Sensor Daten ////////////////////////
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0xF7);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 8 byte of data
  Wire.requestFrom(Addr, 8);
  for (int i = 0; i < 8; i++)
  {
    data[i] = Wire.read();
  }

  // Convert pressure and temperature data to 20-bits
  long adc_p = ((long)(data[0]<<16) | (long)(data[1]<<8) | (long)data[2])>>4;
  long adc_t = ((long)(data[3]<<16) | (long)(data[4]<<8) | (long)data[5])>>4;
  // Convert the humidity 16 Bits
  long adc_h =                        (long)(data[6]<<8) | (long)data[7];

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  double cTemp = (var1 + var2) / 5120.0;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  // Humidity offset calculations
  double var_H = (((double)t_fine) - 76800.0);
  var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
  double humidity = var_H * (1.0 - dig_H1 * var_H / 524288.0);
  (humidity > 100.0)?humidity = 100.0:(humidity < 0.0)?humidity = 0.0:humidity;

  // Output data to KNX bus
//  Serial.swap();
//  Serial.print("Temperature in Celsius : ");
//  Serial.print(cTemp);
//  Serial.println(" C");
//  Serial.print("Pressure : ");
//  Serial.print(pressure);
//  Serial.println(" hPa");
//  Serial.print("Relative Humidity : ");
//  Serial.print(humidity);
//  Serial.println(" RH");
//  Serial.swap();
  Knx.write(0,cTemp);
  Knx.write(1,humidity);
  Knx.write(2,pressure);
  }
}
