# knx_esp8266_sensor
Add cyclic Sensor messages via KNX all 11 sec

BME is connected to Pin 12, 14 with
Wire.begin(12, 14); (SDA,SCL)

// BME280 I2C address is 0x76(108)
#define Addr 0x76



