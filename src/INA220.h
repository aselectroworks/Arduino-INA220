/**************************************************************************/
/*!
  @file     INA220.h
  Author: Atsushi Sasaki(https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#ifndef _INA220_H
#define _INA220_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

#define INA220_ADDR_CONFIGURATION 0x00
#define INA220_ADDR_SHUNT_VOLTAGE 0x01
#define INA220_ADDR_BUS_VOLTAGE 0x02
#define INA220_ADDR_POWER 0x03
#define INA220_ADDR_CURRENT 0x04
#define INA220_ADDR_CALIBRATION 0x05

#define RESET_BIT 15
#define BUS_VOLTAGE_RANGE_BIT 13
#define PGA_BIT_BASE 11
typedef enum {
    PGA_GAIN_x1 = 0,
    PGA_GAIN_x2,
    PGA_GAIN_x4,
    PGA_GAIN_x8,
} PGA_GAIN_Enum;
#define BUS_ADC_RESOLUTION_AVERAGING_BIT_BASE 7
#define SHUNT_ADC_RESOLUTION_AVERAGING_BIT_BASE 3
typedef enum {
    ADC_RESOLUTION_9BITS = 0,
    ADC_RESOLUTION_10BITS,
    ADC_RESOLUTION_11BITS,
    ADC_RESOLUTION_12BITS,
    ADC_AVERAGING_2SAMPLES = 9,
    ADC_AVERAGING_4SAMPLES,
    ADC_AVERAGING_8SAMPLES,
    ADC_AVERAGING_16SAMPLES,
    ADC_AVERAGING_32SAMPLES,
    ADC_AVERAGING_64SAMPLES,
    ADC_AVERAGING_128SAMPLES,
} ADC_SETTING_Enum;
#define MODE_BIT_BASE 0x00
typedef enum {
    POWER_DOWN = 0,
    SHUNT_TRIGGERED,
    BUS_TRIGGERED,
    SHUNT_BUS_TRIGGERED,
    ADC_OFF,
    SHUNT_CONTINUOUS,
    BUS_CONTINUOUS,
    SHUNT_BUS_CONTINUOUS,
} MODE_SETTING_Enum;
typedef union {
    uint16_t raw;
    struct {
        MODE_SETTING_Enum mode : 3;
        ADC_SETTING_Enum sadc : 4;
        ADC_SETTING_Enum badc : 4;
        PGA_GAIN_Enum pg : 2;
        uint8_t brng : 1;
        uint8_t rsv : 1;
        bool rst : 1;
    };
} INA220_CONFIGURATION_REG;

// Uncomment to enable debug messages
#define INA220_DEBUG

// Define where debug output will be printed
#define DEBUG_PRINTER Serial

// Setup debug printing macros
#ifdef INA220_DEBUG
#define DEBUG_PRINT(...) \
    { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) \
    { DEBUG_PRINTER.println(__VA_ARGS__); }
#define DEBUG_PRINTF(...) \
    { DEBUG_PRINTER.printf(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
{}
#define DEBUG_PRINTLN(...)
{}
#define DEBUG_PRINTF(...)
{}
#endif

/**************************************************************************/
/*!
    @brief  INA220 Voltage/Current Meter driver
*/
/**************************************************************************/
class INA220 {
   public:
    INA220(uint8_t deviceAddress);
#ifdef ESP32 || ESP8266
    INA220(int8_t sda, int8_t scl, uint8_t deviceAddress);
#endif
    virtual ~INA220();

    void begin();
    void begin(float maxCurrent, float shuntResistance);

    void checkFlags(bool* ready, bool* overflow);

    uint16_t readVoltageRaw();
    float readVoltage();
    float readVoltage_mV();

    float readShuntVoltage();

    float readCurrent();
    float readCurrent_mA();
    float readCurrent_uA();

    float readPower();

    bool setCalibration(float maxCurrent, float shuntResistance);
    uint16_t readCalibration();

    INA220_CONFIGURATION_REG readConfig(void);
    void writeConfig(INA220_CONFIGURATION_REG conf);

    void softwareReset(void);
    void writeBusVoltageRange(uint8_t val);
    void writePGA(PGA_GAIN_Enum val);
    void writeBusAdcSetting(ADC_SETTING_Enum val);
    void writeShuntAdcSetting(ADC_SETTING_Enum val);
    void writeModeSetting(MODE_SETTING_Enum val);

   private:
    uint8_t _sda = -1;
    uint8_t _scl = -1;
    uint8_t _deviceAddress;
    float _current_LSB;
    INA220_CONFIGURATION_REG _conf = {0x399F};

    void readMultiByte(uint8_t addr, uint8_t size, uint8_t* data);
    uint16_t readWord(uint8_t addr);
    uint8_t readByte(uint8_t addr);
    void writeMultiByte(uint8_t addr, uint8_t* data, uint8_t size);
    void writeWord(uint8_t addr, uint16_t data);
    void writeByte(uint8_t addr, uint8_t data);
};

#endif