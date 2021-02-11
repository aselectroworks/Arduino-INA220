/**************************************************************************/
/*!
  @file     INA220.cpp
  Author: Atsushi Sasaki(https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#include "INA220.h"

#include <Wire.h>

INA220::INA220(uint8_t deviceAddress) : _deviceAddress(deviceAddress) {
#ifdef INA220_DEBUG
    DEBUG_PRINTER.begin(115200);
#endif
    DEBUG_PRINTLN("[INA220] Call Contructor");
}
#ifdef ESP32 || ESP8266
INA220::INA220(int8_t sda, int8_t scl, uint8_t deviceAddress)
    : _sda(sda), _scl(scl), _deviceAddress(deviceAddress) {
#ifdef INA220_DEBUG
    DEBUG_PRINTER.begin(115200);
#endif
    DEBUG_PRINTLN("[INA220] Call Contructor");
}
#endif

INA220::~INA220() {}

void INA220::begin() {
    DEBUG_PRINTLN("[INA220] begin");
    // Initialize I2C
    if (_sda != -1 && _scl != -1) {
        Wire.begin(_sda, _scl);
    } else {
        Wire.begin();
    }
    
    _conf = readConfig(); 
}
void INA220::begin(float maxCurrent, float shuntResistance) {
    begin();

    setCalibration(maxCurrent, shuntResistance);
}

void INA220::checkFlags(bool* ready, bool* overflow) {
    uint16_t reg;
    reg = readWord(INA220_ADDR_BUS_VOLTAGE);
    *ready = (reg & 0x02) >> 1;
    *overflow = reg & 0x01;
}

uint16_t INA220::readVoltageRaw() {
    return (readWord(INA220_ADDR_BUS_VOLTAGE) >> 3);
}

float INA220::readVoltage() {
    return (readWord(INA220_ADDR_BUS_VOLTAGE) >> 3) * 0.004;
}
float INA220::readVoltage_mV() {
    return (readWord(INA220_ADDR_BUS_VOLTAGE) >> 3) * 4;
}

float INA220::readShuntVoltage() {
    return (int16_t)(readWord(INA220_ADDR_SHUNT_VOLTAGE)) * 0.00001;
}

float INA220::readCurrent() {
    return (int16_t)(readWord(INA220_ADDR_CURRENT)) * _current_LSB;
}
float INA220::readCurrent_mA() { return readCurrent() * 1000; }
float INA220::readCurrent_uA() { return readCurrent_mA() * 1000; }

float INA220::readPower() {
    return readWord(INA220_ADDR_POWER) * _current_LSB * 20;
}

bool INA220::setCalibration(float maxCurrent, float shuntResistance) {
    _current_LSB = maxCurrent / 32767;
    uint16_t cal = (uint16_t)(0.04096 / (_current_LSB * shuntResistance));
    writeWord(INA220_ADDR_CALIBRATION, cal);
}
uint16_t INA220::readCalibration() { return readWord(INA220_ADDR_CALIBRATION); }

INA220_CONFIGURATION_REG INA220::readConfig(void) {
    _conf.raw = readWord(INA220_ADDR_CONFIGURATION);
    return _conf;
}
void INA220::writeConfig(INA220_CONFIGURATION_REG conf) {
    _conf = conf;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}

void INA220::softwareReset(void) {
    _conf.rst = 1;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
    _conf = readConfig();
}
void INA220::writeBusVoltageRange(uint8_t val) {
    _conf.brng = val;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}
void INA220::writePGA(PGA_GAIN_Enum val) {
    _conf.pg = val;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}
void INA220::writeBusAdcSetting(ADC_SETTING_Enum val) {
    _conf.badc = val;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}
void INA220::writeShuntAdcSetting(ADC_SETTING_Enum val) {
    _conf.sadc = val;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}
void INA220::writeModeSetting(MODE_SETTING_Enum val) {
    _conf.mode = val;
    writeWord(INA220_ADDR_CONFIGURATION, _conf.raw);
}

// Private Function
void INA220::readMultiByte(uint8_t addr, uint8_t size, uint8_t* data) {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom(_deviceAddress, size);
    for (uint8_t i = 0; i < size; ++i) {
        data[i] = Wire.read();
    }
}
uint16_t INA220::readWord(uint8_t addr) {
    uint8_t rxData[2];
    readMultiByte(addr, 2, rxData);
    return ((rxData[0] << 8) | rxData[1]);
}
uint8_t INA220::readByte(uint8_t addr) {
    uint8_t rxData;
    readMultiByte(addr, 1, &rxData);
    return rxData;
}
void INA220::writeMultiByte(uint8_t addr, uint8_t* data, uint8_t size) {
    Wire.beginTransmission(_deviceAddress);
    DEBUG_PRINT("writeI2C reg 0x")
    DEBUG_PRINT(addr, HEX)
    Wire.write(addr);
    for (uint8_t i = 0; i < size; i++) {
        DEBUG_PRINTF(" -> data[%d]:0x", i)
        DEBUG_PRINT(data[i], HEX)
        Wire.write(data[i]);
    }
    DEBUG_PRINTLN("");
    Wire.endTransmission();
}
void INA220::writeWord(uint8_t addr, uint16_t data) {
    uint8_t txData[2] = {data >> 8, (data & 0x00FF)};
    writeMultiByte(addr, txData, 2);
}
void INA220::writeByte(uint8_t addr, uint8_t data) {
    writeMultiByte(addr, &data, 1);
}
