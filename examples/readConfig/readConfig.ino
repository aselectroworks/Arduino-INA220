#include <INA220.h>

#define INA220_DEV_ADDR 0x40

INA220 ina220(22, 23, INA220_DEV_ADDR); 
INA220_CONFIGURATION_REG reg; 

void setup() {
    ina220.begin(); 
    reg = ina220.readConfig(); 
    Serial.printf("Config REG: 0x%x\n", reg.raw); 
    Serial.printf(" RST: %d\n", reg.rst); 
    Serial.printf(" BRNG: %d\n", reg.brng); 
    Serial.printf(" PG: %d\n", reg.pg); 
    Serial.printf(" BADC: %d\n", reg.badc); 
    Serial.printf(" SADC: %d\n", reg.sadc); 
    Serial.printf(" MODE: %d\n", reg.mode); 
    ina220.setCalibration(0.032, 10); 
}

void loop() {
    Serial.printf("Read Bus Voltage:   %f\n", ina220.readVoltage()); 
    Serial.printf("Read Shunt Voltage: %f\n", ina220.readShuntVoltage()); 
    Serial.printf("Read Current:       %f\n", ina220.readCurrent()); 
    Serial.printf("Read Power:         %f\n", ina220.readPower()); 

    delay(1000); 
}