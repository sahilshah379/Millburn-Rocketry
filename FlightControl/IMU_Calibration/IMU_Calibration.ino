#include "MPU9250.h"
#include "EEPROM.h"

MPU9250 IMU(Wire,0x68);
int status;

uint8_t EepromBuffer[48];
float value;

void setup() {
    Serial.begin(115200);
    while(!Serial) {}

    status = IMU.begin();
    if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
    }
    
    Serial.println("Starting Accelerometer Calibration");
    IMU.calibrateAccel();
    Serial.println("Switch");
    delay(5000);
    IMU.calibrateAccel();
    Serial.println("Switch");
    delay(5000);
    IMU.calibrateAccel();
    Serial.println("Switch");
    delay(5000);
    IMU.calibrateAccel();
    Serial.println("Switch");
    delay(5000);
    IMU.calibrateAccel();
    Serial.println("Switch");
    delay(5000);
    IMU.calibrateAccel();
    Serial.println("Done");
    Serial.println("Starting Magnetometer Calibration");
    delay(5000);
    
    IMU.calibrateMag();

    value = IMU.getAccelBiasX_mss();
    memcpy(EepromBuffer,&value,4);
    value = IMU.getAccelScaleFactorX();
    memcpy(EepromBuffer+4,&value,4);
    value = IMU.getAccelBiasY_mss();
    memcpy(EepromBuffer+8,&value,4);
    value = IMU.getAccelScaleFactorY();
    memcpy(EepromBuffer+12,&value,4);
    value = IMU.getAccelBiasZ_mss();
    memcpy(EepromBuffer+16,&value,4);
    value = IMU.getAccelScaleFactorZ();
    memcpy(EepromBuffer+20,&value,4);
    value = IMU.getMagBiasX_uT();
    memcpy(EepromBuffer+24,&value,4);
    value = IMU.getMagScaleFactorX();
    memcpy(EepromBuffer+28,&value,4);
    value = IMU.getMagBiasY_uT();
    memcpy(EepromBuffer+32,&value,4);
    value = IMU.getMagScaleFactorY();
    memcpy(EepromBuffer+36,&value,4);
    value = IMU.getMagBiasZ_uT();
    memcpy(EepromBuffer+40,&value,4);
    value = IMU.getMagScaleFactorZ();
    memcpy(EepromBuffer+44,&value,4);
    for (size_t i=0; i < sizeof(EepromBuffer); i++) {
        EEPROM.write(i,EepromBuffer[i]);
    }
    Serial.println("Done");
}

void loop() {}
