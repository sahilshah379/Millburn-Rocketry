#include <MPU6050.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <SPI.h>
#include <math.h>

//const int MPU=0x68;
//int16_t ax,ay,az,gx,gy,gz;

void setup(){
    Serial.begin(9600);
    while (!Serial) {
        ;
    }
//    Wire.begin();
//    Wire.beginTransmission(MPU);
//    Wire.write(0x6B);
//    Wire.write(0);
//    Wire.endTransmission(true);
}

void loop(){
//    Wire.beginTransmission(MPU);
//    Wire.write(0x3B);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU,12,true);
//    ax = Wire.read()<<8|Wire.read();
//    ay = Wire.read()<<8|Wire.read();
//    az = Wire.read()<<8|Wire.read();
//    gx = Wire.read()<<8|Wire.read();
//    gy = Wire.read()<<8|Wire.read();
//    gz = Wire.read()<<8|Wire.read();

//    Serial.print("Accelerometer: ");
//      Serial.print("X = "); Serial.print(ax);
//      Serial.print(" | Y = "); Serial.print(ay);
//      Serial.print(" | Z = "); Serial.println(az); 
//      
//      Serial.print("Gyroscope: ");
//      Serial.print("X = "); Serial.print(gx);
//      Serial.print(" | Y = "); Serial.print(gy);
//      Serial.print(" | Z = "); Serial.println(gz);
//      Serial.println(" ");

    double pitch = getPitch(gx,gy,gz);
    double roll = getRoll(gx,gy,gz);
    double yaw = getYaw(gx,gy,gz);
//
//    Serial.print("Pitch: ");Serial.print(pitch);
//    Serial.print("  |  Roll: "); Serial.print(roll);
//    Serial.print("  |  Yaw: "); Serial.println(yaw);

    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    delay(100);
}

double getPitch(int x, int y,int z) {
    double pitch = 180*atan(x/sqrt((y*y)+(z*z)))/PI;
    return pitch;
}
double getRoll(int x, int y,int z) {
    double roll = 180*atan(y/sqrt((x*x)+(z*z)))/PI;
    return roll;
}
double getYaw(int x, int y,int z) {
    double yaw = 180*atan(z/sqrt((x*x)+(z*z)))/PI;
    return yaw;
}
