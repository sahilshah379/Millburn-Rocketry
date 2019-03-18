#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1023) //https://weather.us/observations/air-pressure-station.html

const double mass = 360.0/1000; // kg
const double parachuteArea = 1; // m^2
const double dragCoefficient = 1.5;
const double finalAltitude = 856; // m
const double endTime = 43; // s  -  43 to 46

const double gravity = 9.80665;
const double e = 2.71828;
const double gasConstant = 287.058;

double start;
double initialAltitude;

Adafruit_BMP280 bme;

void setup() {
    Serial.begin(9600);
    pinMode(13,HIGH);
    start = millis();
    if (bme.begin()) {
        initialAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        Serial.println("Failed to find a BMP280 sensor");
        while (1);
    }
}

void loop() {
    double temperature = bme.readTemperature(); // *C
    double pressure = bme.readPressure(); // Pa
    double altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude; // m
    double now = millis();
    double timePassed = (now-start)/1000;
    Serial.print(timePassed);
    Serial.print(": ");
    Serial.println(altitude);
    double timeParachute = descentTime(altitude, pressure, temperature);
    if ((timePassed + timeParachute) > endTime) {
        digitalWrite(13,HIGH);
        Serial.println("Parachute");
        while(1);
    } else {
        digitalWrite(13,LOW);
    }
    delay(100);
}

double descentTime(double altitude, double pressure, double temperature) {
    double airDensity = pressure/(gasConstant*(temperature+273.15)); // kg/m^3  -  around 1.225
    double airResistance = ((airDensity*parachuteArea)/2)*dragCoefficient; // kg/m  -  around 0.24 for air
    double descentTime = sqrt(mass/(gravity*airResistance))*acosh(pow(e,((altitude*airResistance)/mass))); // s
    return descentTime;
}

double acosh(double x) {
    return log(x+sqrt(pow(x,2)-1));
}


//http://www.ambrsoft.com/Physics/FreeFall/FreeFallWairResistance.htm
//https://keisan.casio.com/exec/system/1231475371
//http://www.rocketmime.com/rockets/descent.html
//https://www.omnicalculator.com/physics/free-fall-air-resistance
//https://www.omnicalculator.com/physics/drag-equation
//https://www.omnicalculator.com/physics/air-density
