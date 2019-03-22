#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1023) //https://weather.us/observations/air-pressure-station.html

const double seperatedMass = 350.0/1000; // kg
const double parachuteArea = 1; // m^2
const double dragCoefficient = 1.5;
const double finalAltitude = 856/3.2808; // m
const double endTime = 43; // s  -  43 to 46
const double heightBuffer = 2; // m  -  height to detect if the rocket launched and is descending

const double gravity = 9.80665;
const double e = 2.71828;
const double gasConstant = 287.058;

double start;
double initialAltitude;
double heighestAltitude = 0;
boolean launch;
String fileName;
Adafruit_BMP280 bme;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;
    }
    pinMode(13,HIGH);
    launch = false;
    
    Serial.print("Initializing SD card... ");
    if (!SD.begin(4)) {
        Serial.println("failed to read SD card");
    } else {
        Serial.println("card initialized");
        int launchNumber = 1;
        fileName = "launch" + String(launchNumber) + ".txt";
        while (SD.exists(fileName)) {
            launchNumber += 1;
            fileName = "launch" + String(launchNumber) + ".txt";
        }
    }
    
    if (bme.begin()) {
        initialAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
    } else {
        Serial.println("Failed to find a BMP280 sensor");
        while (1);
    }
    Serial.println();
    logData(String(fileName));
}

void loop() {
    double currentAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
    if (((currentAltitude - initialAltitude) > heightBuffer) and (launch == false)) {
        launch = true;
        start = millis();
        logData("LAUNCH");
    }
    if (launch == true) {
        double altitude = currentAltitude - initialAltitude; // m
        double temperature = bme.readTemperature(); // *C
        double pressure = bme.readPressure(); // Pa
        double now = millis();
        if (altitude > heighestAltitude) {
            heighestAltitude = altitude;
        }
        double timePassed = (now-start)/1000;
        String timeAltitude = String(timePassed,DEC) + ": " + String(altitude,DEC);
        logData(timeAltitude);
        double timeParachute = descentTime(altitude, pressure, temperature);
        if (((timePassed + timeParachute) > endTime) && ((altitude + heightBuffer) < heighestAltitude)) {
            digitalWrite(13,HIGH);
            logData("PARACHUTE");
            while(1);
        } else {
            digitalWrite(13,LOW);
        }
    } else {
        digitalWrite(13,LOW);
    }
    delay(10);
}

double descentTime(double altitude, double pressure, double temperature) {
    double airDensity = pressure/(gasConstant*(temperature+273.15)); // kg/m^3  -  around 1.225
    double airResistance = ((airDensity*parachuteArea)/2)*dragCoefficient; // kg/m  -  around 0.24 for air
    double descentTime = sqrt(seperatedMass/(gravity*airResistance))*acosh(pow(e,((altitude*airResistance)/seperatedMass))); // s
    return descentTime;
}

double acosh(double x) {
    return log(x+sqrt(pow(x,2)-1));
}

void logData(String data) {
    File file = SD.open(fileName, FILE_WRITE);
    if (file) {
        file.println(data);
        file.close();
    }
    Serial.println(data);
}

//http://www.ambrsoft.com/Physics/FreeFall/FreeFallWairResistance.htm
//https://keisan.casio.com/exec/system/1231475371
//http://www.rocketmime.com/rockets/descent.html
//https://www.omnicalculator.com/physics/free-fall-air-resistance
//https://www.omnicalculator.com/physics/drag-equation
//https://www.omnicalculator.com/physics/air-density
