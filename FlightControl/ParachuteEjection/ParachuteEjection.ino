#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// max 908
// speed 302

#define SEALEVELPRESSURE_HPA (1023) //https://weather.us/observations/new-jersey/pressure-qnh/20190324-1400z.html

const double seperatedMass = 370.0/1000; // kg
const double parachuteArea = 2 * (PI*pow((15.5*2.54/2),2) - PI*pow((6*2.54/2),2)); // m^2
const double dragCoefficient = 1.5;
const double finalAltitude = 856/3.2808; // m
const double endTime = 43; // s  -  43 to 46
const double heightBuffer = 2; // m  -  height to determine the stage the rocket is currently in

const double gravity = 9.80665;
const double e = 2.71828;
const double gasConstant = 287.058;

double start;
double initialAltitude;
double heighestAltitude;
boolean launch;
boolean parachute;
String fileName;
Adafruit_BMP280 bme;

void setup() {
    pinMode(13,HIGH);
    launch = false;
    parachute = false;
    heighestAltitude = heightBuffer;
    
    if (!SD.begin(4)) {
        while (1) {
            digitalWrite(13,HIGH);
            delay(100);
            digitalWrite(13,LOW);
            delay(100);
        }
    } else {
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
        while (1) {
            digitalWrite(13,HIGH);
            delay(100);
            digitalWrite(13,LOW);
            delay(100);
            if (bme.begin()) {
                initialAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
                logData("Found BMP280 sensor");
                digitalWrite(13,HIGH);
                break;
            }
        }
    }
    logData(String(fileName));
}

void loop() {
    double currentAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
    double altitude = currentAltitude - initialAltitude; // m
    if ((altitude > heightBuffer) and (launch == false)) {
        launch = true;
        start = millis();
        logData("LAUNCH");
    }
    if (launch == true) {
        double temperature = bme.readTemperature(); // *C
        double pressure = bme.readPressure(); // Pa
        if (altitude > heighestAltitude) {
            heighestAltitude = altitude;
        }
        double now = millis();
        double timePassed = (now-start)/1000;
        String timeAltitude = String(timePassed,DEC) + ": " + String(altitude,DEC);
        if (parachute == true) {
            timeAltitude = "     " + timeAltitude;
        }
        logData(timeAltitude);
        double timeParachute = descentTime(altitude, pressure, temperature);
        if (((timePassed + timeParachute) > endTime) && ((altitude + heightBuffer) < heighestAltitude)) {
            if (parachute == false) {
                logData("PARACHUTE");
                parachute = true;
            }
        }
        if (parachute == true) {
            if (altitude < heightBuffer) {
                logData("LAND");
                // land stage
                while (1) {
                    digitalWrite(13,HIGH);
                    delay(1000);
                    digitalWrite(13,LOW);
                    delay(1000);
                }
            } else {
                // parachute stage
                digitalWrite(13,LOW);
            }
        } else {
            // launch stage
            digitalWrite(13,HIGH);
        }
    } else {
        // pre-launch stage
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
}

//http://www.ambrsoft.com/Physics/FreeFall/FreeFallWairResistance.htm
//https://keisan.casio.com/exec/system/1231475371
//http://www.rocketmime.com/rockets/descent.html
//https://www.omnicalculator.com/physics/free-fall-air-resistance
//https://www.omnicalculator.com/physics/drag-equation
//https://www.omnicalculator.com/physics/air-density
