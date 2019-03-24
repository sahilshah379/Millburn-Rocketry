#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1023) //https://weather.us/observations/new-jersey/pressure-qnh/20190324-1400z.html

double initialAltitude;
String fileName;
Adafruit_BMP280 bme;

void setup() {
    pinMode(13,HIGH);
    if (!SD.begin(4)) {
        while (1) {
            digitalWrite(13,HIGH);
            delay(100);
            digitalWrite(13,LOW);
            delay(100);
        }
    } else {
        int launchNumber = 1;
        fileName = "launch" + String(launchNumber) + ".txt";RC
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
    double t = millis()/1000;
    String timeAltitude = String(t,DEC) + ": " + String(altitude,DEC);
    logData(timeAltitude);
    delay(100);
}

void logData(String data) {
    File file = SD.open(fileName, FILE_WRITE);
    if (file) {
        file.println(data);
        file.close();
    }
}
