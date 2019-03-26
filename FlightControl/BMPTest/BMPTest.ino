#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1023) //https://weather.us/observations/new-jersey/pressure-qnh/20190324-1400z.html

Adafruit_BMP280 bme;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;
    }
    
    if (!bme.begin()) {
        Serial.println("Failed to find a BMP280 sensor");
        while (1);
    }
}

void loop() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure());
    Serial.println(" Pa");
    
    Serial.print("Approx altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    
    Serial.println();
    delay(2000);
}
