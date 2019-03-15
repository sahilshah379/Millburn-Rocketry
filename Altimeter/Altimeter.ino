#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1019) //https://weather.us/observations/air-pressure-station.html
#define BMP3XX_DEBUG

Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    if (!bmp.begin()) {
        Serial.println("Failed to find a BMP3 sensor");
        while (1);
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void loop() {
    Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(1000);
}
