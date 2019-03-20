#include <SPI.h>
#include <SD.h>

File file;

void setup() {
    Serial.begin(9600);
     while (!Serial) {
        ;
    }
    Serial.print("Initializing SD card... ");
    if (!SD.begin(4)) {
        Serial.println("failed to read SD card");
        while (1);
    }
    Serial.println("card initialized");
    Serial.println();
    
    int launchNumber = 1;
    String fileName = "launch" + String(launchNumber) + ".txt";
    while (SD.exists(fileName)) {
        launchNumber += 1;
        fileName = "launch" + String(launchNumber) + ".txt";
    }
    fileName = "launch" + String(launchNumber-1) + ".txt";
    file = SD.open(fileName);
    if (file) {
        Serial.println();
        while (file.available()) {
            Serial.write(file.read());
        }
        file.close();
    } else {
        Serial.print("Error opening ");
        Serial.println(fileName);
    }
}

void loop() {}
