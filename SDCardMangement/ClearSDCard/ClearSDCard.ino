#include <SPI.h>
#include <SD.h>

File root;

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
    Serial.println("Deleted:");
    root = SD.open("/");
    while (true) {
        File file =  root.openNextFile();
        if (!file) {
            break;
        }
        Serial.print("\t");
        Serial.print(file.name());
        SD.remove(file.name());
        if (SD.exists(file.name())) {
            Serial.println(" - fail");
        } else {
            Serial.println(" - success");
        }
    }
    Serial.println();
    Serial.println("Done!");
}

void loop() {}
