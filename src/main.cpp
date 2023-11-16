// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_BMP085.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include <TinyPICO.h>
#include <LoRa.h>
#include <HardwareSerial.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_BMP085 bmp;

File dataFile;
String fileName = "/Rocket";
const int chipSelect = 5;

unsigned long previousMillis = millis();
long interval = 1000;

TinyPICO tp = TinyPICO();

//define the pins used by the transceiver module
#define ss 27
#define rst 26
#define dio0 25

String LoRaData = "";
bool once = false;

//gps: Beitian BN-220
//HardWare Serial(Tx=, Rx= on TinyPico)
HardwareSerial hs;
// The TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    hs.begin(9600, SERIAL_8N1, 33, 32);
    while(!Serial); // wait for serial port to connect. Needed for native USB port only

    //initialize bmp
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        tp.DotStar_SetPixelColor(255, 255, 0);//yellow
        while (1);
    } else {
        Serial.println("BMP180 connection successful");
    }

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    if (!accelgyro.testConnection()) {
        Serial.println("MPU6050 connection failed");
        tp.DotStar_SetPixelColor( 0, 0, 255);//blue
        while (1) {}
    } else {
        Serial.println("MPU6050 connection successful");
    }
    accelgyro.setFullScaleAccelRange(3);

    /* use the code below to change accel/gyro offset values
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        tp.DotStar_SetPixelColor( 255, 0, 255);//purple
        // don't do anything more:
        while(1);
    }
    Serial.println("card initialized.");

    for(int i = 1; i < 100; i++){
      if(!SD.exists(String(fileName + i + ".txt").c_str())){
          dataFile = SD.open(String(fileName + i + ".txt").c_str(), FILE_WRITE);
          Serial.println(String(fileName + i + ".txt"));
          break;
      }
    }

    if (!dataFile) {
        Serial.println("error opening Datalog");
        while(1);
    }

    String headerString = "";
    headerString += "T(ms)";
    headerString += "\t";
    headerString += "Alt(m)";
    headerString += "\t";
    headerString += "AX(m/s/s)";
    headerString += "\t";
    headerString += "AY(m/s/s)";
    headerString += "\t";
    headerString += "AZ(m/s/s)";
    headerString += "\t";
    headerString += "GyroX";
    headerString += "\t";
    headerString += "GyroY";
    headerString += "\t";
    headerString += "GyroZ";
    headerString += "\t";
    headerString += "Sats";
    headerString += "\t";
    headerString += "HDOP";
    headerString += "\t";
    headerString += "Lat";
    headerString += "\t";
    headerString += "Long";

    dataFile.println(headerString);
    dataFile.flush();

    //setup LoRa transceiver module
    LoRa.setPins(ss, rst, dio0);
  
    //replace the LoRa.begin(---E-) argument with your location's frequency 
    //433E6 for Asia
    //866E6 for Europe
    //915E6 for North America
    while (!LoRa.begin(915E6)) {
        tp.DotStar_SetPixelColor( 255, 165, 0);//orange
        Serial.print(".");
        delay(500);
    }
    // Change sync word (0xF3) to match the receiver
    // The sync word assures you don't get LoRa messages from other LoRa transceivers
    // ranges from 0-0xFF
    LoRa.setSyncWord(0xA1);
    Serial.println("LoRa Initializing OK!");

    if(millis() > 5000 && gps.charsProcessed() < 10){
        Serial.println(F("No GPS detected: check wiring."));
        tp.DotStar_SetPixelColor( 0, 255, 255);//teal
        while(true);
    }

    tp.DotStar_SetPixelColor( 0, 255, 0);//green
    delay(5000);
    tp.DotStar_Clear();//turn off light
}

void logInfo(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()){
    dataString += String(gps.location.lat(), 6).c_str();
    dataString += String(gps.location.lng(), 6).c_str();
  }
  else{
    Serial.print(F("INVALID"));
  }
}

void loop() {
    String dataString = "";

    int packetSize = LoRa.parsePacket();
    if(packetSize) {    // received a packet
        Serial.print("Received packet: ");
        // read packet
        while(LoRa.available()) {
            LoRaData = LoRa.readString();
            Serial.println(LoRaData);
            once = true;
        }
    }
    
    if(LoRaData == "Log"){
        if(once){
            tp.DotStar_SetPixelColor( 0, 255, 0);//green
            delay(2500);
            tp.DotStar_Clear();//turn off light
            once = false;
        }

        unsigned long currentMillis = millis();

        if(currentMillis - previousMillis > interval){
            previousMillis = currentMillis;
            
            Serial.println("Sending");

            LoRa.beginPacket();
            LoRa.print("Altitude: ");
            LoRa.print(String(bmp.readAltitude(101500)));
            LoRa.endPacket();

        }
        
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        dataString += String(millis()).c_str();
        dataString += "\t";
        dataString += String(bmp.readAltitude(101500)).c_str();
        dataString += "\t";
        dataString += String(ax).c_str();
        dataString += "\t";
        dataString += String(ay).c_str();
        dataString += "\t";
        dataString += String(az).c_str();
        dataString += "\t";
        dataString += String(gx).c_str();
        dataString += "\t";
        dataString += String(gy).c_str();
        dataString += "\t";
        dataString += String(gz).c_str();

        if(hs.available() > 0){
            logInfo();
        }else{
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
            headerString += "\t";
        }

        dataFile.println(dataString);
        dataFile.flush();
    }
    
    else if(LoRaData == "Stop" && once == true){
        LoRa.beginPacket();
        LoRa.print("Stopped");
        LoRa.endPacket();
        once = false;
    }
}