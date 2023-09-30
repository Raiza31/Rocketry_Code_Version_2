// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_BMP085.h"
#include "SD.h"
#include "SPI.h"

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

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

File dataFile;

String fileName = "/Rocket00";

const int chipSelect = 4;
Sd2Card card;
SdVolume volume;
SdFile root;

void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    } else {
        Serial.println("BMP180 connection successful");
    }


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setFullScaleAccelRange(3);
    Serial.println(accelgyro.getFullScaleAccelRange());

    // use the code below to change accel/gyro offset values
    /*
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

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

     Serial.print("Initializing SD card...");

     // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1);
    }
    Serial.println("card initialized.");

    // print the type of card
    Serial.println();
    Serial.print("Card type:         ");
    switch(card.type()) {
        case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
        case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
        case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
        default:
        Serial.println("Unknown");
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
        Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        while (1);
    }

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("Volume type is:    FAT");
    Serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    Serial.print("Volume size (Kb):  ");
    Serial.println(volumesize);
    Serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Gb):  ");
    Serial.println((float)volumesize / 1024.0);

    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);

    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);

    for(int i = 1; i < 100; i++){
        if(SD.exists(String(fileName + i + ".txt").c_str())){
            dataFile = SD.open(String(fileName + (i+1) + ".txt").c_str(), FILE_WRITE);
            break;
        }
    }

    //dataFile = SD.open("/Rocket001.txt", FILE_WRITE);

    if (!dataFile) {
        Serial.println("error opening datalog.txt");
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

    dataFile.println(headerString);
    dataFile.flush();
}

void loop() {
  //unsigned long currentMillis = millis();
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  String dataString = "";
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
  //dataString += "\n";

  dataFile.println(dataString);

  dataFile.flush();



// #ifdef OUTPUT_READABLE_ACCELGYRO
//   // display tab-separated accel/gyro x/y/z values
//   //Serial.print("a/g:\t");
//   // Serial.print("ax");
//   Serial.print(ax);
//   Serial.print("\t");
//   //Serial.print("ay");
//   Serial.print(ay);
//   Serial.print("\t");
//   //Serial.print("az");
//   Serial.print(az);
//   Serial.print("\t");
//   //Serial.print("gx");
//   Serial.print(gx);
//   Serial.print("\t");
//   //Serial.print("gy");
//   Serial.print(gy);
//   Serial.print("\t");
//   //Serial.print("gz");
//   Serial.print(gz);
//   Serial.print("\t");
// #endif
  
// #ifdef OUTPUT_BINARY_ACCELGYRO
//   Serial.write((uint8_t)(ax >> 8));
//   Serial.write((uint8_t)(ax & 0xFF));
//   Serial.write((uint8_t)(ay >> 8));
//   Serial.write((uint8_t)(ay & 0xFF));
//   Serial.write((uint8_t)(az >> 8));
//   Serial.write((uint8_t)(az & 0xFF));
//   Serial.write((uint8_t)(gx >> 8));
//   Serial.write((uint8_t)(gx & 0xFF));
//   Serial.write((uint8_t)(gy >> 8));
//   Serial.write((uint8_t)(gy & 0xFF));
//   Serial.write((uint8_t)(gz >> 8));
//   Serial.write((uint8_t)(gz & 0xFF));
// #endif

// Serial.println(bmp.readAltitude(101500));

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}