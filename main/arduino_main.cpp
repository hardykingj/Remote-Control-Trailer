
#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

// Libraries
#include <Arduino.h>                                                // General Arduino Library
#include <Bluepad32.h>                                              // Library for Bluepad32
#include "..\components\ESP32Servo-master\src\ESP32_Servo.h"        // Library for Servo Motor
#include "FS.h"                                                     // Library for SD Card
#include "SD.h"                                                     // Library for SD Card
#include "SPI.h"                                                    // Library for SD Card
#include <WiFi.h>                                                   // Library for Wifi
#include "time.h"                                                   // Library for Time
#include <ctime>                                                    // Library for the conversion of Epochtime to GMT
#include <filesystem>                                               // Library to allow folders to be created
#include "DHT.h"                                                    // Library to allow DHT temperature and humidity sensor

// Definitions
using namespace std;                                                // Definition for the folder convention
#define DHTTYPE DHT11                                               // Defining the type of temperature and humidity sensor

// Defining Classes
static GamepadPtr myGamepad;
Servo SteeringServo;

// Pin Configuration
int SteeringPin = 26;
int OnPin = 33;
int ConnectedPin = 27;
int RecordingPin = 15;
int DCDirectionPin = 21;
int DCMotorPin = 12;
int DCSignalPin = 25;
int TempSensor = 14;

// Model Constants
double SteeringPosition = 0;
int maxSteeringAngle = 180;

// Code Constants
long referencemills = 0;
String DataLog;
const char* DataLogchar;
boolean Recording;
double RotVelocity;
double AxisY;
double DCMotorRev;
double DCMotorPWM;
File DataLogFile;

// Temperature and Humidity Sensor Variables
DHT dht(TempSensor, DHTTYPE);
float Temperature;
float Humidity;

// Data Log File Path
String MasterFolderName = "DataLog";
String SubFolderName = "CarLog";
String FolderPath;
String LocalFolderPath;
String FileName = "Log";
String FilePath;

// Time Variables
// Day Variables
String Year;
String Month;
String Day;
String Hour;
String Min;
String Sec;

// Timer variables
unsigned long lastTimeDelay = 0;
unsigned long lastTimeWrite = 0;
unsigned long timerDelay = 62.5;                        // 16Hz Sampling frequency

// Variable to save current epoch time
unsigned long epochTime; 
const char* GMTime;

// Defined network credentials (Using Jacob's iPhone)
const char* ssid     = "iPhone";
const char* password = "12345678";

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

// Callback gets called any time a new gamepad is connect (upto 4 can be connected at anytime)
void onConnectedGamepad(GamepadPtr gp) {

    myGamepad = gp;
    Serial.println("CALLBACK: Gamepad is connected!");

    digitalWrite(ConnectedPin, HIGH);                               // Turing on LED once Gamepad is connected
}

// Callback gets called any time a gamepad is disconnect
void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("CALLBACK: Gamepad is disconnected!");
    myGamepad = nullptr;

    digitalWrite(ConnectedPin, LOW);                                // Turning off LED once Gamepad is connected
}

// Functions required to enable SD card integration
// Setup SD Card
void initSDCard(){
    if(!SD.begin(SS)){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    // Print SD card type on Serial Monitor
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// Listing of directories on SD card
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

// Create a directory on SD card
void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}   

// Removes a directory on SD card
void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

// Read file content on SD card
void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

// Write file content on SD card
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

// Append content to a file on SD card
void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

// Rename a file on SD card
void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

// Delete a file on SD card
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

// Convert controller input into DC motor power
void DCMotorPower(){
    AxisY = (double) myGamepad->axisY();

    RotVelocity = 255 - (((abs(AxisY))/512.0)*255);
    analogWrite(DCMotorPin, (int) RotVelocity);
}

// Creates a function that tests the capability of the SD card
void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

// Arduino setup function. Runs in CPU 1
void setup() {

    setCpuFrequencyMhz(240);

    pinMode(SS, INPUT_PULLDOWN);
    pinMode(MOSI, INPUT_PULLDOWN);
    pinMode(MISO, INPUT_PULLDOWN);
    pinMode(SCK, INPUT_PULLDOWN);

    initWiFi();
    initSDCard();
    configTime(0, 0, ntpServer);

    // BluePad32 Setup Information
    Serial.begin(115200);

    String fv = BP32.firmwareVersion();
    Serial.print("Firmware: ");
    Serial.println(fv);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // SD Card Initiation
    Serial.begin(115200);
        delay(200);
        while (! SD.begin(33)){

        }
        Serial.print("SD OK");

    // Configuring Time
    // Get Epochtime
    epochTime = getTime();                                              // Fetches the Epochtime from web server
    delay(200);                                                         // Delay to ensure the time is collected before any data logging

    // Convert Epochtime to GMT
    time_t epochTime;                                                   // Converting Epochtime into GMT
    time(&epochTime);                                                   // Converting Epochtime into GMT

    // Prints GMT
    GMTime = asctime(localtime(&epochTime));                            // Changing GMTime format to Www Mmm dd hh:mm:ss yyyy
    Serial.print(GMTime);                                               // Printing the updated GMTime format
    tm *local_time = localtime(&epochTime);                             // Creating local_time that will allow interigation into specific time variables

    // Setting time variables
    Year = local_time->tm_year + 1900;
    Month = local_time->tm_mon + 1;
    Day = local_time->tm_mday;

    // Creating Data Log Folders and Files
    FolderPath = "/" + MasterFolderName;                                // Creating folder path for the master folder
    createDir(SD, FolderPath.c_str());                                  // Creating the folder inside the SD card

    FolderPath = FolderPath + "/" + SubFolderName;                      // Updating the folder path to include the sub folder
    createDir(SD, FolderPath.c_str());                                  // Creating the sub folder inside of the master folder

    FolderPath = FolderPath + "/" + Year;                               // Update the folder path to include the year
    createDir(SD, FolderPath.c_str());                                  // Creating the year folder inside of the sub folder

    FolderPath = FolderPath + "/" + Month;                              // Update the folder path to include the month
    createDir(SD, FolderPath.c_str());                                  // Creating the month folder inside of the year folder

    FolderPath = FolderPath + "/" + Day;                                // Update the folder path to include the day
    createDir(SD, FolderPath.c_str());                                  // Creating the day folder inside of the month folder

    // Servo Motor Setup Information
    SteeringServo.attach(SteeringPin);

    // DC Motor Setup Information
    pinMode(DCDirectionPin, OUTPUT);                                // Direction control for DCDirectionPin with direction wire
    pinMode(DCMotorPin, OUTPUT);                                    // PWM for DCMotorPin with PWM wire
    digitalWrite(DCMotorPin, 255);                                  // Default DC Motor to not spin on StartUp
    pinMode(DCSignalPin, INPUT);

    // Temperature and Humidity Sensor Setup Information
    dht.begin();

    // LED Setup Information
    // Complete Setup LED
    pinMode(OnPin, OUTPUT);
    digitalWrite(OnPin, HIGH);                                      // Turning on LED once Setup is complete

    // Gamepad Connected LED
    pinMode(ConnectedPin, OUTPUT);                                  // Setting up ConnectedPin state

    // DataLogging LED
    pinMode(RecordingPin, OUTPUT);                                  // Settung up DataLogging State

    // Set recording status to false
    Recording = false;
}

// Arduino loop function. Runs in CPU 1
void loop() {
    
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    if (myGamepad && myGamepad->isConnected()) {
        
        // Another way to query the buttons, is by calling buttons(), or
        // miscButtons() which return a bitmask.
        // Some gamepads also have DPAD, axis and more.
        char buffer[150];
        snprintf(buffer, sizeof(buffer) - 1,
                 "dpad: 0x%02x, buttons: 0x%04x, a:%1d, b:%1d, x:%1d, y:%1d, axis L: %4d, %4d, axis R: %4d, "
                 "%4d, brake: %4d, throttle: %4d, misc: 0x%02x",
                 myGamepad->dpad(),        // DPAD
                 myGamepad->buttons(),     // bitmask of pressed buttons
                 myGamepad->a(),           // (0 - 1) Boolean value of button A
                 myGamepad->b(),           // (0 - 1) Boolean value of button B
                 myGamepad->x(),           // (0 - 1) Boolean value of button X
                 myGamepad->y(),           // (0 - 1) Boolean value of button Y
                 myGamepad->axisX(),       // (-511 - 512) left X Axis
                 myGamepad->axisY(),       // (-511 - 512) left Y axis
                 myGamepad->axisRX(),      // (-511 - 512) right X axis
                 myGamepad->axisRY(),      // (-511 - 512) right Y axis
                 myGamepad->brake(),       // (0 - 1023): brake button
                 myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
                 myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
        );
        Serial.println(buffer);

        // Get epoch time
        epochTime = getTime();

        // Convert Epochtime to GMT
        time_t epochTime;                                                   // Converting Epochtime into GMT
        time(&epochTime);                                                   // Converting Epochtime into GMT
        tm *local_time = localtime(&epochTime);                             // Creating local_time that will allow interigation into specific time variables

        // Setting specific time variables
        Hour = local_time->tm_hour;
        Min = local_time->tm_min;
        Sec = local_time->tm_sec;

        // Temperature and Humidity Sensor
        Humidity = dht.readHumidity();
        Temperature = dht.readTemperature();

        // Servo Motor Steering
        SteeringPosition = (((myGamepad->axisX())+512)*maxSteeringAngle/1024);
        SteeringServo.write(SteeringPosition);

        // DC Motor Power
        if(myGamepad->axisY() < 0){
            digitalWrite(DCDirectionPin, HIGH);

            DCMotorPower();
        }

        if(myGamepad->axisY() >= 0){
            digitalWrite(DCDirectionPin, LOW);

            DCMotorPower();
        }

        // Reading DC Motor Encoder
        DCMotorPWM = pulseIn(DCSignalPin, HIGH, 10000);
        if (DCMotorPWM == 0){
            DCMotorRev = 0;
        }
        else{
            DCMotorRev = (111111/(DCMotorPWM));
        }

        // Data logging commands
        // Set Recording to true when X is pressed
        if(myGamepad->x() == 1){
            Recording = true;

            LocalFolderPath = FolderPath + "/" + Hour;
            createDir(SD, LocalFolderPath.c_str());

            LocalFolderPath = LocalFolderPath + "/" + Min;
            createDir(SD, LocalFolderPath.c_str());

            FilePath = LocalFolderPath + "/" + FileName + ".csv";
        
            DataLogFile = SD.open(FilePath.c_str(), FILE_WRITE);

            DataLogFile.println("Date,Time,Temperature(C),Humidity(%),Button B,Button X,Left Joystick:X-Axis,Left Joystick:Y-Axis,Steering Angle,Forwards(1) or Backwards(0),InputDCMotorPower,DC Motor Speed (Encoder Value) - Rev/min");
        }

        // Set Recording to flase when B is pressed
        if(myGamepad->b() == 1){
            Recording = false;

            DataLogFile.close();
        }

        if(Recording == true){
            digitalWrite(RecordingPin, HIGH);

            if ((millis() - lastTimeDelay) > timerDelay) {

                DataLog = String(Year) + "-" + String(Month) + "-" + String(Day) + "," + String(Hour) + "-" + String(Min) + "-" + String(Sec) + ","
                + String(Temperature) + "," + String(Humidity) + "," + String(myGamepad->b()) + "," 
                + String(myGamepad->x()) + "," + String(myGamepad->axisX()) + "," 
                + String(myGamepad->axisY()) + "," + String(SteeringPosition) + "," + String(digitalRead(DCDirectionPin)) + "," 
                + String(RotVelocity) + "," + String(DCMotorRev);

                DataLogFile.println(DataLog);

                lastTimeDelay = millis();
            }
        }

        if(Recording == false){
            digitalWrite(RecordingPin, LOW);
        }
    }
}