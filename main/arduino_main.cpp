#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

// Libraries
#include <Arduino.h>                                                // General Arduino Library
#include <Bluepad32.h>                                              // Library for Bluepad32
#include "..\components\ESP32Servo-master\src\ESP32_Servo.h"        // Library for Servo Motor
#include <WiFi.h>                                                   // Library for Wifi
#include <Adafruit_Sensor.h>                                        // Library for Adafruit sensors
#include <Wire.h>                                                   // Library for I2C wiring
#include <Adafruit_LIS3DH.h>                                        // Library for 3-axis accelerometer

// Definitions
#define SOUND_SPEED 0.0343                                          // Defining the speed of sound (CM's))

// Setting Classes
static GamepadPtr myGamepad;                                        // Setting the controller as a gamepad type
Servo SteeringServo;                                                // Setting the servo motor as a servo type

// Pin Configuration
int SteeringPin = 26;
int DCMotorPin1 = 21;
int DCMotorPin2 = 12;
int DCMotorPWM = 25;
int UltraSonicTrigRight = 13;
int UltraSonicEchoRight = 32;
int UltraSonicTrigLeft = 27;
int UltraSonicEchoLeft = 14;

// Constrants
double SteeringPosition = 0;
int CarMaxSteeringAngle = 80;

// Variables
// Controller Variables
double AxisX;
double AxisY;

// Steering Variables
double SteeringAngle;

// Motor Power Varaibles
double DCMotorSpeed;
boolean DCMotorDirection;

// Ultrasonic Sensor Variables
long DurationRight;
long DurationLeft;
float DistanceCMRight;
float DistanceCMLeft;

// Accelerometer Variables (using I2C)
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Timer Varaibles
unsigned long lastTimeDelay = 0;
unsigned long timerDelay = 1;                                       // Target 100Hz Sampling frequency

// Define the Network Credentials (Using TP-link range extender)
const char* ssid     = "TPD_Test_Wifi";                             // Name of the Wifi Network
const char* password = "";                                          // Password of the Wifi Network (no password set)
WiFiServer wifiServer(80);                                          // Port the ESP32 will send data along

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

// Connect a New Gamepad - called any time a new gamepad is connect
void onConnectedGamepad(GamepadPtr gp) {
    myGamepad = gp;
    Serial.println("CALLBACK: Gamepad is connected!");
}

// Disconnected Gamepad - called any time a gamepad is disconnect
void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("CALLBACK: Gamepad is disconnected!");
    myGamepad = nullptr;
}

// Arduino setup function. Runs in CPU 1
void setup() {
    setCpuFrequencyMhz(240);                                        // Overclocking the frequency of the ESP32 core

    // Wifi
    // Connecting to WiFi
    Serial.begin(115200);                                           // Setting the serial baud rate
    WiFi.begin(ssid, password);                                     // Begins connecting to the Wifi network

    // Printing Info While Connecting to Wifi
    while (WiFi.status() != WL_CONNECTED){
        delay(1000);
        Serial.println("Connecting to WiFi ...");
    }

    // Printing Info When Connected to Wifi
    Serial.println("Connected to the Wifi Network");
    Serial.println(WiFi.localIP());

    // Initiate the TCP server
    wifiServer.begin();


    // Bluepad32
    // Checking verion and firmware
    String fv = BP32.firmwareVersion();
    Serial.print("Firmware: ");
    Serial.println(fv);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);


    // Servo Motor Setup Information
    SteeringServo.attach(SteeringPin);


    // DC Motor Setup Information
    pinMode(DCMotorPin1, OUTPUT);
    pinMode(DCMotorPin2, OUTPUT);
    pinMode(DCMotorPWM, OUTPUT);


    // Ultrasonic Sensor Setup Information
    // Right Setup
    pinMode(UltraSonicTrigRight, OUTPUT);                               // Sets the UltraSonicTrigRight pin as an Output
    pinMode(UltraSonicEchoRight, INPUT);                                // Sets the UltraSonicEchoRight pin as an Input
    pinMode(UltraSonicTrigLeft, OUTPUT);                                // Sets the UltraSonicTrigLeft pin as an Output
    pinMode(UltraSonicEchoLeft, INPUT);                                 // Sets the UltraSonicEchoLeft pin as an Input


    // 3-Axis Accelerometer Setup
    while (!Serial) delay(10);                                          // Will pause Zero, Leonardo, etc until serial console opens
    
    if (! lis.begin(0x18)) {                                            // Change this to 0x19 for alternative I2C address
        Serial.println("Couldnt start");
        while (1) yield();
    }
    Serial.println("LIS3DH found!");
    
    Serial.print("Range = "); Serial.print(2 << lis.getRange());
    Serial.println("G");
    
    // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
    Serial.print("Data rate set to: ");
    switch (lis.getDataRate()) {
      case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
      case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
      case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
      case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
      case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
      case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
      case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;
      
      case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
      case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
      case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
    }
}

// Arduino loop function. Runs in CPU 1
void loop() {
    WiFiClient Server = wifiServer.available();                         // Creates a wifi server type to allow transmition of data
    if (Server) {
        while (Server.connected()){                                     // Continues around while loop, if TCP server connections are true (at both ends)
            
            // Bluepad32
            BP32.update();                                              // Updating the the controller inputs

            if (myGamepad && myGamepad->isConnected()) {                // Checks to make sure the controller is still connected
            
                // Controller Inputs
                char buffer[80];                                        // Creates a buffer for the required controller inputs
                snprintf(buffer, sizeof(buffer) - 1,
                     "a:%1d, b:%1d, x:%1d, y:%1d, axis L: %4d, %4d",
                     myGamepad->a(),           // (0 - 1) Boolean value of button A
                     myGamepad->b(),           // (0 - 1) Boolean value of button B
                     myGamepad->x(),           // (0 - 1) Boolean value of button X
                     myGamepad->y(),           // (0 - 1) Boolean value of button Y
                     myGamepad->axisX(),       // (-511 - 512) left X Axis
                     myGamepad->axisY()        // (-511 - 512) left Y axis
                );

                // Ultrasonic Distance Sensor
                digitalWrite(UltraSonicTrigRight, LOW);                                     // Clears the UltraSonicTrigRight pin
                delayMicroseconds(2);                                                       // Holds for 2 micro-seconds

                // Pulse the Right Pin
                digitalWrite(UltraSonicTrigRight, HIGH);                                    // Sets the UltraSonicTrigRight pin to High
                delayMicroseconds(10);                                                      // Holds for 10 micro-seconds
                digitalWrite(UltraSonicTrigRight, LOW);                                     // Sets the UltraSonicTrigRight pin to Low

                // Duration of Echo Right Pin
                DurationRight = pulseIn(UltraSonicEchoRight, HIGH);                         // Reads the UltraSonicEchoRight pin and returns the sound wave travel time in microseconds

                digitalWrite(UltraSonicTrigLeft, LOW);                                      // Clears the UltraSonicTrigLeft pin
                delayMicroseconds(2);                                                       // Holds for 2 micro-seconds

                // Pulse the Left Pin
                digitalWrite(UltraSonicTrigLeft, HIGH);                                     // Sets the UltraSonicTrigLeft pin to High
                delayMicroseconds(10);                                                      // Holds for 10 microseconds
                digitalWrite(UltraSonicTrigLeft, LOW);                                      // Sets the UltraSonicTrigLeft pin to Low

                // Duration of Echo Left Pin
                DurationLeft = pulseIn(UltraSonicEchoLeft, HIGH);                           // Reads the UltraSonicEchoLeft pin and returns the sound wave travel time in microseconds

                // Calculate Ultrasonic Distance Sensor Distances
                DistanceCMRight = DurationRight * SOUND_SPEED/2;                            // Calculates the right distance in CM
                DistanceCMLeft = DurationLeft * SOUND_SPEED/2;                              // Calculates the left distance in CM


                // 3-Axis Accelerometer (normalised to m/s^2)
                sensors_event_t Acceleration;                                               // Creates a sensor event called acceleration
                lis.getEvent(&Acceleration);                                                // Reads for the values from the sensor for the event


                // Servo Motor Steering
                AxisX = myGamepad->axisX();                                                 // Convertes the input from controller to a double known as AxisX
                SteeringPosition = ((AxisX+512)*CarMaxSteeringAngle/1024);                  // Calculates the steering position for given controller input
                SteeringServo.write(SteeringPosition);                                      // Writes steering position to pin
                SteeringAngle = SteeringPosition-(CarMaxSteeringAngle/2);                   // Updates steering angle variable, with the middle being 0 degs

                // DC Motor
                // DC Motor Direction
                AxisY = myGamepad->axisY();                                                 // Converts the input from controller to a double known as AxisY
                
                // Ensures the motor is spinning in the correct direction
                if (AxisY <= 0){
                    digitalWrite(DCMotorPin1, LOW);                                         
                    digitalWrite(DCMotorPin2, HIGH);
                    DCMotorDirection = true;
                }
                else if (AxisY > 0){
                    digitalWrite(DCMotorPin1, HIGH);
                    digitalWrite(DCMotorPin2, LOW);
                    DCMotorDirection = false;
                }

                // DC Motor Power
                DCMotorSpeed = ((abs(AxisY))/512.0)*255;                                    // Calcualtes the DC Motor Speed for the given controller input
                analogWrite(DCMotorPWM, (int) DCMotorSpeed);                                // Writes the DC Motor Speed to the DC Motor PWM pin

                // Data Logging (Continuous)
                if ((millis() - lastTimeDelay) > timerDelay){
                    // Order of Variable Prints
                    // Time, Left Joystick:X-Axis, Left Joystick:Y-Axis, Steering Angle, Forwards(1) or Backwards(0), InputDCMotorPower, Ultrasonic Distance Right (cm), Ultrasonic Distance Left (cm), X-axis Acceleration (m/s^2), Y-axis Acceleration (m/s^2), Z-axis Acceleration (m/s^2)

                    Server.print(AxisX);
                    Server.print(",");
                    Server.print(AxisY);
                    Server.print(",");
                    Server.print(SteeringAngle);
                    Server.print(",");
                    Server.print(DCMotorDirection);
                    Server.print(",");
                    Server.print(DCMotorSpeed);
                    Server.print(",");
                    Server.print(DistanceCMRight);
                    Server.print(",");
                    Server.print(DistanceCMLeft);
                    Server.print(",");
                    Server.print(Acceleration.acceleration.x);
                    Server.print(",");
                    Server.print(Acceleration.acceleration.y);
                    Server.print(",");
                    Server.print(Acceleration.acceleration.z);
                    Server.print("\n");

                    lastTimeDelay = millis();

                }
            }
        }
        Server.stop();
        Serial.println("Client Disconnected");
    }  
}