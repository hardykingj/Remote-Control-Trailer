// Libraries
#include <Arduino.h>                                                // General Arduino Library
#include <WiFi.h>                                                   // Library for Wifi
#include <Adafruit_Sensor.h>                                        // Library for Adafruit sensors
#include <Wire.h>                                                   // Library for I2C wiring
#include <Adafruit_LIS3DH.h>                                        // Library for 3-axis accelerometer

// Pin Configuration
int DCMotorPin1 = 21;
int DCMotorPin2 = 12;
int DCMotorPWM = 25;
int LinearTransducerPin = 33;

// Motor Power Varaibles
double DCMotorSpeed;
boolean DCMotorDirection;

// Linear Transducer Variables
float MaxADCBits = 4095.0f;
float MaxVoltage = 5.0f;
const float VoltsPerBits = MaxVoltage/MaxADCBits;
float LinearTransducerVoltage;
float LinearDistance;

// Accelerometer Variables (using I2C)
Adafruit_LIS3DH AccFront = Adafruit_LIS3DH();
Adafruit_LIS3DH AccRear = Adafruit_LIS3DH();

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


    // DC Motor Setup Information
    pinMode(DCMotorPin1, OUTPUT);
    pinMode(DCMotorPin2, OUTPUT);
    pinMode(DCMotorPWM, OUTPUT);


    // Linear Transducer Setup Information
    pinMode(LinearTransducerPin, INPUT);
    
    
    // 3-Axis Accelerometer Setup
    // Front Accelerometer Setup
    while (!Serial) delay(10);                                          // Will pause Zero, Leonardo, etc until serial console opens
    
    if (! AccFront.begin(0x18)) {                                            // Looks at particular I2C address
        Serial.println("Couldnt start");
        while (1) yield();
    }
    Serial.println("LIS3DH found!");
    
    AccFront.setDataRate(LIS3DH_DATARATE_100_HZ);                       // Set's datalog rate to 100Hz
    AccFront.setRange(LIS3DH_RANGE_4_G);                                // Set's acceleration range to 4G


    // Rear Accelerometer Setup
    while (!Serial) delay(10);                                          // Will pause Zero, Leonardo, etc until serial console opens
    
    if (! AccRear.begin(0x19)) {                                        // Looks at particular I2C address
        Serial.println("Couldnt start");
        while (1) yield();
    }
    Serial.println("LIS3DH rear found!");
    
    AccRear.setDataRate(LIS3DH_DATARATE_100_HZ);                        // Set's datalog rate to 100Hz
    AccRear.setRange(LIS3DH_RANGE_4_G);                                 // Set's acceleration range to 4G
}

// Arduino loop function. Runs in CPU 1
void loop() {
    WiFiClient Server = wifiServer.available();                                         // Creates a wifi server type to allow transmition of data
    if (Server) {
        while (Server.connected()){                                                     // Continues around while loop, if TCP server connections are true (at both ends)
            

            // 3-Axis Accelerometer (normalised to m/s^2)
            sensors_event_t FrontAcceleration;                                          // Creates a sensor event called Front Acceleration
            sensors_event_t RearAcceleration;                                           // Creates a sensor event called Rear Acceleration
            AccFront.getEvent(&FrontAcceleration);                                      // Reads for the values from the sensor for Front Acceleration
            AccRear.getEvent(&RearAcceleration);                                        // Reads for the values from the sensors for Rear Acceleration


            // Linear Transduer
            LinearTransducerVoltage = analogRead(LinearTransducerPin) * VoltsPerBits;   // Reading the analog output from the ADC and converting to voltage range (0v - 5v)
            LinearDistance = (LinearTransducerVoltage-2.5)/0.2;                         // Converting linear transducer voltage to linear distance from set point (exactly in middle) - total travel = 25mm


            // DC Motor
            // DC Motor Direction
            if (LinearDistance >= 0){
                digitalWrite(DCMotorPin1, LOW);                                         
                digitalWrite(DCMotorPin2, HIGH);
                DCMotorDirection = true;
            }
            else if (LinearDistance > 0){
                digitalWrite(DCMotorPin1, HIGH);
                digitalWrite(DCMotorPin2, LOW);
                DCMotorDirection = false;
            }

            // DC Motor Power
            DCMotorSpeed = ((abs(LinearDistance))/12.5)*255;                             // Calcualtes the DC Motor Speed for the given linear transducer distance
            analogWrite(DCMotorPWM, (int) DCMotorSpeed);                                 // Writes the DC Motor Speed to the DC Motor PWM pin


            // Data Logging (Continuous)
            if ((millis() - lastTimeDelay) > timerDelay){
                // Order of Variable Prints
                // Time, Linear Transducer Distance (mm), Forwards(1) or Backwards(0), InputDCMotorPower, Front X-axis Acceleration (m/s^2), Front Y-axis Acceleration (m/s^2), Front Z-axis Acceleration (m/s^2), Rear X-axis Acceleration (m/s^2), Rear Y-axis Acceleration (m/s^2), Rear Z-axis Acceleration (m/s^2)

                Server.print(LinearDistance);
                Server.print(",");
                Server.print(DCMotorDirection);
                Server.print(",");
                Server.print(DCMotorSpeed);
                Server.print(",");
                Server.print(FrontAcceleration.acceleration.x);
                Server.print(",");
                Server.print(FrontAcceleration.acceleration.y);
                Server.print(",");
                Server.print(FrontAcceleration.acceleration.z);
                Server.print(",");
                Server.print(RearAcceleration.acceleration.x);
                Server.print(",");
                Server.print(RearAcceleration.acceleration.y);
                Server.print(",");
                Server.print(RearAcceleration.acceleration.z);
                Server.print("\n");

                lastTimeDelay = millis();
            }
        }
        Server.stop();
        Serial.println("Client Disconnected");
    }  
}