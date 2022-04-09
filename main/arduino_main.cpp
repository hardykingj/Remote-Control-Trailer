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
    WiFiClient Server = wifiServer.available();                                         // Creates a wifi server type to allow transmition of data
    if (Server) {
        while (Server.connected()){                                                     // Continues around while loop, if TCP server connections are true (at both ends)
            
            // 3-Axis Accelerometer (normalised to m/s^2)
            sensors_event_t Acceleration;                                               // Creates a sensor event called acceleration
            lis.getEvent(&Acceleration);                                                // Reads for the values from the sensor for the event

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
                // Time, Linear Transducer Distance (mm), Forwards(1) or Backwards(0), InputDCMotorPower, X-axis Acceleration (m/s^2), Y-axis Acceleration (m/s^2), Z-axis Acceleration (m/s^2)

                Server.print(LinearDistance);
                Server.print(",");
                Server.print(DCMotorDirection);
                Server.print(",");
                Server.print(DCMotorSpeed);
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
        Server.stop();
        Serial.println("Client Disconnected");
    }  
}