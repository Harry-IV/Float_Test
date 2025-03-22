#include <WiFi.h>
#include <WebServer.h>
#include "ArrayList.h"
#include <ESP32Servo.h>

#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define MOTOR_IN1 15
#define MOTOR_IN2 33

MS5837 sensor;

struct DepthPacket {
  float depth;
  unsigned long Time;
};


// Wi-Fi Configuration as Access Point (AF), ie., its own local network
const char* ssid = "JHS_POD";  // Local network SSID
const char* password = "Porsche911!";        // Local network password
const int Wi_Fi_CHANNEL = 6; //

DepthPacket mySensorData;
ArrayList<DepthPacket> mySensorDataList;

int lastOneIndex;
const unsigned long RESETTIME = 60*1000; //reset every x minute
unsigned long lastResetTime = 0;
const int MAXPROFILECOUNT = 2;
int profileCount = 1;
String ctrlCommand = "";

Servo myservo; // Servo Initialization/setup
int servoPin = 32;
int servopos = 1500;


// Pids initialization
float p = 0.5;
float i = 0.01;
float d = 0.1;
float desired_depth=2.5;
float prev_error=0;
float integral=0;
float derivative=0;
float error=0;
int stage;
float start_time;
bool deployed;



// Web Server
WebServer webServer(80);

DepthPacket readData() {
  DepthPacket data;
  sensor.read();
  data.depth = sensor.depth();
  data.Time = millis()/1000;
  return data;
};

void sendData(DepthPacket data){
  String msg = "EX01 " + String(data.Time) + " sec " + String(data.depth) + " m ";
  Serial.println(msg);
};

// Send Sensor Data as plain text
void handleDataRequest() {
  Serial.println("Preparing data payload to be sent ...");
  String dataPayload = "[";
  String record;
  for (int i = 0; i<mySensorDataList.size(); i++) {
    if (i != 0)
      dataPayload += ", ";
    DepthPacket data = mySensorDataList.get(i);
    String record = "{\"Timestamp\": " + String(data.Time) + ", \"depth\": " + String(data.depth) + "}";
    dataPayload += record;
  }
  dataPayload += "]";

  webServer.send(200, "text/plain", dataPayload);
  Serial.println("Data payload is sent.");
};

// Send Sensor Data as JSON
void handleControlRequest() {
  String msg;
  Serial.println("In control request processing ...");

  if (webServer.hasArg("command"))
  {
    ctrlCommand = webServer.arg("command");
    Serial.println(ctrlCommand);

    if (ctrlCommand == "START") {  
      lastResetTime = millis();
      mySensorDataList.clear();
      msg = "Received START command. Starting ...";
    }
    else if (ctrlCommand == "STATUS") {      
      msg = "EX01 " + String(mySensorData.Time) + " sec " + String(mySensorData.depth) + " m ";
    }
    else if (ctrlCommand == "UP") {
      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN1, HIGH);
      delay(2000);
      digitalWrite(MOTOR_IN1, LOW);
      msg = "Going UP!";
    }
    else if (ctrlCommand == "DOWN") {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      delay(2000);
      digitalWrite(MOTOR_IN2, LOW);
      msg = "Going DOWN!";
    }
    else if (ctrlCommand == "DEPLOYFLAPS"){
      for(int pos = servopos; pos >= 1200; pos-=1){
        myservo.writeMicroseconds(pos);
        delay(1);
      }
      servopos = 1200;
      msg = "Deploying Flaps!";
    }
    else if (ctrlCommand == "RETRACTFLAPS"){
      for(int pos = servopos; pos <= 1800; pos+=1){
        myservo.writeMicroseconds(pos);
        delay(1);
      }
      servopos = 1800;
      msg = "Retracting Flaps";
    }
    else {
      msg = "Command " + ctrlCommand + " received.";
    }
   
  }

  webServer.send(200, "text/plain", msg);
  Serial.println("Response to control command is sent.");
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
  
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  // .init sets the sensor model for us but we can override it if required.
  // Uncomment the next line to force the sensor model to the MS5837_30BA.
  sensor.setModel(MS5837::MS5837_02BA);

  sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)

  // Create WiFi network Access Point
  WiFi.softAP(ssid, password, Wi_Fi_CHANNEL);
  Serial.println("Wi-Fi Network Created");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Configure Web Server Routes
  webServer.on("/data", handleDataRequest);
  webServer.on("/ctrl", handleControlRequest);
  webServer.begin();
  Serial.println("Web Server Started");

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 1000, 2000);
  digitalWrite(15, LOW);
  digitalWrite(33, LOW);
}

void loop() {
  //before deployment
  
  if (ctrlCommand == "START" && millis() <= (lastResetTime + RESETTIME)) {
    Serial.println("reading sensor...");
    mySensorData = readData();
    mySensorDataList.add(mySensorData);
    if (stage == 0){
      if (mySensorData > 1.5 and deployed == False){
        for(int pos = servopos; pos >= 1200; pos-=1){
          myservo.writeMicroseconds(pos);
          delay(1);
        }
        deployed = True;
        servopos = 1200;
      }
      error = desired_depth - mySensorData;
      if (error < 0.1 and started == False){
        start_time = millis();
        started = True;
      }
      if (millis()-start_time >= 45){
        stage = 1;
      }
      integral += error;
      derivative = error - prev_error;
      control = p * error + i * integral + d * derivative;
      control = max(min(control, 1.0), -1);

      if (control > derivative){ // going down
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        delay(2000);
        digitalWrite(MOTOR_IN2, LOW);
      }
      else{ // going up
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN1, HIGH);
        delay(2000);
        digitalWrite(MOTOR_IN1, LOW);
      }
      prev_error = error;
      
    }
    else if (stage == 1){ // GOING BACK UP AFTER 45 SECONDS
      if (deployed == True){
        for(int pos = servopos; pos <= 1800; pos+=1){
          myservo.writeMicroseconds(pos);
          delay(1);
        }
        deployed = False;
        servopos = 1800;
      }

      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN1, HIGH);
      delay(2000);
      digitalWrite(MOTOR_IN1, LOW);

    }
  }
  else {
    Serial.println("reading sensor data on surface");
    mySensorData = readData();
    sendData(mySensorData);    
  }

  // Handle Client Requests
  webServer.handleClient();
  delay(2000);
}

