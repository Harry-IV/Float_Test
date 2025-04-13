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
  double depth;
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


// Pids initialization
String p_str, i_str, d_str;
double p = 0.5;
double i = 0.01;
double d = 0.1;
double desired_depth=2.5;
double prev_error=0;
double integral=0;
double derivative=0;
double error=0;
int stage=0;
double start_time=0;
bool deployed=false;
bool started=false;
double control=0;
double depth=0;
double prev_time=0;
String msg;



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

    int ind1 = ctrlCommand.indexOf(':');
    int ind2 = ctrlCommand.indexOf(':', ind1+1);
    int ind3 = ctrlCommand.indexOf(':', ind2+1);
    p_str = ctrlCommand.substring(ind1+1, ind2-2);
    i_str = ctrlCommand.substring(ind2+1, ind3-2);
    d_str = ctrlCommand.substring(ind3+1);
    p = p_str.toDouble();
    i = i_str.toDouble();
    d = d_str.toDouble();

    if (ctrlCommand.startsWith("START") || ctrlCommand.startsWith("STARTNOPIDS")) {  
      lastResetTime = millis();
      mySensorDataList.clear();
      msg = "Received START command. Starting ...";

      prev_time = millis();
    }
    else if (ctrlCommand.startsWith("STATUS")) {
      msg = "EX01 " + String(mySensorData.Time) + " sec " + String(mySensorData.depth) + " m " + " p: " + String(p) + " i: " + String(i) + " d: " + String(d);
    }
    else if (ctrlCommand.startsWith("UP")) {
      digitalWrite(MOTOR_IN2, HIGH);
      digitalWrite(MOTOR_IN1, LOW);
      delay(5000);
      digitalWrite(MOTOR_IN2, LOW);
      msg = "Going UP!";
    }
    else if (ctrlCommand.startsWith("DOWN")) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      delay(5000);
      digitalWrite(MOTOR_IN1, LOW);
      msg = "Going DOWN!";
    }
    else if (ctrlCommand.startsWith("DEPLOYFLAPS")){
      myservo.write(59);
      msg = "Deploying Flaps!";
    }
    else if (ctrlCommand.startsWith("RETRACTFLAPS")){
      myservo.write(90);
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
  
  if (ctrlCommand.startsWith("START")) {
    Serial.println("reading sensor...");
    mySensorData = readData();
    mySensorDataList.add(mySensorData);
    sensor.read();
    depth = sensor.depth();
    msg = "EX01 " + String(mySensorData.Time) + " sec " + String(depth) + " m ";
    webServer.send(200, "text/plain", msg);
    myservo.write(90);
    if (stage == 0){
      if (depth > 1.5 and deployed == false){
        //myservo.write(59);
        deployed = true;
      }
      error = desired_depth - depth;
      if (error < 0.1 and started == false){
        start_time = millis();
        started = true;
      }
      if (millis()-start_time >= 45000){
        stage = 1;
      }
      integral += error;
      derivative = (error - prev_error)/(millis()-prev_time);
      Serial.println(derivative);
      control = p * error + i * integral + d * derivative;
      control = max(min(control, 1.0), -1.0);

      prev_error = error;
      prev_time = millis();

      if (control > derivative){ // going down
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        delay(300);
        digitalWrite(MOTOR_IN1, LOW);
      }
      else{ // going up
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN1, LOW);
        delay(300);
        digitalWrite(MOTOR_IN2, LOW);
      }
      
    }
    else if (stage == 1){ // GOING BACK UP AFTER 45 SECONDS
      if (deployed == true){
        //myservo.write(90);
      }

      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN1, HIGH);
      delay(300);
      digitalWrite(MOTOR_IN1, LOW);
      

    }
  }
  else if (ctrlCommand.startsWith("STARTNOPIDS") && millis() <= (lastResetTime + RESETTIME)) {
    Serial.println("reading sensor...");
    mySensorData = readData();
    mySensorDataList.add(mySensorData);
    sensor.read();
    depth = sensor.depth();
    msg = "EX01 " + String(mySensorData.Time) + " sec " + String(depth) + " m ";
    webServer.send(200, "text/plain", msg);
    myservo.write(90);
    if (stage == 0){
      if (depth > 1.5 and deployed == false){
        //myservo.write(59);
        deployed = true;
      }
      error = desired_depth - depth;
      if (error < 0.1 and started == false){
        start_time = millis();
        started = true;
      }
      if (millis()-start_time >= 45000){
        stage = 1;
      }

      if (depth < 2.5){ // going down
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        delay(300);
        digitalWrite(MOTOR_IN1, LOW);
      }
      else{ // going up
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN1, LOW);
        delay(300);
        digitalWrite(MOTOR_IN2, LOW);
      }
      prev_error = error;
      
    }
    else if (stage == 1){ // GOING BACK UP AFTER 45 SECONDS
      if (deployed == true){
        //myservo.write(90);
        deployed = false;
      }

      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN1, HIGH);
      delay(300);
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
}

