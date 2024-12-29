#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ESP32Servo.h>

using namespace websockets;

const char* ssid = "ssid";
const char* password = "password";

WebsocketsServer server;

const int in1 = 15;
const int in2 = 16;
const int in3 = 17;
const int in4 = 18;

const int enA = 25;
const int enB = 26;

const int encoderA = 22;
const int encoderB = 23;

volatile long counterA = 0;
volatile long counterB = 0;

Servo shoulder;
Servo elbow;
Servo gripper;

int shoulderAngle = 90; // Initial angle for Servo 1
int elbowAngle = 90; // Initial angle for Servo 2
int gripperAngle = 90; // Initial angle for Servo 3


void enLeft() {
  counterA++;
}

void enRight() {
  counterB++;
}

void forwardMotion() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); //full speed
  analogWrite(enB, 255); //full speed
}

void backwardMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); //full speed
  analogWrite(enB, 255); //full speed
}

void leftMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); //full speed
  analogWrite(enB, 255); //full speed
}

void rightMotion() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); //full speed
  analogWrite(enB, 255); //full speed
}

void stopMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0); 
  analogWrite(enB, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), enLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), enRight, RISING);

  // Initialize servos
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	shoulder.setPeriodHertz(50);    // standard 50 hz servo
  elbow.setPeriodHertz(50);    // standard 50 hz servo
  gripper.setPeriodHertz(50);    // standard 50 hz servo
	shoulder.attach(20, 1000, 2000); // attaches the servo on pin 18 to the servo object
  elbow.attach(21, 1000, 2000); // attaches the servo on pin 18 to the servo object
  gripper.attach(22, 1000, 2000); // attaches the servo on pin 18 to the servo object

  shoulder.write(shoulderAngle);
  elbow.write(elbowAngle);
  gripper.write(gripperAngle);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print the ESP32's IP address
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the WebSocket server
  server.listen(80);  // Port 80
  Serial.println("WebSocket server started on port 80");
}

void loop() {
  // Check for new clients
  if (server.poll()) {  // Check if there's an incoming connection
    WebsocketsClient client = server.accept();
    Serial.println("New client connected");

    while (client.available()) {  // Handle messages from the client
      WebsocketsMessage message = client.readBlocking();
      String command = message.data();
      Serial.print("Received command: ");
      Serial.println(command);

      // Execute motion based on received command
      if (command == "F") {
        forwardMotion();
      } else if (command == "B") {
        backwardMotion();
      } else if (command == "L") {
        leftMotion();
      } else if (command == "R") {
        rightMotion();
      } else if (command == "S") {
        stopMotion();
      }
      else if (command.startsWith("servo 1")) {
        String direction = command.substring(6);
        if (direction == "up") {
          shoulderAngle += 5;
          if (shoulderAngle >= 180){
            shoulderAngle = 180;
          }
        } else if (direction == "down") {
          shoulderAngle -= 5;
          if (shoulderAngle <= 0){
            shoulderAngle = 0;
          }
        }
        shoulder.write(shoulderAngle);
        Serial.print("shoulder angle: ");
        Serial.println(shoulderAngle);
      } else if (command.startsWith("servo 2")) {
        String direction = command.substring(6);
        if (direction == "up") {
          elbowAngle += 5;
          if (elbowAngle >= 180){
            elbowAngle = 180;
          }
        } else if (direction == "down") {
          elbowAngle -= 5;
          if (elbowAngle <= 0){
            elbowAngle = 0;
          }
        }
        elbow.write(elbowAngle);
        Serial.print("elbow angle: ");
        Serial.println(elbowAngle);
      } else if (command.startsWith("servo 3")) {
        String direction = command.substring(6);
        if (direction == "up") {
          gripperAngle += 5;
          if (gripperAngle >= 180){
            gripperAngle = 180;
          }
        } else if (direction == "down") {
          gripperAngle -= 5;
          if (gripperAngle <= 0){
            gripperAngle = 0;
          }
        }
        gripper.write(gripperAngle);
        Serial.print("Servo 3 angle: ");
        Serial.println(gripperAngle);
      } else {
        Serial.println("Unknown command");
      }
    }

    Serial.println("Client disconnected");
    stopMotion();
  }

  delay(10);  // Optional small delay
}