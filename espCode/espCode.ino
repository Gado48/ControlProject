#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

const char* ssid = "gado";
const char* password = "00009999";

WebsocketsServer server;

const int in1 = 2;
const int in2 = 3;
const int in3 = 5;
const int in4 = 6;

const int enA = 25;
const int enB = 26;

const int encoderA = 22;
const int encoderB = 23;

volatile long counterA = 0;
volatile long counterB = 0;

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
  analogWrite(enA, 255);  // Full speed
  analogWrite(enB, 255);  // Full speed
}

void backwardMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255);  // Full speed
  analogWrite(enB, 255);  // Full speed
}

void leftMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255);  // Full speed
  analogWrite(enB, 255);  // Full speed
}

void rightMotion() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255);  // Full speed
  analogWrite(enB, 255);  // Full speed
}

void stopMotion() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);  // Stop
  analogWrite(enB, 0);  // Stop
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
      } else {
        Serial.println("Unknown command");
      }
    }

    Serial.println("Client disconnected");
    stopMotion();  // Stop the robot when the client disconnects
  }

  delay(10);  // Optional small delay
}