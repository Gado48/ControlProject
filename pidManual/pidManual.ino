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

// PID variables
float kp = 2.0, ki = 0.5, kd = 1.0; // Tune these values
float integralA = 0, integralB = 0;
float prevErrorA = 0, prevErrorB = 0;
float desiredSpeed = 300; // Target speed (encoder ticks per second)

void enLeft() {
  counterA++;
}

void enRight() {
  counterB++;
}

// Calculate PID-adjusted PWM
int calculatePID(float& integral, float& prevError, float currentSpeed, float targetSpeed) {
  float error = targetSpeed - currentSpeed;
  integral += error;
  float derivative = error - prevError;
  prevError = error;
  return constrain(kp * error + ki * integral + kd * derivative, 0, 255);
}

// Motor control functions with adjustable speed
void setMotorSpeeds(int speedA, int speedB) {
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);
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
  // Speed calculation (ticks per second)
  static unsigned long prevMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= 100) { // Update every 100 ms
    float speedA = counterA * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
    float speedB = counterB * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
    counterA = 0;
    counterB = 0;
    prevMillis = currentMillis;

    // Adjust speeds using PID
    int pwmA = calculatePID(integralA, prevErrorA, speedA, desiredSpeed);
    int pwmB = calculatePID(integralB, prevErrorB, speedB, desiredSpeed);

    setMotorSpeeds(pwmA, pwmB);

    Serial.print("Speed A: ");
    Serial.print(speedA);
    Serial.print(" | Speed B: ");
    Serial.println(speedB);
  }

  // WebSocket handling
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
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        desiredSpeed = 300; // Adjust desired speed
      } else if (command == "B") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        desiredSpeed = 300; // Adjust desired speed
      } else if (command == "L") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        desiredSpeed = 200; // Lower speed for turn
      } else if (command == "R") {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        desiredSpeed = 200; // Lower speed for turn
      } else if (command == "S") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        desiredSpeed = 0;
      } else {
        Serial.println("Unknown command");
      }
    }

    Serial.println("Client disconnected");
    desiredSpeed = 0;  // Stop the robot when the client disconnects
    setMotorSpeeds(0, 0);
  }

  delay(10);  // Optional small delay
}
