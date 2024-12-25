#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <PID_v1.h> // Include PID library

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
double speedA, speedB; // Measured speeds
double outputA, outputB; // PWM outputs
double setpoint = 300; // Target speed (ticks per second)

PID pidA(&speedA, &outputA, &setpoint, 0.0, 0.0, 0.0, DIRECT); // Tune kp, ki, kd
PID pidB(&speedB, &outputB, &setpoint, 0.0, 0.0, 0.0, DIRECT); // Tune kp, ki, kd

void enLeft() {
  counterA++;
}

void enRight() {
  counterB++;
}

// Motor control functions
void setMotorSpeeds() {
  analogWrite(enA, constrain(outputA, 0, 255));
  analogWrite(enB, constrain(outputB, 0, 255));
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

  // Initialize PID controllers
  pidA.SetMode(AUTOMATIC);
  pidB.SetMode(AUTOMATIC);
  pidA.SetOutputLimits(0, 255);
  pidB.SetOutputLimits(0, 255);
}

void loop() {
  // Speed calculation (ticks per second)
  static unsigned long prevMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= 100) { // Update every 100 ms
    speedA = counterA * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
    speedB = counterB * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
    counterA = 0;
    counterB = 0;
    prevMillis = currentMillis;

    // Compute PID outputs
    pidA.Compute();
    pidB.Compute();

    // Set motor speeds
    setMotorSpeeds();

    // Debug output
    Serial.print("Speed A: ");
    Serial.print(speedA);
    Serial.print(" | Speed B: ");
    Serial.print(speedB);
    Serial.print(" | Output A: ");
    Serial.print(outputA);
    Serial.print(" | Output B: ");
    Serial.println(outputB);
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
        setpoint = 300; // Adjust desired speed
      } else if (command == "B") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        setpoint = 300; // Adjust desired speed
      } else if (command == "L") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        setpoint = 200; // Lower speed for turn
      } else if (command == "R") {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        setpoint = 200; // Lower speed for turn
      } else if (command == "S") {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        setpoint = 0;
      } else {
        Serial.println("Unknown command");
      }
    }

    Serial.println("Client disconnected");
    setpoint = 0;  // Stop the robot when the client disconnects
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }

  delay(10);  // Optional small delay
}
