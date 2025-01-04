#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <TinyMPU6050.h>

using namespace websockets;

// WiFi credentials
const char* ssid = "ssid";
const char* password = "password";

// WebSocket server
WebsocketsServer server;

// Motor pins
const int in1 = 14;
const int in2 = 27;
const int in3 = 26;
const int in4 = 25;
const int enA = 22;
const int enB = 23;

const int freq = 5000;
const int resolution = 8;

// Encoder pins
const int encoderA1 = 17;  // Encoder A channel 1 (Motor 1)
const int encoderA2 = 15;  // Encoder A channel 2 (Motor 1)
const int encoderB1 = 19;  // Encoder B channel 1 (Motor 2)
const int encoderB2 = 21;  // Encoder B channel 2 (Motor 2)

volatile long counterA = 0;  // Encoder count for Motor 1
volatile long counterB = 0;  // Encoder count for Motor 2

// Servo pins
Servo shoulder;
Servo elbow;
Servo gripper;
int shoulderAngle = 90; // Initial angle for Servo 1
int elbowAngle = 90;    // Initial angle for Servo 2
int gripperAngle = 90;  // Initial angle for Servo 3

// PID variables for Motor A
double errorA = 0;
double integralA = 0;
double derivativeA = 0;
double lastErrorA = 0;
double outputA = 0;

// PID variables for Motor B
double errorB = 0;
double integralB = 0;
double derivativeB = 0;
double lastErrorB = 0;
double outputB = 0;

// PID constants (set directly in the code)
double kp = 1.0;  // Proportional gain
double ki = 0.0;  // Integral gain
double kd = 0.0;  // Derivative gain

// Target coordinates
float x_target_pickup = 0;
float y_target_pickup = 0;
float x_target_dropoff = 0;
float y_target_dropoff = 0;
float x_current = 0; // Current robot position (x)
float y_current = 0; // Current robot position (y)
float robot_heading = 0; // Robot's heading in radians

// Odometry constants
const float wheel_radius = 3.25; // Wheel radius in cm
const float wheel_circumference = 2 * 3.14159 * wheel_radius; // Wheel circumference
const float ticks_per_revolution = 1000; // 1000 ticks per revolution
const float wheel_base = 19.0; // Distance between wheels in cm

// MPU6050 object
MPU6050 mpu;

// Sensor fusion variables
float gyro_yaw = 0; // Yaw angle from gyroscope
unsigned long last_time = 0; // Last time for gyroscope integration

// Robot state
enum RobotState {
    MANUAL_MODE,
    AUTONOMOUS_PICKUP,
    AUTONOMOUS_DROPOFF
};

RobotState robotState = MANUAL_MODE;  // Start in MANUAL_MODE mode

// Function to handle encoder interrupts for Motor 1
void IRAM_ATTR encoderA1ISR() {
    int stateA1 = digitalRead(encoderA1);
    int stateA2 = digitalRead(encoderA2);
    if (stateA1 == HIGH) {
        if (stateA2 == LOW) counterA++;
        else counterA--;
    } else {
        if (stateA2 == LOW) counterA--;
        else counterA++;
    }
}

// Function to handle encoder interrupts for Motor 2
void IRAM_ATTR encoderB1ISR() {
    int stateB1 = digitalRead(encoderB1);
    int stateB2 = digitalRead(encoderB2);
    if (stateB1 == HIGH) {
        if (stateB2 == LOW) counterB++;
        else counterB--;
    } else {
        if (stateB2 == LOW) counterB--;
        else counterB++;
    }
}

// Motor control functions
void setMotorSpeeds() {
    ledcWrite(enA, constrain(outputA, 0, 255));
    ledcWrite(enB, constrain(outputB, 0, 255));
}

void forwardMotion() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void backwardMotion() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void leftMotion() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void rightMotion() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void stopMotion() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

// Function to compute PID output for Motor A
void computePIDA(double setpointA, double speedA) {
    errorA = setpointA - speedA; // Calculate error
    integralA += errorA; // Accumulate integral
    derivativeA = errorA - lastErrorA; // Calculate derivative

    // Compute PID output
    outputA = (kp * errorA) + (ki * integralA) + (kd * derivativeA);

    // Constrain output to PWM range (0-255)
    outputA = constrain(outputA, 0, 255);

    // Save current error for next iteration
    lastErrorA = errorA;
}

// Function to compute PID output for Motor B
void computePIDB(double setpointB, double speedB) {
    errorB = setpointB - speedB; // Calculate error
    integralB += errorB; // Accumulate integral
    derivativeB = errorB - lastErrorB; // Calculate derivative

    // Compute PID output
    outputB = (kp * errorB) + (ki * integralB) + (kd * derivativeB);

    // Constrain output to PWM range (0-255)
    outputB = constrain(outputB, 0, 255);

    // Save current error for next iteration
    lastErrorB = errorB;
}

// Function to move the arm to the target coordinates
void move_arm_to_target() {
    shoulderAngle = 120;  
    elbowAngle = 150;     
    gripperAngle = 66;

    // Move the arm
    shoulder.write(shoulderAngle);
    elbow.write(elbowAngle);
    gripper.write(gripperAngle);
}

void lift_arm(){
    shoulderAngle = 90;  
    elbowAngle = 180;     
    gripperAngle = 66;    

    // Move the arm
    shoulder.write(shoulderAngle);
    elbow.write(elbowAngle);
    gripper.write(gripperAngle);
}

// Function to update robot position using odometry
void update_position() {
    static unsigned long last_update_time = 0;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_update_time) / 1000.0; // Convert to seconds

    // Calculate distance traveled by each wheel
    float distanceA = (counterA / ticks_per_revolution) * wheel_circumference;
    float distanceB = (counterB / ticks_per_revolution) * wheel_circumference;

    // Calculate linear and angular displacement
    float linear_displacement = (distanceA + distanceB) / 2.0;
    float angular_displacement = (distanceB - distanceA) / wheel_base;

    // Update robot position
    x_current += linear_displacement * cos(robot_heading);
    y_current += linear_displacement * sin(robot_heading);
    
    // Update robot heading using MPU6050 gyroscope
    mpu.Execute(); // Update MPU6050 data
    float gyro_z = mpu.GetGyroZ(); // Get angular velocity around the Z-axis (yaw)
    gyro_yaw += gyro_z * delta_time; // Integrate angular velocity to get yaw angle
    
    // Fuse encoder and gyroscope data for heading
    robot_heading = gyro_yaw; // Use gyroscope for heading
    
    // Reset encoder counts
    counterA = 0;
    counterB = 0;

    // Update last update time
    last_update_time = current_time;

    // Send updated position to the GUI
    send_robot_position();
}

// Function to send robot position to the GUI
void send_robot_position() {
    StaticJsonDocument<200> doc;
    doc["type"] = "position";
    doc["x"] = x_current;
    doc["y"] = y_current;
    doc["heading"] = robot_heading;

    String json;
    serializeJson(doc, json);
    server.broadcast(json);
}

// Function to move the robot towards the target coordinates
void move_to_target() {
    if (robotState == AUTONOMOUS_PICKUP) {
        // Move to pickup position
        float x_diff = x_target_pickup - x_current;
        float y_diff = y_target_pickup - y_current;

        setpointA = x_diff;
        setpointB = y_diff;

        computePIDA(setpointA, speedA);
        computePIDB(setpointB, speedB);
        setMotorSpeeds();

        // Check if reached pickup position
        if (abs(x_diff) < 5 && abs(y_diff) < 5) {
            stopMotion();
            move_arm_to_target();  // Pick up the box
            lift_arm();
            robotState = AUTONOMOUS_DROPOFF;  // Switch to drop-off state
        }
    } else if (robotState == AUTONOMOUS_DROPOFF) {
        // Move to drop-off position
        float x_diff = x_target_dropoff - x_current;
        float y_diff = y_target_dropoff - y_current;

        setpointA = x_diff;
        setpointB = y_diff;

        computePIDA(setpointA, speedA);
        computePIDB(setpointB, speedB);
        setMotorSpeeds();

        // Check if reached drop-off position
        if (abs(x_diff) < 5 && abs(y_diff) < 5) {
            stopMotion();
            // Drop the box (move arm to drop position)
            shoulderAngle = 120;
            elbowAngle = 150;
            gripperAngle = 180;
            shoulder.write(shoulderAngle);
            elbow.write(elbowAngle);
            gripper.write(gripperAngle);
            robotState = MANUAL_MODE;  // Switch back to MANUAL_MODE mode
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize motor pins
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    ledcAttach(enA, freq, resolution);
    ledcAttach(enB, freq, resolution);

    // Initialize encoder pins
    pinMode(encoderA1, INPUT_PULLUP);
    pinMode(encoderA2, INPUT_PULLUP);
    pinMode(encoderB1, INPUT_PULLUP);
    pinMode(encoderB2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA1), encoderA1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB1), encoderB1ISR, CHANGE);

    // Initialize servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    shoulder.setPeriodHertz(50);    // standard 50 hz servo
    elbow.setPeriodHertz(50);       // standard 50 hz servo
    gripper.setPeriodHertz(50);     // standard 50 hz servo
    shoulder.attach(13, 1000, 2000); // attaches the servo on pin 13 to the servo object
    elbow.attach(15, 1000, 2000);    // attaches the servo on pin 15 to the servo object
    gripper.attach(19, 1000, 2000);  // attaches the servo on pin 19 to the servo object

    shoulder.write(shoulderAngle);
    elbow.write(elbowAngle);
    gripper.write(gripperAngle);

    // Initialize MPU6050
    mpu.Initialize();
    mpu.Calibrate(); // Calibrate the MPU6050

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
        speedA = counterA * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
        speedB = counterB * (1000.0 / (currentMillis - prevMillis)); // Ticks per second
        counterA = 0;
        counterB = 0;
        prevMillis = currentMillis;

        // Compute PID outputs
        computePIDA(setpointA, speedA);
        computePIDB(setpointB, speedB);

        // Set motor speeds
        setMotorSpeeds();

        // Update robot position using odometry
        update_position();
    }

    // Check for new WebSocket messages
    if (server.poll()) {
        WebsocketsClient client = server.accept();
        Serial.println("New client connected");

        while (client.available()) {
            WebsocketsMessage message = client.readBlocking();
            String command = message.data();
            Serial.print("Received command: ");
            Serial.println(command);

            // Parse JSON command
            StaticJsonDocument<200> doc;  // Adjust the size as needed
            DeserializationError error = deserializeJson(doc, command);

            if (error) {
                Serial.println("Failed to parse JSON command");
                continue;
            }

            // Extract the command from JSON
            if (doc.containsKey("target")) {
                // Extract target coordinates
                JsonObject target = doc["target"];
                x_target_pickup = target["x"];
                y_target_pickup = target["y"];
                const char* target_type = target["type"];
                Serial.print("Received target coordinates: (");
                Serial.print(x_target_pickup);
                Serial.print(", ");
                Serial.print(y_target_pickup);
                Serial.print(") for ");
                Serial.println(target_type);

                if (strcmp(target_type, "pickup") == 0) {
                    robotState = AUTONOMOUS_PICKUP;  // Switch to pickup mode
                }
            } else if (doc.containsKey("command")) {
                const char* cmd = doc["command"];
                if (strcmp(cmd, "break_auto") == 0) {
                    robotState = MANUAL_MODE;  // Break autonomous mode
                    Serial.println("Autonomous mode stopped");
                } else if (strcmp(cmd, "autonomous_pickup") == 0) {
                    robotState = AUTONOMOUS_PICKUP;  // Switch to pickup mode
                    Serial.println("Moving to pickup coordinates");
                } else if (robotState == MANUAL_MODE) {
                    if (strcmp(cmd, "F") == 0) {
                        forwardMotion();
                    } else if (strcmp(cmd, "B") == 0) {
                        backwardMotion();
                    } else if (strcmp(cmd, "L") == 0) {
                        leftMotion();
                    } else if (strcmp(cmd, "R") == 0) {
                        rightMotion();
                    } else if (strcmp(cmd, "S") == 0) {
                        stopMotion();
                    } else if (doc.containsKey("servo")) {
                        int servo = doc["servo"];
                        const char* direction = doc["direction"];
                        if (servo == 1) {
                            if (strcmp(direction, "up") == 0) {
                                shoulderAngle += 5;
                                if (shoulderAngle >= 180) shoulderAngle = 180;
                            } else if (strcmp(direction, "down") == 0) {
                                shoulderAngle -= 5;
                                if (shoulderAngle <= 0) shoulderAngle = 0;
                            }
                            shoulder.write(shoulderAngle);
                        } else if (servo == 2) {
                            if (strcmp(direction, "up") == 0) {
                                elbowAngle += 5;
                                if (elbowAngle >= 180) elbowAngle = 180;
                            } else if (strcmp(direction, "down") == 0) {
                                elbowAngle -= 5;
                                if (elbowAngle <= 0) elbowAngle = 0;
                            }
                            elbow.write(elbowAngle);
                        } else if (servo == 3) {
                            if (strcmp(direction, "up") == 0) {
                                gripperAngle += 5;
                                if (gripperAngle >= 180) gripperAngle = 180;
                            } else if (strcmp(direction, "down") == 0) {
                                gripperAngle -= 5;
                                if (gripperAngle <= 0) gripperAngle = 0;
                            }
                            gripper.write(gripperAngle);
                        }
                    } else {
                        Serial.println("Unknown command");
                    }
                }
            }
        }

        Serial.println("Client disconnected");
    }

    // Move towards the target if in autonomous mode
    if (robotState != MANUAL_MODE) {
        move_to_target();
    }

}

