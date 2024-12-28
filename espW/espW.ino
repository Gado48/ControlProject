#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// Wi-Fi credentials
const char* ssid = "ssid";        // Replace with your Wi-Fi SSID
const char* password = "password"; // Replace with your Wi-Fi password

// WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// FreeRTOS queue for commands
QueueHandle_t commandQueue;

// Motor control pins (adjust these as needed)
#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 6
#define MOTOR_LEFT_PIN 7
#define MOTOR_RIGHT_PIN 8

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.println("Connecting to Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    // Create a queue to handle commands
    commandQueue = xQueueCreate(10, sizeof(String));

    // Initialize motor control pins
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN, OUTPUT);

    // Create tasks
    xTaskCreatePinnedToCore(webSocketTask, "WebSocket Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motorControlTask, "Motor Control Task", 2048, NULL, 2, NULL, 1);
}

void loop() {
    // FreeRTOS will handle the tasks; the loop can remain empty.
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        String command = String((char *)payload);
        Serial.println("Received command: " + command); // Debugging line

        // Send the command to the motor control queue
        if (xQueueSend(commandQueue, &command, 0) != pdPASS) {
            Serial.println("Queue is full! Command discarded.");
        }
    }
}

// Task to handle WebSocket communication
void webSocketTask(void *pvParameters) {
    while (true) {
        webSocket.loop();  // Handle WebSocket communication
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield control to FreeRTOS
    }
}

// Task to handle motor control
void motorControlTask(void *pvParameters) {
    String command;

    while (true) {
        // Wait for a command from the queue
        if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
            Serial.println("Executing Command: " + command);
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
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to FreeRTOS
    }
}

// Motor control functions
void forwardMotion() {
    Serial.println("Moving Forward");
    digitalWrite(MOTOR_FORWARD_PIN, HIGH );
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Example 1-second motion
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
}

void backwardMotion() {
    Serial.println("Moving Backward");
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
}

void leftMotion() {
    Serial.println("Turning Left");
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(MOTOR_LEFT_PIN, LOW);
}

void rightMotion() {
    Serial.println("Turning Right");
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
}

void stopMotion() {
    Serial.println("Stopping");
    // Stop all motors
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
}