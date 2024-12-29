import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode
import pygame
import websocket
import json
import time

cap = cv.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Setup WebSocket connection
try:
    ws = websocket.WebSocket()
    ws.connect("ws://192.168.151.7:80")  # Replace with your ESP32 IP and port
except Exception as e:
    print(f"Error connecting to WebSocket: {e}")
    exit()

# Initialize Pygame
pygame.init()

# Set up display
width, height = 400, 400
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Robot Controller")

# Variables to track key states
current_command = None
selected_servo = 1  # Start with the first servo


# Function to send servo control commands
def send_servo_command(servo, direction):
    ws.send(json.dumps({"servo": servo, "direction": direction}))
    print(f"Servo {servo} moved {direction}")


while True:
    ret, frame = cap.read()

    for barcode in decode(frame):
        print(barcode.data)
        myData = barcode.data.decode("utf-8")
        print(myData)

        pts = np.array([barcode.polygon], np.int32)
        cv.polylines(frame, [pts], True, (255, 0, 0), 5)
        pts2 = barcode.rect
        cv.putText(
            frame,
            myData,
            (pts2[0], pts2[1]),
            cv.FONT_HERSHEY_COMPLEX,
            1,
            (255, 0, 0),
            2,
        )

    keys = pygame.key.get_pressed()  # Check the state of keys
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    # Check key states and send commands
    if keys[pygame.K_w]:
        if current_command != "F":
            ws.send(json.dumps({"command": "F"}))
            print("Forward")
            current_command = "F"
    elif keys[pygame.K_s]:
        if current_command != "B":
            ws.send(json.dumps({"command": "B"}))
            print("Backward")
            current_command = "B"
    elif keys[pygame.K_a]:
        if current_command != "L":
            ws.send(json.dumps({"command": "L"}))
            print("Left")
            current_command = "L"
    elif keys[pygame.K_d]:
        if current_command != "R":
            ws.send(json.dumps({"command": "R"}))
            print("Right")
            current_command = "R"
    else:
        if current_command is not None:  # If no keys are pressed, stop
            ws.send(json.dumps({"command": "S"}))
            print("Stop")
            current_command = None

    # Servo selection
    if keys[pygame.K_1]:
        selected_servo = 1
        print("Controlling Servo 1")
    elif keys[pygame.K_2]:
        selected_servo = 2
        print("Controlling Servo 2")
    elif keys[pygame.K_3]:
        selected_servo = 3
        print("Controlling Servo 3")

    # Servo angle adjustment
    if keys[pygame.K_UP]:
        send_servo_command(selected_servo, "up")
        time.sleep(0.05)  # Small delay for smooth control
    elif keys[pygame.K_DOWN]:
        send_servo_command(selected_servo, "down")
        time.sleep(0.05)

    win.fill((0, 0, 0))
    pygame.display.update()
    time.sleep(0.01)  # Add a small delay to reduce CPU usage

    cv.imshow("In", frame)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# Clean up
pygame.quit()
ws.close()
# add to GUI
