import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode
import pygame
import websocket
import json
import time

# Initialize video capture
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
width, height = 800, 600
win = pygame.display.set_mode((width, height), pygame.RESIZABLE)
pygame.display.set_caption("Robot Controller")

# Variables to track key states
current_command = None
selected_servo = 1  # Start with the first servo
robot_location = (0, 0)  # Robot's current location (x, y)
servo_angles = {"Servo 1": 90, "Servo 2": 90, "Servo 3": 90}  # Servo angles

# Font for Pygame GUI
font = pygame.font.SysFont("Arial", 24)


# Function to send servo control commands
def send_servo_command(servo, direction):
    ws.send(json.dumps({"servo": servo, "direction": direction}))
    print(f"Servo {servo} moved {direction}")


# Function to send motion commands
def send_motion_command(command):
    ws.send(json.dumps({"command": command}))
    print(f"Motion command: {command}")


# Function to send camera coordinates
def send_camera_coordinates(x, y):
    ws.send(json.dumps({"camera": {"x": x, "y": y}}))
    print(f"Sent camera coordinates: ({x}, {y})")


# Function to update robot location on the GUI
def update_robot_location(x, y):
    global robot_location
    robot_location = (x, y)


# Function to update servo angles
def update_servo_angles(servo, angle):
    global servo_angles
    servo_angles[servo] = angle


# Function to convert OpenCV frame to Pygame surface and resize it to fit the window
def cv2_to_pygame(frame, target_size):
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  # Convert BGR to RGB
    frame = cv.resize(frame, target_size)  # Resize to fit the window
    frame = np.rot90(frame)  # Rotate 90 degrees
    frame = pygame.surfarray.make_surface(frame)
    return frame


# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    # Decode barcodes in the frame
    for barcode in decode(frame):
        print(barcode.data)
        myData = barcode.data.decode("utf-8")
        print(myData)

        # Extract barcode coordinates
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

        # Send barcode coordinates to ESP32
        send_camera_coordinates(pts2[0], pts2[1])

    # Handle Pygame events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.VIDEORESIZE:  # Handle window resizing
            width, height = event.size
            win = pygame.display.set_mode((width, height), pygame.RESIZABLE)

    # Check key states and send commands
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        if current_command != "F":
            send_motion_command("F")
            current_command = "F"
    elif keys[pygame.K_s]:
        if current_command != "B":
            send_motion_command("B")
            current_command = "B"
    elif keys[pygame.K_a]:
        if current_command != "L":
            send_motion_command("L")
            current_command = "L"
    elif keys[pygame.K_d]:
        if current_command != "R":
            send_motion_command("R")
            current_command = "R"
    else:
        if current_command is not None:  # If no keys are pressed, stop
            send_motion_command("S")
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

    # Update GUI
    win.fill((0, 0, 0))  # Clear the screen

    # Display camera feed in full window
    frame_pygame = cv2_to_pygame(frame, (width, height))  # Resize to fit the window
    win.blit(
        frame_pygame, (0, 0)
    )  # Position the camera feed to cover the entire window

    # Display instructions on the top left
    instructions = [
        "W: Forward",
        "S: Backward",
        "A: Left",
        "D: Right",
        "1/2/3: Select Servo",
        "Up/Down: Adjust Servo",
        "Q: Quit",
    ]
    for i, text in enumerate(instructions):
        instruction_text = font.render(text, True, (255, 255, 255))
        win.blit(
            instruction_text, (10, 10 + i * 30)
        )  # Position instructions on the top left

    # Display current states, position, and servo angles on the top right
    states = [
        f"Current Command: {current_command}",
        f"Selected Servo: {selected_servo}",
        f"Robot Location: ({robot_location[0]}, {robot_location[1]})",
        f"Servo 1 Angle: {servo_angles['Servo 1']}",
        f"Servo 2 Angle: {servo_angles['Servo 2']}",
        f"Servo 3 Angle: {servo_angles['Servo 3']}",
    ]
    for i, text in enumerate(states):
        state_text = font.render(text, True, (255, 255, 255))
        win.blit(
            state_text, (width - 300, 10 + i * 30)
        )  # Position states on the top right

    pygame.display.update()

    # Exit on 'Q' key press
    if keys[pygame.K_q]:
        break

    time.sleep(0.01)  # Add a small delay to reduce CPU usage

# Clean up
pygame.quit()
ws.close()
cap.release()
cv.destroyAllWindows()
