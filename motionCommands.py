import pygame
import websocket
import json
import time

# Setup WebSocket connection
try:
    ws = websocket.WebSocket()
    ws.connect("ws://192.168.1.13:80")  # Replace with your ESP32 IP and port
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

# Main loop
run = True
while run:
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

    win.fill((0, 0, 0))
    pygame.display.update()
    time.sleep(0.01)  # Add a small delay to reduce CPU usage

# Clean up
pygame.quit()
ws.close()
