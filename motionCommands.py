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

# Main loop
run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                ws.send(json.dumps({"command": "F"}))
                print("Forward")
            elif event.key == pygame.K_s:
                ws.send(json.dumps({"command": "B"}))
                print("Backward")
            elif event.key == pygame.K_a:
                ws.send(json.dumps({"command": "L"}))
                print("Left")
            elif event.key == pygame.K_d:
                ws.send(json.dumps({"command": "R"}))
                print("Right")

        if event.type == pygame.KEYUP:
            ws.send(json.dumps({"command": "stop"}))

    win.fill((0, 0, 0))
    pygame.display.update()
    time.sleep(0.01)  # Add a small delay to reduce CPU usage

# Clean up
pygame.quit()
ws.close()