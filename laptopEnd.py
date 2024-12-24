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

    cv.imshow("In", frame)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# Clean up
pygame.quit()
ws.close()
# add to GUI
