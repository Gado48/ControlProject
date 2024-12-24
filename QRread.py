import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

cap = cv.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

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

    cv.imshow("In", frame)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break
