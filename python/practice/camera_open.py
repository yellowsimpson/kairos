import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame =cap.read()   

    cv2.imshow("Video", frame )

    if cv2.waitKey(10) & 0xff ==ord('q'):
        break
