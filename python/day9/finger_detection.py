import cv2
import sys
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Video open failed!')
    sys.exit()
ret, back = cap.read()
if not ret:
    print('Background image registration failed!')
    sys.exit()
back = cv2.cvtColor(back, cv2.COLOR_BGR2GRAY)
back = cv2.GaussianBlur(back, (3, 3), 0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    diff = cv2.absdiff(gray, back)
    _, diff = cv2.threshold(diff, 127, 255, cv2.THRESH_BINARY)
    cnt, _, stats, _ = cv2.connectedComponentsWithStats(diff)
    for i in range(1, cnt):
        print("detect")
        x, y, w, h, s = stats[i]
        if s < 100:
            continue
        cv2.rectangle(frame, (x, y, w, h), (0, 0, 255), 2)
    cv2.imshow('frame', frame)
    cv2.imshow('diff', diff)
    if cv2.waitKey(30) == 27:
        break
    back = gray.copy()
cap.release()
cv2.destroyAllWindows()
