import cv2
import numpy as np

def pair_eyes(eyes):
    paired_eyes = []
    eyes = sorted(eyes, key=lambda x: x[0])  # x 좌표 기준으로 정렬
    i = 0
    while i < len(eyes) - 1:
        # 현재 눈과 다음 눈이 충분히 가까운지 확인하여 쌍으로 묶기
        x1, y1, w1, h1 = eyes[i]
        x2, y2, w2, h2 = eyes[i + 1]
        if abs(x2 - (x1 + w1)) < w1:
            paired_eyes.append(((x1, y1, w1, h1), (x2, y2, w2, h2)))
            i += 2
        else:
            i += 1
    return paired_eyes

cap = cv2.VideoCapture(0)
eye_cascade = cv2.CascadeClassifier(r'day7\haarcascade_eye.xml')

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    eyes = eye_cascade.detectMultiScale(gray_frame, scaleFactor=2.0, minNeighbors=5, minSize=(30, 30))
    print(eyes)

    paired_eyes = pair_eyes(eyes)

    for (eye1, eye2) in paired_eyes:
        x1, y1, w1, h1 = eye1
        x2, y2, w2, h2 = eye2
        top_left_x = min(x1, x2)
        top_left_y = min(y1, y2)
        bottom_right_x = max(x1 + w1, x2 + w2)
        bottom_right_y = max(y1 + h1, y2 + h2)

        cv2.rectangle(frame, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 0, 0), -1)


    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows() 

