'''
# 얼굴 찾기
import cv2
cap = cv2.VideoCapture(0)
face_cascade=cv2.CascadeClassifier(r'day7\haarcascade_eye.xml')

while True:
    _, frame =cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=2, minNeighbors=8, minSize=(30, 30))
    #30X30 같이 작은 얼굴은 무시한다
    print(faces)
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),3)
    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame)
    if cv2.waitKey(10) & 0xff ==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

'''
import cv2
import random

cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier(r'day7\haarcascade_eye.xml')

# 미리 정의된 색상 리스트
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255)]

color_index = 0

while True:
    _, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=2, minNeighbors=6, minSize=(30, 30))
    # 30x30 같이 작은 얼굴은 무시한다
    print(faces)

    for (x, y, w, h) in faces:
        # 색상 리스트에서 색상을 순환하면서 선택
        color = colors[color_index % len(colors)]
        color_index += 1
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, -1)

    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame)

    if cv2.waitKey(10) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
