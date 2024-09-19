import cv2

# 카메라 준비
cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

# cap.set(3, 640)  # width
# cap.set(4, 480)  # height

# 카메라 읽기
while True:
    # 프레임을 이미지로 읽어들임
    _, frame = cap.read()
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        # 얼굴 영역을 잘라서 그 부분에서 눈을 찾기
        roi_gray = gray_frame[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        eyes = eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=5, minSize=(10, 10), maxSize=(50, 50))

        if len(eyes) >= 2:
            eye_list = sorted(eyes, key=lambda eye: eye[0])
            left_eye = eye_list[0]
            right_eye = eye_list[1]

            eye_x1 = left_eye[0]
            eye_y1 = left_eye[1]
            eye_x2 = right_eye[0] + right_eye[2]
            eye_y2 = right_eye[1] + right_eye[3]

            cv2.rectangle(roi_color, (eye_x1, eye_y1), (eye_x2, eye_y2), (0, 0, 0), -1)

    # 결과 보여주기
    cv2.imshow('Video', frame)

    # 키보드 q가 눌렸을 때,
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()