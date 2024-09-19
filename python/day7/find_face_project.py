#얼굴 찾기
import cv2
cap = cv2.VideoCapture(0)
face_cascade=cv2.CascadeClassifier(r'C:\Users\sally\Desktop\new\haarcascade_frontalface_default.xml')
while True:
    _, frame =cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    #30X30 같이 작은 얼굴은 무시한다
    print(faces)
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame )
    if cv2.waitKey(10) & 0xff ==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
