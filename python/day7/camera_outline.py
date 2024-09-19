import cv2
print(cv2.__version__)
cap = cv2.VideoCapture(0) #어디에 있는 카메라 쓸거야
eye_cascade = cv2.CascadeClassifier(r'C:\Users\gwonh\Desktop\python\haarcascade_eye.xml')#r을 넣던가 \\넣기

while True:
    _, frame=cap.read()
    frame=cv2.flip(frame,1)

    cv2.imshow("Video", frame)
      
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
     # 얼굴 찾기
    eyes1 = eye_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    # eyes2 = eye_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # print(len(eyes1))
    # print(len(eyes2))
    ans = []
    for i in range(len(eyes1)):
        check = False
        for j in range(len(eyes1)):
            if i == j:
                continue
            check_eye = eyes1[j]
            eye = eyes1[i]
            if abs(check_eye[0]-eye[0]) < 100 and abs(check_eye[1]-eye[1]) < 10:
                print(abs(check_eye[0]-eye[0]), abs(check_eye[1]-eye[1]))
                check = True
                break

        if check:
            ans.append(eye)

    for eye in ans:
        cv2.rectangle(frame, (eye[0],eye[1]), (eye[0] + eye[2], eye[1]+ eye[3]), (0, 255, 0), 2)
    # for eye in eyes2:
    #cv2.rectangle(frame, (eye[0],eye[1]), (eye[0] + eye[2], eye[1]+ eye[3]), (0, 255, 0), 2)

    
        
    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame)

    if cv2.waitKey(10) & 0xff == ord('q'): #10 frame 단위로 키보드 q 눌렀을때 break
        break

cap.release()
cv2.destroyAllWindows()
