import cv2

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
  
    frame = cv2.flip(frame, -1)
    #영상 뒤집기
    yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    change_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    cv2.imshow("Video", frame)
    #영상 출력
    cv2.imshow("YUV Frame", yuv_frame)
    #YUV로 변환된 영상이 출력
    cv2.imshow("Change Frame", change_frame)
    #RGB로 변환된 영상이 출력
    
    if cv2.waitKey(10) & 0xff == ord('q'):
    #q버튼을 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()
