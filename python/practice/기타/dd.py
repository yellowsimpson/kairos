import cv2

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
  
    frame = cv2.flip(frame, -1)
    yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    change_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    cv2.imshow("Video", frame)
    cv2.imshow("YUV Frame", yuv_frame)
    cv2.imshow("Change Frame", change_frame)
    
    if cv2.waitKey(10) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
