import cv2
print(cv2.__version__)

cap = cv2.VideoCapture(0)
cap.set(10,10000)
#카메라 창 크기 설정 (x축)
cap.set(4, 100)
#카메라 창 크기 설정 (y축)
low = 50
high =150

while True:
    ret, frame = cap.read()
# ret =읽은거 ghkrdls, frame = 읽은 값 알려준거

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny_frame = cv2.Canny(gray_frame, low, high)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    blur_frame = cv2.Gaussianblur(frame, (5,5), 10)

    # print(ret)
    
    cv2.imshow("Video", frame)
    cv2.imshow("Gray", gray_frame)
    cv2.imshow("RGB", rgb_frame)
    cv2.imshow("Canny", canny_frame)
    cv2.imshow("blur", blur_frame)

    if cv2.waitKey(10) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
