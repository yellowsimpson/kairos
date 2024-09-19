import cv2
cap = cv2.VideoCapture(0)
cap.set(3, 200)
cap.set(4, 100)
low = 50
high = 150
def callBack1(value1):
    pass
def callBack2(value2):
    pass
cv2.namedWindow("Style", cv2.WINDOW_NORMAL)
cv2.createTrackbar("sigma_s", "Style", 0, 300, callBack1)
cv2.createTrackbar("sigma_r", "Style", 0, 100, callBack2)
while True:
    _, frame =cap.read()
    cv2.imshow("Video", frame )
    sigma_s =cv2.getTrackbarPos("sigma_s", "Style")
    sigma_r =cv2.getTrackbarPos("sigma_r", "Style") / 100
    frame_style = cv2.stylization(frame, sigma_s=300, sigma_r=1)
    frame_style = cv2.stylization(frame, sigma_s=sigma_s, sigma_r=sigma_r)
    canny_frame = cv2.Canny(frame_style, low, high)
    cv2.imshow("Stylish", frame_style)
    if cv2.waitKey(10) & 0xff ==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()











