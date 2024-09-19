import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3,100)
cap.set(4,200)


def callback(value):
    pass

cv2.namedWindow('track', cv2.WINDOW_NORMAL)

hsv_1 = np.load('savefile.npy')
low = hsv_1[0]
high = hsv_1[1]

cv2.createTrackbar('low_h', 'track', low[0], 360, callback)
cv2.createTrackbar('low_s', 'track', low[1], 360, callback)
cv2.createTrackbar('low_v', 'track', low[2], 360, callback)
cv2.createTrackbar('high_h', 'track', high[0], 360, callback)
cv2.createTrackbar('high_s', 'track', high[1], 360, callback)
cv2.createTrackbar('high_v', 'track', high[2], 360, callback)


while True:
    
    _, frame = cap.read()

    low_h = cv2.getTrackbarPos('low_h', 'track')
    low_s = cv2.getTrackbarPos('low_s', 'track')
    low_v = cv2.getTrackbarPos('low_v', 'track')
    high_h = cv2.getTrackbarPos('high_h', 'track')
    high_s = cv2.getTrackbarPos('high_s', 'track')
    high_v = cv2.getTrackbarPos('high_v', 'track')
    
    
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    low = np.array([low_h, low_s, low_v])
    high = np.array([high_h, high_s, high_v])
    
    mask = cv2.inRange(hsv_img, low, high)
    

    kernel = np.ones((5, 5), np.uint8) # 노이즈필터
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_contour) > 1000:
            cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 20, (0, 255, 0), -1)
        print(max_contour)
    
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    cv2.imshow('track', result)
    # print(low, high)
    
    hsv = [low, high]
    # print(type(hsv))
    
    if cv2.waitKey(10) & 0xff == ord('q'):
        break
    elif cv2.waitKey(10) & 0xff == ord('s'):
        np.save('savefile', hsv)

cap.release()
cv2.destroyAllWindows()