import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3,100)
cap.set(4,200)


def callback(value):
    pass

cv2.namedWindow('track', cv2.WINDOW_NORMAL)

cv2.createTrackbar('low_h', 'track', 0, 255, callback)
cv2.createTrackbar('low_s', 'track', 0, 255, callback)
cv2.createTrackbar('low_v', 'track', 0, 255, callback)
cv2.createTrackbar('high_h', 'track', 255, 255, callback)
cv2.createTrackbar('high_s', 'track', 255, 255, callback)
cv2.createTrackbar('high_v', 'track', 255, 255, callback)

np.save("파일 이름", hsv)

while True:
    
    _, frame = cap.read()

    low_h = cv2.getTrackbarPos('low_h', 'track')
    low_s = cv2.getTrackbarPos('low_s', 'track')
    low_v = cv2.getTrackbarPos('low_v', 'track')
    high_h = cv2.getTrackbarPos('high_h', 'track')
    high_s = cv2.getTrackbarPos('high_s', 'track')
    high_v = cv2.getTrackbarPos('high_v', 'track')
    
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2LUV)
    #HSV

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
            cv2.drawContours(frame, [max_contour], -1, (0, 255,0), 2)

            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(max_contour) > 1000:
                    cv2.drawContours(frame, [max_contour], -1, (0, 255,0), 2)
                        # 윤곽선의 무게 중심 계산
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # 추적된 위치에 원 그리기
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