import cv2
import numpy as np
path = 'dog.jpg'
img = cv2.imread(path)
img = cv2.resize(img, (400,400))
def callback(value):
    pass
cv2.namedWindow('track', cv2.WINDOW_NORMAL)
cv2.createTrackbar('low_h', 'track', 0, 255, callback)
cv2.createTrackbar('low_s', 'track', 0, 255, callback)
cv2.createTrackbar('low_v', 'track', 0, 255, callback)
cv2.createTrackbar('high_h', 'track', 255, 255, callback)
cv2.createTrackbar('high_s', 'track', 255, 255, callback)
cv2.createTrackbar('high_v', 'track', 255, 255, callback)
while True:
    low_h = cv2.getTrackbarPos('low_h', 'track')
    low_s = cv2.getTrackbarPos('low_s', 'track')
    low_v = cv2.getTrackbarPos('low_v', 'track')
    high_h = cv2.getTrackbarPos('high_h', 'track')
    high_s = cv2.getTrackbarPos('high_s', 'track')
    high_v = cv2.getTrackbarPos('high_v', 'track')
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low = np.array([low_h, low_s, low_v])
    high = np.array([high_h, high_s, high_v])
    mask = cv2.inRange(hsv_img, low, high)
    result = cv2.bitwise_and(img, img, mask = mask)
    cv2.imshow('track', result)
    print(low, high)
    if cv2.waitKey(10) & 0xff == ord(' '):
        break
cv2.destroyAllWindows()
