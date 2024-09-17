import cv2
import numpy as np

def callBack(value):
    pass

img_path = 'dog.jpg'

img = cv2.imread(img_path)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_blur = cv2.GaussianBlur(img_gray, (5,5), 1)

cv2.namedWindow("Slider")
cv2.createTrackbar("high_t", "Slider", 10, 255, callBack)

while True:

    high_t = cv2.getTrackbarPos("high_t", "Slider")
    img_canny = cv2.Canny(img_blur, 10, high_t )

    cv2.imshow("Cat", img_canny)

    if cv2.waitKey(10) & 0xff==ord('q'):
        break
cv2.destroyAllWindows()
