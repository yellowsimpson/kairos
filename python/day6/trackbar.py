#<trackbar>
import cv2
import numpy as np

path1 = 'img\chonsik.jpg'
path2 = 'img\chonsik2.jpg'

img1 = cv2.imread(path1)
img2 = cv2.imread(path2)

img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
img1_blur = cv2.GaussianBlur(img1, (5,5), 20)

def callback(value):
    pass

cv2.namedWindow('slider')
cv2.createTrackbar('high_t','slider', 125, 255, callback)

while True:
    
    high_t = cv2.getTrackbarPos('high_t', 'slider')
    img1_canny = cv2.Canny(img1_blur, 10, high_t)
    
    cv2.imshow('chonsik', img1_canny)
    if cv2.waitKey(10) & 0xff == ord(' '):
        break
    
cv2.destroyAllWindows()