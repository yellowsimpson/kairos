import cv2
import numpy as np
path1 = 'dog.jpg'
img1 = cv2.imread(path1)
img1 = cv2.resize(img1, (400, 400))
def callback1(value):
    pass
def callback2(value):
    pass

cv2.namedWindow('style', cv2.WINDOW_NORMAL)
cv2.createTrackbar('sigma_s', 'style', 10, 300, callback1)
cv2.createTrackbar('sigma_r', 'style', 0, 100, callback2)

while True:
    img1_style = cv2.stylization(img1, sigma_s = 100, sigma_r= 1)
    sigma_s = cv2.getTrackbarPos('sigma_s', 'style')
    sigma_r = cv2.getTrackbarPos('sigma_r', 'style') / 100
    img1_style = cv2.stylization(img1, sigma_r=sigma_r, sigma_s=sigma_s)
    cv2.imshow('chonsik',img1)
    cv2.imshow('chonsik', img1_style)
    if cv2.waitKey(10) & 0xff == ord(' '):
        break

cv2.destroyAllWindows()
