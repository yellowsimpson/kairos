#특정색 검출 툴 제작

import cv2
import numpy as np

path = '강아지.jpg'

img = cv2.imread(path)

# cv2.imshow('img',img)

# roi = img[50:400, 20:100, :]

def on_trackbar_change(value):
    b = cv2.getTrackbarPos('B', '강아지.jpg')
    g = cv2.getTrackbarPos('G', '강아지.jpg')
    r = cv2.getTrackbarPos('R', '강아지.jpg')

    # modified_roi = roi.copy()
    # modified_roi[:, :, 0] = b
    # modified_roi[:, :, 1] = g
    # modified_roi[:, :, 2] = r

    # cv2.imshow("강아지.jpg", modified_roi)

cv2.namedWindow("강아지.jpg")

cv2.createTrackbar('B', '강아지.jpg', 0, 255, on_trackbar_change)
cv2.createTrackbar('G', '강아지.jpg', 0, 255, on_trackbar_change)
cv2.createTrackbar('R', '강아지.jpg', 0, 255, on_trackbar_change)

cv2.setTrackbarPos('B', '강아지.jpg', 0)
cv2.setTrackbarPos('G', '강아지.jpg', 0)
cv2.setTrackbarPos('R', '강아지.jpg', 0)

# on_trackbar_change(roi)

cv2.waitKey(0)
cv2.destroyAllWindows()

