#<color_live>
import cv2
import numpy as np

path1 = 'img\chonsik.jpg'

img1 = cv2.imread(path1)

roi = img1[100:600, 300:770, :]

def on_trackbar_change(val):
    b = cv2.getTrackbarPos('B', 'Cut image')
    g = cv2.getTrackbarPos('G', 'Cut image')
    r = cv2.getTrackbarPos('R', 'Cut image')

    modified_roi = roi.copy()
    modified_roi[:, :, 0] = b
    modified_roi[:, :, 1] = g
    modified_roi[:, :, 2] = r

    cv2.imshow("Cut image", modified_roi)

cv2.namedWindow("Cut image")

cv2.createTrackbar('B', 'Cut image', 0, 255, on_trackbar_change)
cv2.createTrackbar('G', 'Cut image', 0, 255, on_trackbar_change)
cv2.createTrackbar('R', 'Cut image', 0, 255, on_trackbar_change)

cv2.setTrackbarPos('B', 'Cut image', 0)
cv2.setTrackbarPos('G', 'Cut image', 0)
cv2.setTrackbarPos('R', 'Cut image', 0)

on_trackbar_change(roi)

cv2.waitKey()
cv2.destroyAllWindows()
