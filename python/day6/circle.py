#<circle>
import cv2

path = 'img\chonsik.jpg'

img = cv2.imread(path)

y = img.shape[0]//2
x = img.shape[1]//2

cv2.circle(img, (x, y), 50, (0,0,255), -1)

cv2.imshow('chonsik', img)

cv2.waitKey()
cv2.destroyAllWindows()