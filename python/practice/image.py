import cv2
import numpy as np

path = 'bono.jpg'

image = cv2.imread(path)
print(image.shape)
print(image.shape[0])

cv2.circle(image, (image.shape[1]//2, image.shape[0]//2), 5, (0, 0, 255), 2, -1)
#image.shape[1]//2, image.shape[0]//2, -> (x, y좌표 )


cv2.imshow("Ham", image)

cv2.waitKey()
cv2.destroyAllWindows()


