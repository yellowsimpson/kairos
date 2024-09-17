#<img_color_filter>
import cv2

path = 'img\chonsik.jpg'

img = cv2.imread(path)
img = cv2.resize(img, (400,400))

def callback(value):
    img[:,:,0] = 0

def callback1(value):
    img[:,:,1] = 0

def callback2(value):
    img[:,:,2] = 0

cv2.namedWindow('color')
cv2.createTrackbar('b', 'color', 0, 255, callback)
cv2.createTrackbar('g', 'color', 0, 255, callback1)
cv2.createTrackbar('r', 'color', 0, 255, callback2)

while True:
    
    # img[:,:,0] = 
    
    cv2.imshow('color', img)
    
    if cv2.waitKey(10) & 0xff == ord(' '):
        break
    
cv2.destroyAllWindows()
