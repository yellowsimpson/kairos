import cv2
import numpy as np
img_path = 'bono.jpg'
img = cv2.imread(img_path)
img = cv2.resize(img,(400,400))
def callBack1(value):
    pass
def callBack2(value):
    pass
cv2.namedWindow("Style", cv2.WINDOW_NORMAL)
cv2.createTrackbar("sigma_s","Style",0,300, callBack1)
cv2.createTrackbar("sigma_r","Style",0,15, callBack2)
img_style = cv2.stylization(img, sigma_s=100, sigma_r=1)
while True :
    sigma_s = cv2.getTrackbarPos("sigma_s","Style")
    sigma_r = cv2.getTrackbarPos("sigma_r","Style")
    print(f'sig_r : {sigma_r}')
    img_style = cv2.stylization(img, sigma_s=sigma_s, sigma_r=sigma_r)
    #img_style = cv2.stylization(img, sigma_s=100, sigma_r=1)
    cv2.imshow("Image",img)
    cv2.imshow("Style",img_style)
    if cv2.waitKey(10)& 0xff == ord('q'):
        break
cv2.destroyAllWindows()
