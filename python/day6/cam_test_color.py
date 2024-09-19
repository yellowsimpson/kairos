#<cam_test_color>
import cv2

cap = cv2.VideoCapture(0)
cap.set(3,100)
cap.set(4,200)
low = 50
hight = 150

while True:
    _, frame = cap.read()
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    RGB_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    canny_frame = cv2.Canny(gray_frame, low, hight) # 윤곽 강조
    blur_frame = cv2.GaussianBlur(frame, (5,5), 20) #블러 처리   # 기본적으론 gray -> blur -> canny
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    cv2.imshow('video',frame)
    cv2.imshow('gray', gray_frame)
    cv2.imshow('rgb', RGB_frame)
    cv2.imshow('canny', canny_frame)
    cv2.imshow('blur', blur_frame)
    cv2.imshow('hsv', hsv_frame)
    
    if cv2.waitKey(10) & 0xff == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
