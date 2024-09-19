#<img_add>
import cv2
import numpy as np

path1 = 'img\chonsik.jpg'
path2 = 'img\chonsik2.jpg'

img1 = cv2.imread(path1)
img2 = cv2.imread(path2)

img2 = cv2.resize(img2, (img1.shape[1],img1.shape[0]))

img3 = cv2.add(img1, img2)

img4 = img1[:img1.shape[0]//2, :, :] 

img1[:img1.shape[0]//2, :, :] = [255,0,0]# 범위 내 img 위에 색 도포

img1[:,:,1] = 0 # 범위 내 img 위에 색 필터

mask = np.ones((img2.shape[1], img2.shape[0], 3), dtype='uint8') * 50 # dtype = 'uint8' : data 값을 1 -> 255까지 내보냄 np.ones 때문에 1로 묶이기 때문
print(mask.shape)
print(img1.shape)
print(img2.shape)
print(img4.shape)

img_b = cv2.add(img1, mask)

cv2.imshow('chonsik1', img1)
# cv2.imshow('chonsik2', img2)
# cv2.imshow('double chonsik', img3)
# cv2.imshow('bright', img_b)
# cv2.imshow('Roi', img4)

cv2.waitKey()
cv2.destroyAllWindows()
