import cv2
import numpy as np

path1 = 'dog.jpg'
path2 = 'bono.jpg'

img1 = cv2.imread(path1)
img2 = cv2.imread(path2)

print(img2.shape)
#하나의 이미지 크기 확인
img1 = cv2.resize(img1, (img2.shape[1], img2.shape[0]))
#이미지 크기를 같게 만들어주는 작업
img3 = cv2.add(img1, img2)

img4 = img1[:, :,: ]

img1[:,:,1] = 0
#y, x, 색 인덱스



# mask = np.ones((img2.shape[0], 0, 3), dtype = 'unit8') *50
# print(mask.shape)

# print(mask.shape)
# img_b = cv2.add(img1, mask)
# img_d = cv2.subtract(img2, mask)

# print(mask)

# #img1 은 밝게 img2는 어둡게
# cv2.imshow("Dog", img1)
# cv2.imshow("Bono_b", img_b)
# cv2.imshow("Bono_b", img_b)
# cv2.imshow("Dog_Bono", img2)

cv2.imshow("A", img4)
#사진을 보여줌
cv2.waitKey()
cv2.destroyAllWindows()


