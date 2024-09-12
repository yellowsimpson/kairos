from imutils import paths
import os 
import numpy as np
import cv2
from aspectawarepreprocessor import AspectAwarePreprocessor
from tensorflow.keras.preprocessing.image import img_to_array
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelBinarizer
from tensorflow.keras.applications import VGG16
from tensorflow.keras.layers import Input
from tensorflow.keras.layers import Dropout
from tensorflow.keras.layers import Flatten
from tensorflow.keras.layers import Dense
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import RMSprop
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.optimizers import SGD
from sklearn.metrics import classification_report

imagePaths = list(pa5ths.list_images("./VGGnet_transfer_learning/images"))
#image 경로를 알려줌
print(imagePaths)
#출력 ['./images\\cats\\cats_00966.jpg', './images\\cats\\cats_00967.jpg', './images\\dogs\\dogs_00978.jpg', './images\\dogs\\dogs_00979.jpg', './images\\panda\\panda_00937.jpg', './images\\panda\\panda_00938.jpg']

classNames = [pt.split(os.path.sep)[-2] for pt in imagePaths]
#image os->pt로 나눠
# print(classNames)
#출력 ['cats', 'cats', 'dogs', 'dogs', 'panda', 'panda']

classNames = [str(x) for x in np.unique(classNames)]
# print(classNames)
#출력 ['cats', 'dogs', 'panda']

aap = AspectAwarePreprocessor(224, 224)

#입력 이미지들의 숫자 array (넘파이 형태) 리스트
data = []
#고양이, 개, 판다
labels = []

#이 for문은 입력 이미지를 실제로 data에 넣어주고, 
#결과값도 lables에 넣어주는 일을 한다.
for (i, imagePath) in enumerate(imagePaths):
    # print(imagePath)
    #enumberate -> 
    image = cv2.imread(imagePath)

    
    # print(image)
    # cv2.imshow('win', image)
    # cv2.waitKey(1000)
    #이미지를 계속 보고 싶으면  cv2.waitKey이 명령어 적어줘야되
    #숫자를 0으로 하면 무한대로 기다리고 1000으로 하면 1초마다 사진이 다음장으로 넘어가
    label = imagePath.split(os.path.sep)[-2]
    #-2 파일 이름이랑 label은 사라지고 데이터 형식만 나옴

    image = aap.preprocess(image)
    #이미지를 하나의 사이즈로 통일하는 명령어
	#

    print(label)
    data.append(image)
    labels.append(label)

data = np.array(data)
labels = np.array(labels)
print(data.shape)
print(labels.shape)

#노말라이징 0 ~ 255 -> 0 ~ 1
data = data.astype("float")/255.0
print(data[0][0][0])

(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.25, random_state=42)
#test_size=0.25 :25%는 test,75%는 train으로 사용하겠다는 의미
#train x : 이미지
#train y : 숫자

print(trainX.shape)
print(trainY.shape)


#라벨바이나리징 만약 라벨이 0, 1, 2, 3개면
# 0 -> [1, 0, 0]
# 1 -> [0, 1, 0]
# 2 -> [0, 0, 1]


#라벨바이너리징은 결과값
print("시작전", trainY)
trainY = LabelBinarizer().fit_transform(trainY)
testY = LabelBinarizer().fit_transform(testY)
print("시작후",trainY)

#### 여기서 부터 전의 학습 시작 ###
# VGG16 CNN 신경망을 다운받아 사용하는데 가중치값은 예전에 imageNET으로 이미 훈련된 값을 사용
#CNN 레이어 뒤에 DENSE레이어에는 붙이지 않는다.
#input -> 입력층 모양이 224 x 224 x 3

baseModel = VGG16(weights="imagenet", include_top=False,
	input_tensor=Input(shape=(224, 224, 3)))
#weights="imagenet" : 배나온 전직 골프선수가 섰던 기술을 그대로 가져다 쓰겠다.
#include_top=False : dense layer 안쓰겠다.

#나머지 일반 신경망 레이어 붙이기
headModel = baseModel.output
headModel = Flatten(name="flatten")(headModel)
headModel = Dense(255, activation="relu")(headModel)
headModel = Dropout(0.5)(headModel)
headModel = Dense(3, activation="softmax")(headModel)

model =  Model(inputs=baseModel.input, outputs=headModel)

#만들어진 신경망 요약하기
print(model.summary())

#각 신경망을 훈련할 것인지 결정하는 변수인 trainable을 false로
#즉 baseMode CNN 부분은 트레이닝하지 않도록 조치
# 배나온 골프 아저씨한테 train을 더시켜?? no -> 이미 골프 잘해!!
for layer in baseModel.layers:
	layer.trainable = False

#Optimizer
opt = RMSprop(learning_rate=0.001)
#Model Compile
model.compile(loss="categorical_crossentropy", optimizer=opt,
	metrics=["accuracy"])

#데이터 가상화(데이터 증가(뻥튀기)시키는거)
aug = ImageDataGenerator(rotation_range=30, width_shift_range=0.1,
	height_shift_range=0.1, shear_range=0.2, zoom_range=0.2,
	horizontal_flip=True, fill_mode="nearest")

#실제적인 트레이닝 코드
model.fit(aug.flow(trainX, trainY, batch_size=32),
	validation_data=(testX, testY), epochs=1,
	steps_per_epoch=len(trainX) // 32, verbose=1)
#batch size : 한번에 학습시킬 데이터의 양
# verbose : training값 출력해라

#예측
predictions = model.predict(testX, batch_size=32)
print(classification_report(testY.argmax(axis=1),
	predictions.argmax(axis=1), target_names=classNames))


for layer in baseModel.layers[15:]:
	layer.trainable = True
	
opt = SGD(learning_rate=0.001)
model.compile(loss="categorical_crossentropy", optimizer=opt,
	metrics=["accuracy"])

model.fit(aug.flow(trainX, trainY, batch_size=32),
	validation_data=(testX, testY), epochs=1,
	steps_per_epoch=len(trainX) // 32, verbose=1)

predictions = model.predict(testX, batch_size=32)
print(classification_report(testY.argmax(axis=1),
	predictions.argmax(axis=1), target_names=classNames))

model.save('block.h5')
