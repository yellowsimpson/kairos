from imutils import paths
from aspectawarepreprocessor import AspectAwarePreprocessor
import cv2
import os
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelBinarizer
from tensorflow.keras.optimizers import SGD
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import BatchNormalization
from tensorflow.keras.layers import Conv2D
from tensorflow.keras.layers import MaxPooling2D
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Flatten
from tensorflow.keras.layers import Dropout
from tensorflow.keras.layers import Dense
import matplotlib.pyplot as plt

# 이미지 경로 설정
imagePaths = list(paths.list_images("./images"))

data = []
labels = []

aap = AspectAwarePreprocessor(64, 64)

# 데이터와 레이블 가져오기
for imagePath in imagePaths:
    image = cv2.imread(imagePath)
    image = aap.preprocess(image)
    data.append(image)
    label = imagePath.split(os.path.sep)[-2]
    print(imagePath.split(os.path.sep)[-1])
    labels.append(label)

# 리스트 -> numpy 배열
data = np.array(data)
labels = np.array(labels)

# 정규화
data = data.astype("float") / 255.0

# 데이터 분할
(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.25, random_state=42, shuffle=True)

# 레이블 이진화
lb = LabelBinarizer()
trainY = lb.fit_transform(trainY)
testY = lb.fit_transform(testY)

# 옵티마이저 정의
opt = SGD(learning_rate=0.01, momentum=0.9, nesterov=True)

# 이미지 증강 설정
aug = ImageDataGenerator(
    rotation_range=30,
    width_shift_range=0.1,
    height_shift_range=0.1,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    fill_mode="nearest"
)

# 모델 정의
model = Sequential()
model.add(Conv2D(32, (3, 3), padding="same", input_shape=(64, 64, 3)))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(Conv2D(32, (3, 3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Conv2D(64, (3, 3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(Conv2D(64, (3, 3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Flatten())
model.add(Dense(512))
model.add(Activation("relu"))
model.add(BatchNormalization())
model.add(Dropout(0.5))

model.add(Dense(3))
model.add(Activation("softmax"))

# 모델 컴파일
model.compile(loss="categorical_crossentropy", optimizer=opt, metrics=["accuracy"])

# 모델 훈련
epoch_num = 100
H = model.fit(
    aug.flow(trainX, trainY, batch_size=32),
    validation_data=(testX, testY),
    steps_per_epoch=len(trainX) // 32,
    epochs=epoch_num,
    verbose=1
)

# 모델 저장
model.save("block.h5")

# 학습 결과 시각화
plt.style.use("ggplot")
plt.figure()
plt.plot(np.arange(0, epoch_num), H.history["loss"], label="train_loss")
plt.plot(np.arange(0, epoch_num), H.history["val_loss"], label="val_loss")
plt.plot(np.arange(0, epoch_num), H.history["accuracy"], label="train_acc")
plt.plot(np.arange(0, epoch_num), H.history["val_accuracy"], label="val_acc")
plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.savefig("vgg.png")
plt.show()
