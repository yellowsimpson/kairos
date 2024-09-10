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


imagePaths = list(paths.list_images("./images"))
print(imagePaths)

classNames = [pt.split(os.path.sep)[-2] for pt in imagePaths]
print(classNames)

classNames = [str(x) for x in np.unique(classNames)]
print(classNames)

aap = AspectAwarePreprocessor(224, 224)

data = []
labels = []

for (i, imagePath) in enumerate(imagePaths):
    image = cv2.imread(imagePath)
    #cv2.imshow('1', image)
    label = imagePath.split(os.path.sep)[-2]

    #image = img_to_array(image)
    image = aap.preprocess(image)
    #cv2.imshow('2', image)
    #cv2.waitKey(0)
    data.append(image)
    labels.append(label)

data = np.array(data)
labels = np.array(labels)

#print(data.shape)
#cv2.imshow('3', data[0])
#cv2.waitKey(0)

data = data.astype("float")/255.0

(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.25, random_state=42)

#print(trainY[0])
trainY = LabelBinarizer().fit_transform(trainY)
testY = LabelBinarizer().fit_transform(testY)
#print(trainY[0])

baseModel = VGG16(weights="imagenet", include_top=False,
	input_tensor=Input(shape=(224, 224, 3)))

headModel = baseModel.output
headModel = Flatten(name="flatten")(headModel)
headModel = Dense(255, activation="relu")(headModel)
headModel = Dropout(0.5)(headModel)

# add a softmax layer
headModel = Dense(3, activation="softmax")(headModel)

model =  Model(inputs=baseModel.input, outputs=headModel)

print(model.summary())

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

model.save('uga.h5')
