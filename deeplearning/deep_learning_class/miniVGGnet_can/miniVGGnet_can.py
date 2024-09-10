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
import random

# path for images 
imagePaths = list(paths.list_images("./images"))

data = []
labels = []

aap = AspectAwarePreprocessor(64, 64)

#img = cv2.imread(imagePaths[randint(1, len(imagePaths))])
#img = cv2.flip(1, img)
#cv2,imwrite(sdfsd;fl)
# get label and data 
# data -> numpy array (64, 64, 3) label -> string 
for imagePath in imagePaths:
    image = cv2.imread(imagePath)
    image = aap.preprocess(image)
    data.append(image)
    label =  imagePath.split(os.path.sep)[-2]
    print(imagePath.split(os.path.sep)[-1])
    labels.append(label)
   
# list variable -> numpy array
data = np.array(data)
labels = np.array(labels)

# nomalizing 
data = data.astype("float")/255

# train data test data split 
(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.25, random_state=42, shuffle=True)
# print(trainX.shape)

# e.g. for 10 labels 7 -> [0 0 0 0 0 0 0 1 0 0 0] 
lb = LabelBinarizer()
trainY = lb.fit_transform(trainY)
testY = lb.fit_transform(testY)

# optimizer for gradient descent method 
opt = SGD(lr=0.01, decay =0.01/40, momentum=0.9, nesterov=True)

'''
aug = ImageDataGenerator(rotation_range=30, width_shift_range=0.1,
	height_shift_range=0.1, shear_range=0.2, zoom_range=0.2,
	horizontal_flip=True, fill_mode="nearest")
'''

# model
model = Sequential()
# 1 CONV
model.add(Conv2D(32, (3,3), padding="same", input_shape=(64, 64, 3)))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(Conv2D(32, (3,3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))

# 2 CONV
model.add(Conv2D(64, (3,3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(Conv2D(64, (3,3), padding="same"))
model.add(Activation("relu"))
model.add(BatchNormalization(axis=-1))
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))

# FC
model.add(Flatten())
model.add(Dense(512))
model.add(Activation("relu"))
model.add(BatchNormalization())
model.add(Dropout(0.5))

# Class
model.add(Dense(3))
model.add(Activation("softmax"))

# Model compile 
model.compile(loss="categorical_crossentropy",optimizer=opt, metrics=["accuracy"])

'''
# with Augmentation 
H = model.fit_generator(aug.flow(trainX, trainY, batch_size=32),
	validation_data=(testX, testY), steps_per_epoch=len(trainX) // 32,
	epochs=10, verbose=1)
'''
# without augmentation
epoch_num = 100
H = model.fit(trainX, trainY, validation_data=(testX, testY), 
              batch_size=32, epochs=epoch_num, verbose=1)

model.save("vgg_cans.h5")

plt.style.use("ggplot")
plt.figure()
plt.plot(np.arange(0, epoch_num), H.history["loss"], label="train_loss")
plt.plot(np.arange(0, epoch_num), H.history["val_loss"], label="val_loss")
plt.plot(np.arange(0, epoch_num), H.history["accuracy"], label="train_acc")
plt.plot(np.arange(0, epoch_num), H.history["val_accuracy"], label="val_acc")
plt.title("Training Loss and Accuracy on CIFAR-10")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.savefig("vgg.png")
plt.show()