from imutils import paths
import cv2
import os 
import numpy as np
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from tensorflow.keras.optimizers import SGD
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import BatchNormalization
from tensorflow.keras.layers import Conv2D
from tensorflow.keras.layers import MaxPooling2D
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Flatten
from tensorflow.keras.layers import Dropout
from tensorflow.keras.layers import Dense
import matplotlib.pyplot as plt
from aspectawarepreprocessor import AspectAwarePreprocessor
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications.resnet50 import preprocess_input
from tensorflow.keras.applications.resnet50 import ResNet50
from tensorflow.keras import Input
from tensorflow.keras.layers import Dropout, BatchNormalization
import tensorflow as tf


imagePaths = list(paths.list_images("./animals"))
#print(imagePaths)

data = []
labels = [] 

aap = AspectAwarePreprocessor(32, 32)

for (i, imagePath) in enumerate(imagePaths):
    image = cv2.imread(imagePath) 
    #image = cv2.resize(image, (64, 64))
    image = aap.preprocess(image)
    data.append(image)
    label =  imagePath.split(os.path.sep)[-2]
    if label == 'cats':
        label = 0
    elif label == 'dogs':
        label = 1
    elif label == 'panda':
        label = 2
    else:
        print('uga')
    labels.append(label)

#print(labels[0])
#print(data[0]) 

data = np.array(data)
labels = np.array(labels)

data = data.astype("float")/255

(train_images, test_images, train_labels, test_labels) = train_test_split(data, labels, test_size=0.25, random_state=42, shuffle=True)

base_model = ResNet50(include_top=False, pooling = 'avg' , input_shape = (32,32 ,3), weights = 'imagenet')
base_model.trainable = False
print(base_model.summary())

print(train_labels[0])
print(test_labels[0])
lb = LabelBinarizer()
input_y = lb.fit_transform(train_labels)
test_y = lb.fit_transform(test_labels)
print(input_y.shape, test_y.shape)
print(input_y[0])
print(test_y[0])


inputs = Input(shape=(32,32,3))
#x = tf.keras.layers.preprocessing.Resizing(32, 32)(inputs)
x = tf.keras.applications.resnet50.preprocess_input(inputs)
x = base_model(x, training = False)
x = Flatten()(x)								# Fully Connected에 온전하게 학습을 위해 펼쳐준다	
outputs = Dense(3, activation = 'softmax')(x)	# Softmax 함수로 10개 분류하는 분류기 
model_res = tf.keras.Model(inputs, outputs)	# model_res 란 이름의 인풋과 아웃풋이 정해진 모델 생성
print(model_res.summary())

"""pre-trained 모델인 ResNet과 사용자가 정의한 NN을 결합한 전이학습 모델을 구성한다. """

model_res.compile(optimizer = tf.keras.optimizers.Adam(learning_rate= 0.001),
                  loss = 'categorical_crossentropy',
                  metrics=['accuracy'])

"""전이학습 모델을 트레이닝 한다. """

model_res.fit(train_images, input_y, epochs = 50, validation_data=(test_images, test_y), batch_size= 256)
