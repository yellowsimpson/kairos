from tensorflow.keras.models import load_model
import cv2
import numpy as np
from google.colab.patches import cv2_imshow

model = load_model('./cifar10_cnn.h5')

labelNames = ["airplane", "automobile", "bird", "cat", "deer", "dog", "frog", "horse", "ship", "turck"]

img = cv2.imread('./plane2.png')
cv2.imshow(img)
cv2.waitKey(img)
cv2.destroyAllWinodws()

img_resized = cv2.resize(img, (32, 32))
cv2_
