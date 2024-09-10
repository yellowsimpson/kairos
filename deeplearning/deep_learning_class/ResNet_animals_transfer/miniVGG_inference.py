import cv2
import os 
from tensorflow.keras.models import load_model
import numpy as np

cap = cv2.VideoCapture(0)

class_labes = ["keyboard", "mouse", "smatphone"]
model = load_model("uga.h5")
#img = cv2.imread("mouse_53.png")

while True:
    ret, img = cap.read()
    if ret:
        img_scaled = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
        img_path = 'mouse_53.png'
        label = img_path[0:-7]
        data = img_scaled  # [[[255, 123, 45 ] [   ]...]]]

        data = data.astype("float")/255.0  # [[[0.896, ] [   ]...]]]
        
        X= np.asarray([data]) # [[[[0.896, ] [   ]...]]]]

        s = model(X, training=False)  # [ 0.5 0.001 0.023]
        index = np.argmax(s)
        if index == 0:
            print("keyboard")
            strr = 'keyboard'
        elif index == 1:
            print("mouse")
            strr = 'mouse'
        elif index == 2:
            print("smartphone")
            strr = 'smartphone'
        print(index)
        cv2.putText(img, strr, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        cv2.imshow('win', img)
        if cv2.waitKey(1)&0xff == ord('q'):
            break

cv2.destroyAllWindows()