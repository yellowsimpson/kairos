import numpy as np
import time 

class Perceptron:
    def __init__(self, N, alpha):
        self.W = np.random.randn(N+1)/np.sqrt(N)
        self.alpha = alpha

    def step(self, x):
        if x > 0:
            return 1
        else:
            return 0

    def fit(self, X, y, epochs = 10):
        X = np.c_[X, np.ones(X.shape[0])]  # 1 bias add to X input array
        for epoch in range(epochs):
            for (x, target) in zip(X, y):
                p = self.step(np.dot(x, self.W))
                if p != target:
                    error = p - target  # wi(t +1) = wi(t) +α(dj −yj)xj,i
                    self.W += -self.alpha*error*x

    def predict(self, X): # X = [0 1] e.g.
        X = np.atleast_2d(X)
        X = np.c_[X, np.ones(1)]
        p = self.step(np.dot(X, self.W))
        print(p)
        print('------------------')


per = Perceptron(2, 0.5)
print(per.W)
# AND logic input output 
X =  np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
Y = np.array([[0], [1], [1], [0]])
per.fit(X, Y) # self.W - perceptron 
# predict
x = np.array([0,0])
per.predict(x)
x = np.array([1,0])
per.predict(x)
x = np.array([0,1])
per.predict(x)
x = np.array([1,1])
per.predict(x)
   