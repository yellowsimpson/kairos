import numpy as np

class NeuralNetwork:
    def __init__(self, layers, alpha=0.1):
        self.W = [] 
        self.layers = layers  # list [ 2, 2, 2, 1]
        self.alpha = alpha

        # 계수 생성 loop
        for i in np.arange(0, len(layers) - 2):
            # len(layer) -2 -> out -마지막 hidden layer 제외
            w = np.random.randn(layers[i]+1 ,layers[i+1]+1)
            self.W.append(w/np.sqrt(layers[i]))
        # 마지막 hidden - out 사이의 계수 
        w = np.random.randn(layers[-2]+1, layers[-1])
        self.W.append(w/np.sqrt(layers[-2]))

    def sigmoid(self, x):
        return 1.0 / (1 + np.exp(-x))

    def sigmoid_deriv(self, x):
        return x * (1 - x)
    # X = np.array([[ 0,0], [ 0,1], [1,0], [0,0]])
    # y = np.array([[0], [1], [1], [0]])
    def fit(self, X, y, epochs=1000):
        # 아래 코드는 4x2 행렬에 바이어스를 넣어서 4x3 행렬만들기
        X = np.c_[X, np.ones((X.shape[0]))] # X.shape[0] = 4
        #print(X)
        for epoch in np.arange(0, epochs):
            for (x, target) in zip(X, y): 
                # x = [0 0 1] target = [0] 
                #print(x, target)
                self.fit_partial(x, target)

    def fit_partial(self, x, y):
        # x = [ 0 0 1] y = [0]
        A = [np.atleast_2d(x)]
        # feed 
        for layers in np.arange(0, len(self.W)):
            net = A[layers].dot(self.W[layers])
            out = self.sigmoid(net)
            A.append(out)
        # A= [array([[0., 0., 1.]]), array([[0.54067744, 0.51712677, 0.26921834]]), array([[0.50808391, 0.57745501, 0.56186676]]), array([[0.4979378]])]
        error = A[-1] - y 
        D = [error * self.sigmoid_deriv(A[-1])]
        for layers in np.arange(len(A) -2, 0, -1):
            delta = D[-1].dot(self.W[layers].T)
            delta = delta*self.sigmoid_deriv(A[layers])
            D.append(delta)
        #print(D)
        D = D[::-1]
        for layer in np.arange(0, len(self.W)):
            self.W[layer] += -self.alpha * A[layer].T.dot(D[layer])

    def predict(self, X):  # X = [0 1]
        p = np.atleast_2d(X)
        p = np.c_[p, np.ones((p.shape[0]))]
        for layer in np.arange(0, len(self.W)):
            p = self.sigmoid(np.dot(p, self.W[layer]))
        return p


nn = NeuralNetwork([2, 2, 1], 0.5)
X = np.array([[ 0,0], [ 0,1], [1,0], [1,1]])
y = np.array([[0], [1], [1], [0]])
nn.fit(X, y, 1000)

for (x, target) in zip(X, y):
    pred = nn.predict(x)[0][0]
    step = 1 if pred > 0.5 else 0
    print(target[0], pred, step)
