import numpy as np

# Jannik & Shyngyskhan
class Net():

    def __init__(self, weights):
        self.weights = weights
        self.memory = [0,0,0,0]

    def forwardPass(self, input, theta, activation):
        #input = np.insert(input, 0, 1, axis=1) # add bias
        z = np.dot(input, theta.T)
        a = activation(z)
        return a

    def feedForward(self, X):
        self.a1 = np.concatenate((X, self.memory)) # add memory
        self.a2 = self.forwardPass(self.a1, self.weights[0], tanh)
        # Idea: make the memory a moving average to smooth things out
        self.memory = (self.a2*2 + self.memory)/3 # store hidden layer activations
        self.a3 = self.forwardPass(self.a2, self.weights[1], linear)
        return self.a3

def initalize_theta(rows, cols):
    theta = []
    for _ in range(rows):
        temp_theta = []
        for _ in range(cols):
            value = np.random.uniform(-1, 1)
            temp_theta.append(value)
        theta.append(temp_theta)
    return np.asarray(theta)

def relu(x):
    return np.max((x,np.zeros(x.shape)),axis=0)

def linear(x):
    return x

def tanh(x):
    return np.tanh(x)