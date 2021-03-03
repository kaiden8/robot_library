import csv
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from KalmanFilter import KalmanFilter

class UltrasonicSensor():
    def __init__(self):
        self.data = None


    def importData(self):
        self.data = np.genfromtxt('test_data/ultrasonic.csv', delimiter=',')
        self.yMeasured = self.data[:,0]
        self.yTrue = self.data[:,1]
        self.numSamples = np.arange(0,len(self.yMeasured))
        self.error = np.subtract(self.yTrue, self.yMeasured)
        self.model = LinearRegression().fit(self.yMeasured.reshape((-1,1)), self.error)

        r_sq = self.model.score(self.yMeasured.reshape((-1,1)), self.error)
        print('coefficient of determination:', r_sq)
        print('intercept:', self.model.intercept_)
        print('slope:', self.model.coef_)

        # y_pred = (self.model.coef_ * 80) + self.model.intercept_
        # print('predicted response:', y_pred, sep='\n')



    def graphData(self):

        plt.subplot(1, 2, 1)

        plt.plot(self.numSamples,self.yMeasured)
        plt.plot(self.numSamples,self.yTrue)

        plt.subplot(1, 2, 2)

        plt.plot(self.yTrue,self.error)
        plt.show()


    def graphFilteredData(self):
        print("graphFilteredData")

    def filterData(self):
        print("filterData")

        dataFilter = KalmanFilter()
        # calculate(X, P, A, Q, B, U, Y, H, R):

        P = np.array([0])
        A = np.array([1])
        Q = np.array([0.00001]) # random guess
        B = np.array([0])
        U = np.array([0])
        H = np.array([1])
        R = np.array([0.001668050349391]) # obtained from covariance function in excel using taken data
        for i in range(100):
            X = np.array([self.yTrue[i]])
            Y = np.array([self.yMeasured[i]])
            dataFilter.calculate(X, P, A, Q, B, U, Y, H, R)
