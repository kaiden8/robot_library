import numpy as np
from numpy import dot, sum, tile, linalg 
from numpy.linalg import inv
import matplotlib.pyplot as plt

class KalmanFilter():
	def __init__(self):
		kalmanGain = []
		estimatedValue = []
		error = []

	def calculate(self, X, P, A, Q, B, U, Y, H, R):
		# X: The mean state estimate of the previous step (1k−). 
		# P: The state covariance of previous step (1k−). 
		# A: The transition n    n×matrix. 
		# Q: The process noise covariance matrix. 
		# B: The input effect matrix. 
		# U: The control input.


		#  predict next state
		X = dot(A, X) + dot(B, U)
		P = dot(A, dot(P, A.T)) + Q
		print("START")
		print("X: ", X)
		print("P: ", P)

		# update
		IM = dot(H, X)     
		IS = R + dot(H, dot(P, H.T))     
		K = dot(P, dot(H.T, 1/IS)) #inv(IS)))     
		X = X + dot(K, (Y-IM))     
		P = P - dot(K, dot(IS, K.T)) 
		print("K: ", K)
		print("X: ", X)
		print("P: ", P)
		print("FINISH ")
		# print(X) 
		# print(K)   
		# LH = self.gauss_pdf(Y, IM, IS)
		# return (X)

	def gauss_pdf(self, X, M, S):     
		if M.shape()[1] == 1:         
			DX = X - tile(M, X.shape()[1])           
			E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)         
			E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))         
			P = exp(-E)     
		elif X.shape()[1] == 1:         
			DX = tile(X, M.shape()[1])- M           
			E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)         
			E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))         
			P = exp(-E)     
		else:         
			DX = X-M           
			E = 0.5 * dot(DX.T, dot(inv(S), DX))         
			E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))         
			P = exp(-E)     
		return (P[0],E[0])