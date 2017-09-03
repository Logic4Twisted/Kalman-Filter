import random
import numpy as np
import matplotlib.pyplot as plt

import turtle
import time


class KalmanFilter:
	def __init__(self, sigma_x):
		self.P = np.array([[sigma_x*sigma_x, 1000.0*sigma_x], [1000.0*sigma_x, 1000000.0]])
		self.Q = np.ones((2,2)) * 0.1
		self.R = np.ones((1,1)) * (sigma_x*sigma_x)
		self.is_init = False


	def process(self, measurment, dt):
		if not self.is_init:
			self.x = np.array([[measurment],[0.0]])
			self.is_init = True
			return
		A = np.array([[1.0, dt],[0.0, 1.0]])
		x_p = A.dot(self.x)
		print 'x_p = ', x_p 
		A_t = np.transpose(A)
		P_p = A.dot(self.P).dot(A_t) + self.Q
		print 'P_p =', P_p 

		H = np.array([[1.0, 0.0]])
		H_t = np.transpose(H)

		S = H.dot(P_p).dot(H_t) + self.R
		print 'S = ', S
		S_i = np.linalg.inv(S)
		print 'S_i = ', S_i

		K = P_p.dot(H_t).dot(S_i)

		print 'K = ', K

		z = np.array([[measurment]])
		self.x = x_p + K.dot(z - H.dot(x_p))

		print 'x = ', self.x

		I = np.identity(2)
		self.P = (I - K.dot(H)).dot(P_p)

		print 'P = ', self.P

	def state(self):
		return self.x

	def covar(self):
		return self.P


sigma = 1.0
v = 1.0
dt = 1.0

x_actual = 10.0
v_actual = v

kf = KalmanFilter(sigma)
no_iter = 30

measured = []
actual = []
position = []
vs = []
covars = []
t = range(no_iter)


for i in t:
	x_measured = random.gauss(x_actual, sigma)
	print '-'*50
	print 'measured position :', x_measured
	x = kf.process(x_measured, dt)
	print 'filter output :', kf.state()[0]
	print 'actual :', x_actual

	measured.append(x_measured - x_actual)
	actual.append(x_actual)
	position.append(kf.state()[0] - x_actual)
	vs.append(kf.state()[1] - v_actual)
	covars.append(kf.covar()[0][0])
	alex.forward(x_actual)
	
	x_actual += v*dt

plt.plot(t, measured, 'ro', t, position, 'g', t, covars, 'b')
plt.show()
