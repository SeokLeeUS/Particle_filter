from math import *
import random
import sys
print(sys.version)

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

class robot:
	def __init__(self):
		self.x = random.random()*world_size
		self.y = random.random()*world_size
		self.orientation = random.random()*2.0*pi
		self.forward_noise = 0.0
		self.turn_noise    = 0.0
		self.sense_noise   = 0.0


	def set(self,new_x,new_y,new_orientation):
		if new_x < 0 or new_x >= world_size:
			raise ValueError ('x coordinate out of bound')

		if new_y< 0 or new_y>=world_size:
			raise valueError ('Y coordinate out of bound')
		if new_orientation < 0 or new_orientation >= 2*pi:
			raise ValueError ('Orientation must be in [0..2pi]')

		self.x = float(new_x)
		self.y = float(new_y)
		self.orientation = float(new_orientation)

	def set_noise(self,new_f_noise, new_t_noise, new_s_noise):

		self.forward_noise = float(new_f_noise);
		self.turn_noise = float(new_t_noise);
		self.sense_noise = float(new_s_noise);


	def sense(self):
		Z = []
		for i in range(len(landmarks)):
			dist = sqrt((self.x - landmarks[i][0])**2 + (self.y- landmarks[i][1]) **2)
			dist += random.gauss(0.0,self.sense_noise)
			Z.append(dist)
		return Z

	def move(self,turn,forward):
		if forward < 0:
			raise ValueError ('Robot cant move backwards')

		orientation = self.orientation + float(turn) + random.gauss(0.0,self.turn_noise)
		orientation %= 2*pi

		dist = float(forward) + random.gauss(0.0, self.forward_noise)
		x = self.x + (cos(orientation)*dist)
		y = self.y + (sin(orientation)*dist)
		x %= world_size
		y %= world_size


		res = robot()
		res.set(x,y,orientation)
		res.set_noise(self.forward_noise,self.turn_noise, self.sense_noise)
		return res

	def Gaussian(self,mu,sigma,X):

		return exp(-((mu-x)**2)/(sigma**2)/2.0)/sqrt(2.0*pi*(sigma**2))

	def measurement_prob(self,measurement):

		prob = 1.0;
		for i in range(len(landmarks)):
			dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
			prob *= self.Gaussian(dist, self.sense_noise, measurement[i])

		return prob

	def __repr__(self):
		return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))	

def eval(r,p):
	sum = 0.0;
	for i in range(len(p)):
		dx  = (p[i].x-r.x+(world_size/2.0))% world_size - (world_size/2.0)
		dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
		err = sqrt(dx * dx + dy * dy)
		sum += err
	return sum / float(len(p))	


####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

myrobot = robot()
# enter code here
forward_noise = 5.0
turn_noise = 0.1
sense_noise = 5.0
myrobot.set_noise(5.0,0.1,5.0)

myrobot.set(30.0, 50.0, pi/2)
myrobot = myrobot.move(-pi/2, 15.0)
print (myrobot.sense())
myrobot = myrobot.move(-pi/2, 10.0)
print (myrobot.sense())


# Now we want to create particles,
# p[i] = robot(). In this assignment, write
# code that will assign 1000 such particles
# to a list.
#
# Your program should print out the length
# of your list (don't cheat by making an
# arbitrary list of 1000 elements!)
#
# Don't modify the code below. Please enter
# your code at the bottom.

N = 1000
p = []

#for i in range(1000):
	#P[i] = robot()
#	P.append(i) = myrobot
#P[1] = robot()
for i in range(N):
	p.append(robot())
print (len(p))

# Now we want to simulate robot
# motion with our particles.
# Each particle should turn by 0.1
# and then move by 5. 

p2= []
for i in range(N):
    p2.append(p[i].move(0.1,5.0))

p = p2
print(p)

