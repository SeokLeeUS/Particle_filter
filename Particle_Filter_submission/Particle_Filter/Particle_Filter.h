#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

class ParticleFilter {
public:

	//constructor
	//@param num_particles Number of particles
	ParticleFilter() :  num_particles(0), is_initialized(false){}

	//destructor
	~ParticleFilter() {}


/*
curly braces is used for list initialization 
https://stackoverflow.com/questions/18222926/why-is-list-initialization-using-curly-braces-better-than-the-alternatives
https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/

#include<iostream>
using namespace std;

class Point {
private:
	int x;
	int y;
public:
	Point(int i = 0, int j = 0):x(i), y(j) {}
	//  The above use of Initializer list is optional as the
	//	constructor can also be written as:
	//	Point(int i = 0, int j = 0) {
	//		x = i;
	//		y = j;
	//	}
	//

int getX() const { return x; }
int getY() const { return y; }
};

int main() {
	Point t1(10, 15);
	cout << "x = " << t1.getX() << ", ";
	cout << "y = " << t1.getY();
	return 0;
}

*/

/**
* init initializes particle filter by initializing particles to Gaussian
* distribution around first position and all the weights to 1. 
* @param x initial x position [m] (simulated estimated from GPS)
* @param y initial y position [m]
* @param theta initial orientation [rad]
* @param std[] Array of dimension 3 [standard deviation of x[m], standard deviation of y[m], standard deviation of yaw [rad]]
*/


	void init(double x, double y, double theta, double std[]);

/**
* prediction predicts the state for the next time step
* using the process model. 
* @param delta_t Time between time step t and t+1 in measurements. [s]
* @param std_pos[] Array of dimension 3 [standard deviation of x[m],
* standard deviation of y[m], standard deviatin of yaw [rad]]
* @ param velocity Velocity of car from t to t+1 [m/s]
* @ param yaw_rate Yaw rate of car from t to t+1[rad/s]
*/

	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);

/**
* dataAssociation Finds which observations correspond to which landmarks
* (likely by using a nearest-neighbors data association). 
* @param predicted Vector of predicted landmark observations
* @param observation Vector of landmark observation 
*/

	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);

/**
* updateWeights Updates the weights for each particle based on the likelihood
* of the observed measurements. 
* @param sensor_range Range[m] of sensor
* @param std_landmark[] Array of dimension 2
* [Landmark measurement uncertainty [x[m],y[m]]]
* @param observations Vector of landmark observations
* @param map Map class containing map landmarks
*/


	void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs>& observations, const Map& map_landmarks);

/**
* resample Resamples from the updated set of particles to form 
* the new set of particles. 
*/

	void resample();


/**
* Set a particle list of associations, along with the associations' 
* calculated world x, y coordinates
* This can be a very useful debugging tool to make sure transformations
* are correct and association correctly connected
*/

	void SetAssociations(Particle& particle, const std::vector<int>& associations, const std::vector<double>& sense_x, const std::vector<double>& sense_y);

/**
* initialized returns whether particle filter is initialized yet or not. 
*/

	const bool initialized() const{
		return is_initialized;
}

/**
* used for obtaining debugging information related to particles. 
*/


	std::string getAssociations(Particle best);
	std::string getSenseCoord(Particle best, std::string coord);

// Set of current particles
	std::vector<Particle> particles;


private:

	int num_particles;
	bool is_initialized;
	std::vector<double> weights;
};

#endif

