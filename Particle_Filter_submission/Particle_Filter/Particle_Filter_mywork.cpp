#define _USE_MATH_DEFINES
#include "Particle_Filter.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "helper_functions.h"
// for random number generation
#include <cstdlib>
//#include <ctime>
#include <chrono>

using namespace std;
using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/**
	* TODO: set the number of particles. initialize all particles to 
	* first position (based on estimates of x,y, theta and their uncertainties from GPS) and all weights to 1. 
	TODO: add random Gaussian noise to each particle.
	* note: consult particle_filter.h for more information about this method. 

	*/
	using std::normal_distribution;
	std::default_random_engine gen;
	num_particles = 1000; // TODO: set the number of particles

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	//TODO: not sure ParticleFilter::particles[i] can replace with particles[i]?

	for (int i = 0; i < num_particles; ++i) {

		
		ParticleFilter::particles[i].x = dist_x(gen);
		ParticleFilter::particles[i].y = dist_y(gen);
		ParticleFilter::particles[i].theta = dist_theta(gen);
		ParticleFilter::particles[i].weight = 1;
	}

	ParticleFilter::is_initialized = true;
	ParticleFilter::num_particles = num_particles; //TODO to see if this is the right implementation 
	}

	

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	using std::normal_distribution;
	std::default_random_engine gen_predict;
	//TODO: num_particles vs. ParticleFilter::num_particles, set the number of particles

	for (int i = 0; i < num_particles; ++i) {
		

		//prediction by taking into account controls input 
		ParticleFilter::particles[i].x = ParticleFilter::particles[i].x +velocity/yaw_rate*(sin(ParticleFilter::particles[i].theta+yaw_rate*delta_t)-sin(ParticleFilter::particles[i].theta));
		ParticleFilter::particles[i].y = ParticleFilter::particles[i].y +velocity/yaw_rate*(cos(ParticleFilter::particles[i].theta)-cos(ParticleFilter::particles[i].theta +yaw_rate*delta_t));
		ParticleFilter::particles[i].theta = ParticleFilter::particles[i].theta +yaw_rate*delta_t;

		normal_distribution<double> dist_x(ParticleFilter::particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(ParticleFilter::particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(ParticleFilter::particles[i].theta, std_pos[2]);

		ParticleFilter::particles[i].x = dist_x(gen_predict);
		ParticleFilter::particles[i].y = dist_y(gen_predict);
		ParticleFilter::particles[i].theta = dist_theta(gen_predict);
	}

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {

	/**
	  
	  * TODO: Find the predicted measurement that is closest to each
	  *   observed measurement and assign the observed measurement to this
	  *   particular landmark.
	  * NOTE: this method will NOT be called by the grading code. But you will
	  *   probably find it useful to implement this method and use it as a helper
	  *   during the updateWeights phase.
	  * http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/pdfs/rob2-11-dataassociation.pdf
	  * min() is coming from algorithm library:
	  * https://m.blog.naver.com/PostView.nhn?blogId=kks227&logNo=220246545025&proxyReferer=https%3A%2F%2Fwww.google.com%2F
	  */

	int obs_num = observations.size();
	int predict_num = predicted.size();
	double nearest_dist;
	double minVal = 1000; // dummy initial value to make it big to calculate minimum value. 
	for (int j = 0; j < obs_num; ++j)
	{	
		for (int i = 0; i < predict_num; ++i)
		{
			nearest_dist = sqrt(pow((predicted[i].x - observations[j].x), 2) + pow(predicted[i].y - observations[j].y, 2));
			minVal = min(nearest_dist, minVal);

			if (fabs(nearest_dist - minVal)<0.01) // to compensate the small number difference
			{
				observations[j].id = i;
			}
		}
		
	}

	
	
	
	
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs>& observations, const Map& map_landmarks) {

// TODO need to find out mu_x,mu_y  (coordinate of nearest landmark)
// TODO need to know x_obs, y_obs from observation vector. where to get and how to ???

/**
* TODO: Update the weights of each particle using a mult-variate Gaussian
*   distribution. You can read more about this distribution here:
*   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
* NOTE: The observations are given in the VEHICLE'S coordinate system.
*   Your particles are located according to the MAP'S coordinate system.
*   You will need to transform between the two systems. Keep in mind that
*   this transformation requires both rotation AND translation (but no scaling).
*   The following is a good resource for the theory:
*   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
*   and the following is a good resource for the actual equation to implement
*   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
*/
// the below is homogenous transformation 
// shouldn't I call association function?




	vector<LandmarkObs> obs_transform;
	vector<LandmarkObs> predicted;
	//ParticleFilter::dataAssociation(observations, map_landmarks);
	
	for (int i = 0; i < num_particles; ++i)
	
	{
		for (unsigned int j = 0; j < observations.size(); ++j)
		{
			LandmarkObs obs;
			obs.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
			obs.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) - (cos(particles[i].theta) * observations[j].y);
			obs.id = observations[j].id;
			obs_transform.push_back(obs);
		}

		//vector<LandmarkObs> predicted;

		for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k)
		{
			double distance = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);

			if (distance < sensor_range)
			{
				LandmarkObs landmark;
				landmark.x = map_landmarks.landmark_list[k].x_f;
				landmark.y = map_landmarks.landmark_list[k].y_f;
				landmark.id = map_landmarks.landmark_list[k].id_i;
				predicted.push_back(landmark);
			}

		}


		ParticleFilter::dataAssociation(predicted, obs_transform);

	}

	


	for (unsigned int i = 0; i < observations.size(); ++i) {

		// TODO need to find out mu_x,mu_y  (coordinate of nearest landmark)
		// TODO need to know x_obs, y_obs from observation vector. where to get and how to ???
		double mu_x = predicted[observations[i].id].x;
		double mu_y = predicted[observations[i].id].y;

		double gauss_norm;
		gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
		double exponent;
		exponent = (pow(observations[i].x - mu_x, 2) / (2 * pow(std_landmark[0], 2))) + (pow(observations[i].y - mu_y, 2) / (2 * pow(std_landmark[1], 2)));
		//double weight;
		ParticleFilter::particles[i].weight = gauss_norm * exp(-exponent);

	}


}


void ParticleFilter::resample(){

	/**   TODO: Resample particles with replacement with probability proportional
		* to their weight.
		* NOTE : You may find std::discrete_distribution helpful here.
		* http ://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
		* https://robotics.stackexchange.com/questions/479/particle-filters-how-to-do-resampling
		* http://cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf
		* https://www.youtube.com/watch?v=aHLslaWO-AQ
		* 

	*/

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // creating seed
	std::default_random_engine gen_resample(seed); 
	std::uniform_real_distribution<> ranN (0,1);
	
	int index;
	int N = particles.size();
	index= int(ranN(gen_resample) * N);
	double beta = 0.0;	
	double mw = 0.0;
	std::vector<Particle> p3;

	for (int j = 0; j < N; j++)
	{
		mw = max(particles[j].weight,mw);

	}
	
	double max_weight = mw; //obtain maximum weight
	
	for (int i = 0; i < N; i++)
	{
		beta = beta+ ranN(gen_resample)*2*max_weight;
		while (particles[index].weight < beta) {

			beta = beta - particles[index].weight;
			index = (index+1)%N;
		}
		p3.push_back(particles[index]);
	}
	particles = p3;
}


void ParticleFilter::SetAssociations(Particle& particle, const vector<int>& associations, const vector<double>& sense_x, const vector<double>& sense_y)
{

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y; 

}

string ParticleFilter::getAssociations(Particle best) {

	vector<int> v = best.associations;
	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);
	return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {

	vector<double> v;

	if (coord == "X") {

		v = best.sense_x;
	}
	else {

		v = best.sense_y;

	}

	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);
	return s;

}

