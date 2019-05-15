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
	num_particles = 100; // TODO: set the number of particles

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	//TODO: not sure ParticleFilter::particles[i] can replace with particles[i]?

	for (int i = 0; i < num_particles; ++i) {

		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized = true;
	//ParticleFilter::num_particles = num_particles; //TODO to see if this is the right implementation 
}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	using std::normal_distribution;
	std::default_random_engine gen_predict;
	//TODO: num_particles vs. ParticleFilter::num_particles, set the number of particles

	for (int i = 0; i < num_particles; ++i) {

		double new_x;
		double new_y;
		double new_theta;

		if (fabs(yaw_rate) <1e-5)
		{
			new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			new_theta = particles[i].theta;

		}
		else
		{
			new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);

		particles[i].x = dist_x(gen_predict);
		particles[i].y = dist_y(gen_predict);
		particles[i].theta = dist_theta(gen_predict);
	}

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> & observations) {

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

	
	for (int j = 0; j < obs_num; ++j)
	{
		//std::vector<double> nearest_dist(predict_num,0.0); // initialize the vector with 0.0 and size of predict_num;
		std::vector<double> nearest_dist;
		for (int i = 0; i < predict_num; ++i)
		{
			cout << "landmark # " << i << "- predicted_x:  " << predicted[i].x << " predicted_y: " << predicted[i].y << "\n";
			double distance = sqrt(pow((predicted[i].x - observations[j].x), 2) + pow(predicted[i].y - observations[j].y, 2));
			nearest_dist.push_back(distance);
		
		}

		double minElement = *min_element(nearest_dist.begin(), nearest_dist.end());
		int minElementIndex = min_element(nearest_dist.begin(), nearest_dist.end()) - nearest_dist.begin();
		cout << "Observation_ID #: " << j << ", obs_x: " << observations[j].x << ", obs_y: " << observations[j].y << ", min_distance: " << minElement << ",min_index: "<< minElementIndex << ", landmark_x: " << predicted[minElementIndex].x << ", landmark_y: " << predicted[minElementIndex].y << "\n";
		observations[j].id = minElementIndex;
	}


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> & observations, const Map & map_landmarks) {

	for (int i = 0; i < num_particles; ++i)

	{
		vector<LandmarkObs> obs_transform;
		vector<LandmarkObs> predicted;
		LandmarkObs obs;

		for (unsigned int j = 0; j < observations.size(); ++j)
		{
			LandmarkObs trans_obs;
			obs = observations[j];
			trans_obs.x = particles[i].x + (cos(particles[i].theta) * obs.x) - (sin(particles[i].theta) * obs.y);
			trans_obs.y = particles[i].y + (sin(particles[i].theta) * obs.x) + (cos(particles[i].theta) * obs.y);
			trans_obs.id = obs.id;
			obs_transform.push_back(trans_obs);
		}
		for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k)
		{
			double sens_dist = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
			if (sens_dist < sensor_range)
			{
				LandmarkObs landmark;
				landmark.x = map_landmarks.landmark_list[k].x_f;
				landmark.y = map_landmarks.landmark_list[k].y_f;
				landmark.id = map_landmarks.landmark_list[k].id_i;
				predicted.push_back(landmark);
			}
		}

		

		particles[i].weight = 1;
		dataAssociation(predicted, obs_transform);

		
		
		for (unsigned int m = 0; m < obs_transform.size(); ++m) 
		{

			double gauss_norm;
			double exponent;
			// TODO need to find out mu_x,mu_y  (coordinate of nearest landmark)
			// TODO need to know x_obs, y_obs from observation vector. where to get and how to ???
			double mu_x = predicted[obs_transform[m].id].x;
			double mu_y = predicted[obs_transform[m].id].y;

			
			gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			exponent = (pow(obs_transform[m].x - mu_x, 2) / (2 * pow(std_landmark[0], 2))) + (pow(obs_transform[m].y - mu_y, 2) / (2 * pow(std_landmark[1], 2)));
			//cout << "predicted id: " << obs_transform[m].id << ", mu_x: " << mu_x << ", observation_x: " << obs_transform[m].x << ", mu_y: " << mu_y << ", observation_y: " << obs_transform[m].y << ", gauss_norm: " << gauss_norm << ", exponent: " << exponent << "\n";
			particles[i].weight *= gauss_norm * exp(-exponent);
		}
		
		
	}
}


void ParticleFilter::resample() {

	/**   TODO: Resample particles with replacement with probability proportional
		* to their weight.
		* NOTE : You may find std::discrete_distribution helpful here.
		* http ://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
		* https://robotics.stackexchange.com/questions/479/particle-filters-how-to-do-resampling
		* http://cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf
		* https://www.youtube.com/watch?v=aHLslaWO-AQ
		*

	*/
	
	for (int i = 0; i < particles.size(); ++i)
	{
		cout << "weight:" << particles[i].weight << endl;
	}
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // creating seed
	std::default_random_engine gen_resample(seed);
	std::uniform_real_distribution<> ranN(0, 1);

	int index;
	int N = particles.size();
	index = int(ranN(gen_resample) * N);
	double beta = 0.0;
	double mw = 0.0;
	std::vector<Particle> p3;

	for (int j = 0; j < N; j++)
	{
		mw = max(particles[j].weight, mw);

	}

	double max_weight = mw; //obtain maximum weight

	for (int i = 0; i < N; i++)
	{
		beta = beta + ranN(gen_resample) * 2 * max_weight;
		while (particles[index].weight < beta) {

			beta = beta - particles[index].weight;
			index = (index + 1) % N;
		}
		p3.push_back(particles[index]);
	}
	particles = p3;
}


void ParticleFilter::SetAssociations(Particle & particle, const vector<int> & associations, const vector<double> & sense_x, const vector<double> & sense_y)
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

