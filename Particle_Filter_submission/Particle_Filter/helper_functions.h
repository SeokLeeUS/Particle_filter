#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "map.h"
using namespace std;


#ifndef M_PI
const double M_PI = 3.141592653589;
#endif

struct control_s {

	double velocity; //velocity [m/s]
	double yawrate; //yaw rate[rad/s]
};

struct ground_truth {

	double x; // global vehicle x position[m]
	double y; // global vehicle y position [m]
	double theta; //global vehicle yaw [rad]
};

/**
* struct representing one landmark observation measurement. 
*/


struct LandmarkObs {

	int id; // id of matching landmark in the map
	double x; // local x position of landmark observation [m]
	double y; // local y position of land mark observation [m]
};

/**
* compute the Euclidean distance between two 2d points. 
* @param(x1,y1) x and y coordinates of first point
* @param(x2,y2) x and y coordinates of second point
* @output Euclidean distance between two 2D points 
*/

inline double dist(double x1, double y1, double x2, double y2) {

	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
* computes the error between ground truth and particle filter data. 
* @param (gt_x,gt_y,gt_theta) x,y and theta of ground truth 
* @param (pf_x,pf_y,pf_theta) x,y, and theta of particle filter. 
* @output error between ground truth and particle filter data. 
* inline function  https://shaeod.tistory.com/366

*/

inline double* getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {

	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI) {
		error[2] = 2.0 * M_PI - error[2];
	}

	return error;
}

/**
* reads map data from a file
* @param filename Name of file containing map data. 
* @output True if opening and reading file was successful. 

*/


inline bool read_map_data(std::string filename, Map& map) {
	// get file of map
	std:ifstream in_file_map(filename.c_str(), std::ifstream::in);
	if (!in_file_map) {
		return false;
	}

	// declare single line of map file 
	std::string line_map;


	// run over each single line

	while (getline(in_file_map, line_map)) {

		std::istringstream iss_map(line_map);

		float landmark_x_f, landmark_y_f;
		int id_i;

		// read data from current line to values

		iss_map >> landmark_x_f;
		iss_map >> landmark_y_f;
		iss_map >> id_i;

		// declare single_landmark

		Map::single_landmark_s single_landmark_temp;

		// set values

		single_landmark_temp.id_i = id_i;
		single_landmark_temp.x_f = landmark_x_f;
		single_landmark_temp.y_f = landmark_y_f;

		// add to landmark list of map

		map.landmark_list.push_back(single_landmark_temp);

	}
	return true;
}

/**
* reads control data from a file. 
* @param filename Name of file contaiing control measurements. 
* @output True if opening and reading file was successful
*/
inline bool read_control_data(std::string filename, std::vector<control_s>& position_meas) {

	std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
	if (!in_file_pos) {
		return false;
	}

	std::string line_pos;

	while (getline(in_file_pos,line_pos)) {

		std::istringstream iss_pos(line_pos);

		// declare position values:
		double velocity, yawrate;
	
		// declare single control measurement:

		control_s meas;


		// read data from line to values:

		iss_pos >> velocity;
		iss_pos >> yawrate;
	

		// set values

		meas.velocity = velocity;
		meas.yawrate = yawrate;

		// add to list of control measurements:

		position_meas.push_back(meas);
	}
	return true;
}

/**
* reads ground truth data from a file 
* @param filename Name of file containing ground truth. 
* @output true if opening and reading file was successful. 
*/

inline bool read_ground_truth(std::string filename, std::vector<ground_truth>& gt) {

	std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);

	if (!in_file_pos) {
		return false;
	}

	std::string line_pos;

	while (getline(in_file_pos, line_pos)) {

		/** istringstream example: http://egloos.zum.com/striders/v/1082329
		* take word by word splited by space. 

		*/

		std::istringstream iss_pos(line_pos);

		//declare position values

		double x, y, azimuth;

		// declare single ground truth

		ground_truth single_gt;

		// read data from line to values

		iss_pos >> x;
		iss_pos >> y;
		iss_pos >> azimuth;


		// set values
		single_gt.x = x;
		single_gt.y = y;
		single_gt.theta = azimuth;

		gt.push_back(single_gt);
	}
	return true;

}

/**
 * Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename,
	std::vector<LandmarkObs>& observations) {
	// Get file of landmark measurements
	std::ifstream in_file_obs(filename.c_str(), std::ifstream::in);
	// Return if we can't open the file
	if (!in_file_obs) {
		return false;
	}

	// Declare single line of landmark measurement file
	std::string line_obs;

	// Run over each single line
	while (getline(in_file_obs, line_obs)) {

		std::istringstream iss_obs(line_obs);

		// Declare position values
		double local_x, local_y;

		//read data from line to values
		iss_obs >> local_x;
		iss_obs >> local_y;

		// Declare single landmark measurement
		LandmarkObs meas;

		// Set values
		meas.x = local_x;
		meas.y = local_y;

		// Add to list of control measurements
		observations.push_back(meas);
	}
	return true;
}



#endif