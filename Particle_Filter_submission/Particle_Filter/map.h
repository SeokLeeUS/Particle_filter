#ifndef MAP_H_
#define MAP_H_
#include <vector>
using namespace std;
class Map {
public:
	struct single_landmark_s {

		int id_i;
		float x_f;
		float y_f;


	};

	vector<single_landmark_s> landmark_list; // list of landmarks in the map
};

#endif