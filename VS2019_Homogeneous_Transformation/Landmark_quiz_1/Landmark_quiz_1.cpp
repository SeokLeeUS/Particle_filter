#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <math.h>
using namespace std;
int main() {
	double x_part, y_part, x_obs, y_obs, theta;
	x_part = 4;
	y_part = 5;
	x_obs = 2;
	y_obs = 2;
	theta = -M_PI / 2;

	double x_map;
	x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);

	double y_map;
	y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

	cout << int(round(x_map)) << ',' << int(round(y_map)) << endl;

	return 0;
}