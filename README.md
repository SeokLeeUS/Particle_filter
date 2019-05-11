# Particle_filter
Udacity self driving car nano degree project #5 prep work


|filter variance  |State space  |belief        |efficiency |in robotics|
|:---             |:-           |:-            |:-         |:-         |
|Histogram filter |discrete     |multi modal   |exponential|approximate|
|Kalman filter    |continuous   |uni modal     |quadratic  |approximate|
|Particle filter  |continuous   |multimodal    |?          |approximate|


# A flow chart describes particle filter impmentation:

![Particlefiiter_flowchart 1](/particle_filter_figure/particle_filter_algorithm_flowchart.png)

# Steps for implementation:

|sequence|algorithm flow chart|
|:--     |:--                 |
|1       |initialization      |
|2       |prediction          |
|3       |weight updates      |
|4       |resampling          |


## 1. initialization
- Estimate position from GPS input
- This is the most practical way to initialize the particles and generate real time output

## 2. prediction
- Add control input(yaw rate& velocity) for all particles

## 3. update
- Update particle weights using map landmark positions and feature measurements

## 4. resampling
- resample M times drawing a prticle i proportional to its weight. 

## 5. return new particle set to prediciton
- new set of particles representing the Bayes filter posterior probability. 

# Project implementation tips

- sample Gaussian ditribution to determine particles
- consider Gaussian sensor noise around initial GPS poisition and heading estimates
- C++ standard library normal distribution and C++ standard library random engine function 

- C++ random library supports normal distribtion class. The class receives mean (mean()) and standard deviation (stddev()) as inputs. 

- example:

```
std::default_random_engine eng;
std::normal_distribution<double> dist(5.0,2.0);
// mean 5, standard deviation 2
array<int,10> tmp = {0, }; 
//initialize with zero and set the array size of 10
for (int i = 0; i < 10000; ++i) {
  double number = dist(eng);
  if((number>= 0.0) &&(number <10.0)) ++tmp[int(number)];
}

```
[ref: https://blog.naver.com/drvoss/220340760979]

```
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

using namespace std;

int main()
{
	default_random_engine generator;
	normal_distribution<double> distribution(5.0, 4.0);
	map<int, int> hist;
	for (int n = 0; n < 5000000; ++n) {
		++hist[std::round(distribution(generator))];
	}

	for (auto p : hist) {

		cout << std::fixed << std::setprecision(1) << std::setw(2) << p.first << ' ' << std::string(p.second / 25000, '*') << endl;
	}

	getchar();

	return 0;
}
```



