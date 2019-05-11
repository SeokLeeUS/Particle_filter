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

## 2. prediction
- Add control input(yaw rate& velocity) for all particles

## 3. update
- Update particle weights using map landmark positions and feature measurements

## 4. resampling
- resample M times drawing a prticle i proportional to its weight. 

## 5. return new particle set to prediciton
- new set of particles representing the Bayes filter posterior probability. 





