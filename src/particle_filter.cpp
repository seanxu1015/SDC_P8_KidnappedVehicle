/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  num_particles = 15;
  is_initialized = true;
  for (unsigned int i=0; i<num_particles; ++i) {
    Particle P;
    P.id = i;
    P.x = dist_x(gen);
    P.y = dist_y(gen);
    P.theta = dist_theta(gen);
    P.weight = 1.0;
    weights.push_back(1.0);
    particles.push_back(P);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  default_random_engine gen;
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  if (yaw_rate < 1e-3) {
    for (unsigned int i=0; i<num_particles; ++i) {
      Particle P = particles[i];
      P.x += velocity * cos(P.theta) * delta_t + dist_x(gen);
      P.y += velocity * sin(P.theta) * delta_t + dist_y(gen);
      P.theta += yaw_rate * delta_t + dist_theta(gen);
      particles[i] = P;
    }
  } else {
    for (unsigned int i=0; i<num_particles; ++i) {
      Particle P = particles[i];
      P.x += velocity/yaw_rate*(sin(P.theta+yaw_rate*delta_t)-sin(P.theta)) + dist_x(gen);
      P.y += velocity/yaw_rate*(cos(P.theta)-cos(P.theta+yaw_rate*delta_t)) + dist_y(gen);
      P.theta += yaw_rate * delta_t + dist_theta(gen);
      particles[i] = P;
    }
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  for (unsigned int i=0; i<observations.size(); ++i) {
    double min_distance = numeric_limits<double>::max();
    LandmarkObs obs = observations[i];
    for (unsigned int j=0; j<predicted.size(); ++j) {
      LandmarkObs pred = predicted[j];
      double distance = sqrt((obs.x-pred.x)*(obs.x-pred.x)+(obs.y-pred.y)*(obs.y-pred.y));
      if (min_distance > distance) {
        min_distance = distance;
        obs.id = pred.id;
      }
    }
    observations[i] = obs;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

  double w_sum = 0;
  // calculate some constants here
  const double sensor_range_2 = sensor_range * sensor_range;
  const double sig_x_2 = std_landmark[0] * std_landmark[0];
  const double sig_y_2 = std_landmark[1] * std_landmark[1];
  const double C_xy = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

  // iter through every particles
  for (unsigned int i=0; i<num_particles; ++i) {
    Particle P = particles[i];

    // coordinate transformation and filter those out of sensor range
    vector<LandmarkObs> obs_in_range;
    for (unsigned int j=0; j<observations.size(); ++j) {
      LandmarkObs ori_obs = observations[j];
      if (ori_obs.x*ori_obs.x+ori_obs.y*ori_obs.y<sensor_range_2) {
        LandmarkObs trans_obs;
        trans_obs.x = ori_obs.x * cos(P.theta) - ori_obs.y * sin(P.theta) + P.x;
        trans_obs.y = ori_obs.x * sin(P.theta) + ori_obs.y * cos(P.theta) + P.y;
        obs_in_range.push_back(trans_obs);
      }
    }

    // store data in landmarks in a vector
    // unordered_map<int,LandmarkObs> lm_dict;
    vector<LandmarkObs> landmarks;
    for (unsigned int j=0; j<map_landmarks.landmark_list.size(); ++j) {
      LandmarkObs lmo;
      lmo.id = map_landmarks.landmark_list[j].id_i;
      lmo.x = map_landmarks.landmark_list[j].x_f;
      lmo.y = map_landmarks.landmark_list[j].y_f;
      landmarks.push_back(lmo);
      // lm_dict[lmo.id] = lmo;
    }

    // associate predicted landmarks and observations
    dataAssociation(landmarks, obs_in_range);
    // calculate probabilities to update weights
    double w = 1.0;
    for (unsigned int j=0; j<obs_in_range.size(); ++j) {
      LandmarkObs obs = obs_in_range[j];
      double mu_x = 0;
      double mu_y = 0;
      for (unsigned int k=0; k<landmarks.size();++k) {
        if (obs.id == landmarks[k].id) {
          mu_x = landmarks[k].x;
          mu_y = landmarks[k].y;
        }
      }
      double x = obs.x;
      double y = obs.y;
      // const double mu_x = lm_dict[obs.id].x;
      // const double mu_y = lm_dict[obs.id].y;
      w *= C_xy * exp(-0.5*((x-mu_x)*(x-mu_x)/sig_x_2+(y-mu_y)*(y-mu_y)/sig_y_2));
    }
    weights[i] = w;
    w_sum += w;
  }
  // normalize weights
  for (unsigned int i=0; i<num_particles; ++i) {
    if (w_sum > 0) {
      weights[i] /= w_sum;
      particles[i].weight = weights[i];
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  discrete_distribution<int> ddist(weights.begin(), weights.end());
  vector<Particle> rs_particles;
  for (unsigned int i=0; i<num_particles; ++i) {
    int ind = ddist(gen);
    Particle P = particles[ind];
    rs_particles.push_back(P);
  }
  particles = rs_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
