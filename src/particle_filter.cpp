/*
 * particle_filter.cpp
 *
 * Edited on: April 12, 2018

 * Author: Jie Wang & Tiffany Huang 
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// initialization based on GPS (x, y) and IMU (yaw)

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;

	// generate normal distributions for x, y, and theta 

	std::default_random_engine gen;

	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);

	// generate particles 

	for (int i = 0; i < num_particles; i++){
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1;

		particles.push_back(particle);

		weights.push_back(1);
	}

	is_initialized = true;
}

/* Prediction step: the location of each particle at next time step will be predicted based on state model
   Always remember, the particals are the simulating pose of the car

   Even for the prediction based on model, the controls is noiseless, but some Gaussian noise (std_pos) still be
   added to the transform position. The reason we are doing that is when later we are doign the resampling of the 
   weights and only a strongest weight survive so that at the end of that you will just have one particle fill up
   your entire list if the prediction is noiseless. Here we're using some jitter and from here we can sort of
   generate new particles, so we avoid the problem that our list is occupied by a single particle. 
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	// http://www.cplusplus.com/reference/random/normal_distribution/
	default_random_engine gen;

	for (int i = 0; i < num_particles; i++){
		double new_x;
		double new_y;
		double new_theta;

		// predict the new states of the car by the state space physical model 
		// considering control inputs(velocity and yaw rate)

		if (fabs(yaw_rate) < 0.0001){
			new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else {
			new_x = particles[i].x + velocity / yaw_rate  * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity / yaw_rate  * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		// adding random Gaussian noise (sensor noise) to the predicted particle states
		normal_distribution<double> N_x(new_x, std_pos[0]);
		normal_distribution<double> N_y(new_y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[1]);

		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, double sensor_range){
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark using nearest neighbor algorithm. 
	// use it as a helper during the updateWeights phase.

	for (int i = 0; i < observations.size(); i++){
		// current observation
		//LandmarkObs curr_observation = observations[i];
		LandmarkObs & curr_observation = observations[i];

		// initialize the minumum distance to 2 sensor_range 
		double minimum_dist = sensor_range * 2.0;

		// initialize the observed landmark id to be -1
		int observation_id = -1;

		for (int j = 0; j < predicted.size(); j++){
			//current predicted 
			//LandmarkObs curr_prediction = predicted[i]; ////////////////////////// errors
			LandmarkObs curr_prediction = predicted[j];

			// distance 
			double current_dist = dist(curr_observation.x, curr_observation.y, curr_prediction.x, curr_prediction.y);

			// 
			if (current_dist < minimum_dist){
				minimum_dist = current_dist;
				observation_id = curr_prediction.id;
			}
		}
		curr_observation.id = observation_id;
	}	
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.

	// loop each particle
	double weight_normalizer = 0.0;

	for (int i = 0; i < num_particles; i++){

		//retrive particle states
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		// Step 1: fliter landmarks inside the sensor range
		// create a vector for the landmarks inside the sensor range w.r.t. the predicted particles 
		vector<LandmarkObs> range_landmarks;

		// loop each landmark of the map
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++){
			Map::single_landmark_s position_landmark = map_landmarks.landmark_list[j];

			double distance = dist(p_x, p_y, position_landmark.x_f, position_landmark.y_f); 
			if (distance < sensor_range){
				range_landmarks.push_back(LandmarkObs{position_landmark.id_i, position_landmark.x_f, position_landmark.y_f});
			}
		}

		// Step 2: transform the observations w.r.t. predicted particles to w.r.t. global map
		vector<LandmarkObs> observations_map;
		for (int j = 0; j < observations.size(); j++){
			double t_x = p_x + cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y;
			double t_y = p_y + sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y;
			observations_map.push_back(LandmarkObs{observations[j].id, t_x, t_y });
		}

		// Step 3: associate the observed landmarks (from the sensor, lidar for example) 
		// to the landmarks in the map
		dataAssociation(range_landmarks, observations_map, sensor_range);

		// Step 4: calculate weight of the particle using Multivariate Gaussian distribution
		particles[i].weight = 1.0;

		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		double obs_weight = 1.0;

		for (int i = 0; i < observations_map.size(); i++){
			double obs_x = observations_map[i].x;
			double obs_y = observations_map[i].y;
			double obs_id = observations_map[i].id;

			for (int j = 0; j < range_landmarks.size(); j++){
				double range_x = range_landmarks[j].x;
				double range_y = range_landmarks[j].y;
				double range_id = range_landmarks[j].id;

				if (obs_id == range_id){
					double x_part = pow((obs_x - range_x), 2) / (2 * sig_x * sig_x);
					double y_part = pow((obs_y - range_y), 2) / (2 * sig_y * sig_y);
					double obs_weight = exp(-(x_part + y_part)) / (2 * M_PI * sig_x * sig_y);

					particles[i].weight *= obs_weight;
				}
			}
		}

		weight_normalizer += particles[i].weight;
	}
	//vector<double> weights;  // get all particles weight
	for (int i = 0; i < num_particles; i++){
		particles[i].weight /= weight_normalizer;
		//weights.push_back(particles[i].weight);
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 

	// generate discrete distribution based on weights
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	// resample particles according to weights
	vector<Particle> resample_particles;
	resample_particles.resize(num_particles);

	for (int i = 0; i < num_particles; i++){
		resample_particles[i] = particles[distribution(gen)];
	}

	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
