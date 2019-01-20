/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <random>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::default_random_engine gen;

  num_particles = 100; 


  // Create a normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particle p;

    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);   
    p.weight = 1.0;

    particles.push_back(p);

  }
  
  is_initialized = true;

  std::cout<<"initialized"<<is_initialized;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  // Create a normal (Gaussian) distribution for sensor noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++){
    if (yaw_rate > 0.00001){
      particles[i].x = velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y = velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta = particles[i].theta + yaw_rate * delta_t;
    }
    else{
      particles[i].x = velocity * delta_t * cos(particles[i].theta);
      particles[i].y = velocity * delta_t * sin(particles[i].theta);
    }

    // Add random Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);

    std::cout << "Particle" << particles[i].x << std::endl;
  }  

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (unsigned int i = 0; i < observations.size(); ++i) {
    int closest_landmark = 0;
    int min_dist = 999999;
    int curr_dist;

    // Iterate through all predicted landmarks to check which is closest
    for (unsigned int j = 0; j < predicted.size(); ++j) {
      // Calculate Euclidean distance
      curr_dist = sqrt(pow(predicted[i].x - observations[j].x, 2)
                   + pow(predicted[i].y - observations[j].y, 2));
      // Compare to min_dist and update if closest
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        closest_landmark = j;
      }
    }
    
    observations[i].id = closest_landmark;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
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

  for (int i = 0; i < num_particles; i++){

    // transform observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> mapObservations;
    for (unsigned int j = 0; j < observations.size(); j++){
      LandmarkObs observation;
      observation.id = observations[j].id;
      observation.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y); // transform to map x coordinate
      observation.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y); // transform to map y coordinate
      
      mapObservations.push_back(observation);
    }

    // only interested in landmarks that are in the sensor range for the current particle
    vector<LandmarkObs> landmarks;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
      float landmark_x =  map_landmarks.landmark_list[j].x_f;
      float landmark_y =  map_landmarks.landmark_list[j].y_f;

      if (fabs(landmark_x - particles[i].x) < sensor_range && fabs(landmark_y - particles[i].y) < sensor_range){
        landmarks.push_back(LandmarkObs {map_landmarks.landmark_list[j].id_i, landmark_x, landmark_y});
      }

    }

    // associate the predicted measurement that is closest to each observed measurement
    dataAssociation(landmarks, mapObservations);


    // calculate weights using a multi-variate Gaussian distribution

    // reset the particle weight to 1
    particles[i].weight = 1.0;

    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];

    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y); // calculate normalization term

    double mo_x, mo_y;
    for (unsigned int j = 0; j < mapObservations.size(); j++){

      unsigned int k = 0;
      do{
        if (landmarks[k].id == mapObservations[j].id){
          mo_x = landmarks[k].x;
          mo_y = landmarks[k].y;
          break;
        }
        k++;
      }
      while (k < landmarks.size());

      double obs_x = mapObservations[j].x;
      double obs_y = mapObservations[j].y;
      
      // calculate exponent
      double exponent = (pow(obs_x - mo_x, 2) / (2 * pow(sig_x, 2)))
                  + (pow(obs_y - mo_y, 2) / (2 * pow(sig_y, 2)));
        
      // calculate weight using normalization terms and exponent
      double weight = gauss_norm * exp(-exponent);

      particles[i].weight *= weight;
    }
    
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::default_random_engine gen;

  vector<Particle> resampled;

  vector<double> weights;
  for (int i = 0; i < num_particles; i++){
    weights.push_back(particles[i].weight);
  }

  std::uniform_real_distribution<double> uniformRealDist(0, num_particles - 1);
  int index = uniformRealDist(gen);

  double beta = 0.0;
  double mw = *std::min_element(std::begin(weights), std::end(weights));

  for (int i = 0; i < num_particles; i++){
    beta += uniformRealDist(gen) * 2.0 * mw;

    while (beta > weights[index]){
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
      
    resampled.push_back(particles[index]);
  }

  particles = resampled;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}