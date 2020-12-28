/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

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

/**
 * Set the number of particles. Initialize all particles to
 *   first position (based on estimates of x, y, theta and their
 *   uncertainties from GPS) and all weights to 1.
 * Add random Gaussian noise to each particle.
 */
void ParticleFilter::init(
  double x, double y, double theta, double std[]
) {
  Particle p;
  num_particles = 3;  // TODO: Set the number of particles

  // Create a normal (Gaussian) distribution for x, y, and theta, resp.
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Generate particles using the above Gaussian distribution
  for (int i = 0; i < num_particles; i++) {
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
}

/**
 * Add measurements to each particle and add random Gaussian noise.
 */
void ParticleFilter::prediction(
  double delta_t, double std_pos[], double velocity, double yaw_rate
) {
  std::default_random_engine gen;
  double v_yr = velocity / yaw_rate; // vel over yaw_rate
  double yr_dt = yaw_rate * delta_t; // yaw_rate times dt

  for (auto& p : particles) {
    double xf = p.x + v_yr * (sin(p.theta + yr_dt) - sin(p.theta));
    double yf = p.y + v_yr * (cos(p.theta) - cos(p.theta + yr_dt));
    double tf = p.theta + yr_dt;
    std::normal_distribution<double> dist_x(xf, std_pos[0]);
    std::normal_distribution<double> dist_y(yf, std_pos[1]);
    std::normal_distribution<double> dist_theta(tf, std_pos[2]);
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

/**
 * Find the predicted measurement that is closest to each
 *   observed measurement and assign the observed measurement to this
 *   particular landmark.
 */
vector<LandmarkObs> ParticleFilter::dataAssociation(
  vector<LandmarkObs>& predicted,
  vector<LandmarkObs>& observations
) {
  vector<LandmarkObs> associations;
  for (auto& o : observations) {
    double min_dist = -1.0;
    int min_id = 0;
    double min_x = 0.0;
    double min_y = 0.0;
    for (auto& p : predicted) {
      double d = dist(o.x, o.y, p.x, p.y);
      if (min_dist < 0 || d < min_dist) {
        min_dist = d;
        min_id = p.id;
        min_x = p.x;
        min_y = p.y;
      }
    }
    LandmarkObs assoc;
    assoc.id = min_id;
    assoc.x = min_x;
    assoc.y = min_y;
    associations.push_back(assoc);
  }
  return associations;
}

/*
 * Update the weights of each particle using a multi-variate
 * Gaussian distribution.
 */
void ParticleFilter::updateWeights(
  double sensor_range, double std_landmark[],
  const vector<LandmarkObs> &observations,
  const Map &map_landmarks
) {
  for (int i = 0; i < particles.size(); i++) {
    Particle p = particles[i];
    double c = cos(p.theta);
    double s = sin(p.theta);
    // Construct a list of predicted mesaurements consisting of
    // all the landmarks within the sensor range from this particle.
    vector<LandmarkObs> predicted;
    vector<LandmarkObs> t_obs;
    LandmarkObs landmark;
    for (auto& lm : map_landmarks.landmark_list) {
      if (dist(lm.x_f, lm.y_f, p.x, p.y) <= sensor_range) {
        landmark.id = lm.id_i;
        landmark.x = lm.x_f;
        landmark.y = lm.y_f;
        predicted.push_back(landmark);
      }
    }
    for (auto& lo : observations) {
      // Transform each measured position into the map coordinate system
      // based on the current position of the particle.
      landmark.id = 0;
      landmark.x = p.x + (c * lo.x) - (s * lo.y);
      landmark.y = p.y + (s * lo.x) + (c * lo.y);
      t_obs.push_back(landmark);
    }
    // Data association - find nearest neighbour from prediction
    // and associate the actual landmark for each observation.
    vector<LandmarkObs> associated_obs = dataAssociation(predicted, t_obs);
    // Calculate multivariate Gaussian probability.
    double x_std = std_landmark[0];
    double y_std = std_landmark[1];
    double total_weight = 1.0;
    for (int j = 0; j < t_obs.size(); j++) {
      double x = t_obs[j].x;
      double y = t_obs[j].y;
      double mx = associated_obs[j].x;
      double my = associated_obs[j].y;
      double d = 2 * M_PI * x_std * y_std;
      double e = (x - mx) * (x - mx) / (2 * x_std * x_std)
               + (y - my) * (y - my) / (2 * y_std * y_std);
      double prob = exp(-e) / d;
      total_weight *= prob;
    }
    // Set the weight for this particle
    weights[i] = total_weight;
  } // end for each particle
}

/**
 * Resample particles with replacement with probability proportional
 *   to their weights.
 */
void ParticleFilter::resample() {
  std::default_random_engine gen;
  std::discrete_distribution<> d(weights.begin(), weights.end());
  vector<Particle> previous = particles;
  particles.clear();
  for (int i = 0; i < num_particles; i++) {
    particles.push_back(previous[d(gen)]);
  }
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