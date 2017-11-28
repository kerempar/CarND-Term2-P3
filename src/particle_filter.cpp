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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initialize all particles to first position (based on estimates of
	// x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Adds random Gaussian noise to each particle.
  
  default_random_engine gen;
  
  // get standard deviations for x, y, and yaw for readibility
  double std_x     = std[0]; // standard deviation of x [m]
	double std_y     = std[1]; // standard deviation of y [m]
  double std_theta = std[2]; // standard deviation of yaw [rad]
  
  if (!is_initialized) {
    
    cout << "initialization..." << endl;
    
    // set the number of particles
    num_particles = 100;
    
    // resize vectors accordingly
    particles.resize(num_particles);
    weights.resize(num_particles);
    
    // create normal (Gaussian) distributions for x, y and theta
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    
    cout << "x: " << x << " y: " << y << " theta: " << theta << endl;
    
    // initialize all particles to first position
    // based on estimates of x, y, theta and their uncertainties from GPS
    // add random Gaussian noise to each particle.
    for (int i = 0; i < num_particles; i++)  {
      particles[i].id = i;
      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);
      
      // initialize all weights to 1
      particles[i].weight = 1.0;
      weights[i] = 1.0;
      
      particles[i].associations.clear();
      particles[i].sense_x.clear();
      particles[i].sense_y.clear();
      
      //cout << "x: " << particles[i].x << " y: " << particles[i].y << " theta: " << particles[i].theta << endl;
    }
    
    is_initialized = true;
  }
  
} // init


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Predicts the state for the next time step using the process model
  // Adds measurements to each particle and adds random Gaussian noise.
  
  default_random_engine gen;
  
  // get standard deviations for x, y, and yaw for readibility
  double std_x     = std_pos[0]; // standard deviation of x [m]
  double std_y     = std_pos[1]; // standard deviation of y [m]
  double std_theta = std_pos[2]; // standard deviation of yaw [rad]
  
  // create normal (Gaussian) distributions for x, y and theta
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);
  
  double x, y, theta;
  double x_f, y_f, theta_f;
  
  cout << endl << "prediction..." << endl;
  cout << "velocity: " << velocity << " yaw_rate: " << yaw_rate << endl;
  
  for (int i = 0; i < num_particles; i++)  {
    // get x, y, theta for readibility
    x     = particles[i].x;
    y     = particles[i].y;
    theta = particles[i].theta;
    
    if (fabs(yaw_rate) < 0.001) {
      // equations for updating x, y when the yaw rate is equal to zero
      x_f     = x + velocity * delta_t * cos(theta);
      y_f     = y + velocity * delta_t * sin(theta);
      theta_f = theta;
    }
    else {
      // equations for updating x, y and the yaw angle when the yaw rate is not equal to zero
      x_f     = x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
      y_f     = y + (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
      theta_f = theta + yaw_rate * delta_t;
    }
    
    // set new values
    particles[i].x = x_f;
    particles[i].y = y_f;
    particles[i].theta = theta_f;
    
    // add random Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    
    //cout << "x: " << particles[i].x << " y: " << particles[i].y << " theta: " << particles[i].theta << endl;
  }
  
} // prediction


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // Finds which observations correspond to which landmarks (likely by using
  // a nearest-neighbors data association
  
  double distance, min_distance;
  double x_obs, y_obs;
  double x_landmark, y_landmark;
  long   predicted_landmark_id;
  
  long   num_observations = observations.size();
  
  for (int i=0; i < num_observations; i++)  {
    x_obs = observations[i].x;
    y_obs = observations[i].y;
    
    //cout << "observation: " << i << " x_obs: " << x_obs << " y_obs: " << y_obs;
    
    min_distance = LONG_MAX;
    
    // find the nearest landmark to this observation
    for (int j=0; j < predicted.size(); j++)  {
      
      x_landmark = predicted[j].x;
      y_landmark = predicted[j].y;
        
      // calculate Euclidian distance
      distance = dist(x_obs, y_obs, x_landmark, y_landmark);
        
      if (distance < min_distance) {
        predicted_landmark_id = predicted[j].id;
        min_distance = distance;
      }
    }
    // assign this particular landmark to the observed measurement
    observations[i].id = predicted_landmark_id;
    
    //cout << " nearest: " << predicted_landmark_id << " distance: " << min_distance << endl;
  }
} // dataAssociation

    
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // Updates the weights for each particle based on the likelihood of the observed measurements
  
  cout << "update weights..." << endl;
  
  // standard deviations for landmark measurements, x and y
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];

  // x, y coordinates and yaw of the particle
  double x_part, y_part, theta;
  
  // observed measurement x, y coordinates in vehicle coordinates
  double x_obs, y_obs;
  
  // observed measurement x, y coordinates in map coordinates
  double x_map, y_map;
  
  // coordinates of the nearest landmark in the map
  double mu_x, mu_y;
  
  // landmark ID
  long id;
  
  // variables for intermediate terms of multivariate Gaussian probability
  double gauss_norm, exponent, mvgp;

  long num_observations = observations.size();
  
  // vector for observations in map coordinates
  vector<LandmarkObs> observations_m;
  observations_m.resize(num_observations);
  
  long num_landmarks = map_landmarks.landmark_list.size();
  
  // vector for predicted landmarks in the map (list of landmarks in sersor range)
  vector<LandmarkObs> predicted;
  
  // calculate normalization term for multivariate-Gaussian distribution
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
  
  for (int i=0; i < num_particles; i++)  {

    // get particle coordinates for readibility
    x_part = particles[i].x;
    y_part = particles[i].y;
    theta  = particles[i].theta;
    
    // transform observation coordinates to map coordinates
    for (int j=0; j < num_observations; j++)  {
      
      // apply homogenous transformation
      x_obs = observations[j].x;
      y_obs = observations[j].y;
      
      //cout << "x_obs: " << x_obs << " y_obs: " << y_obs << endl;
      
      x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      
      //cout << "x_map: " << x_map << " y_map: " << y_map << endl;
      
      observations_m[j].x = x_map;
      observations_m[j].y = y_map;
    }
    
    // find predicted landmarks in sensor range
    predicted.clear();
    for (int j=0; j < num_landmarks; j++)  {
      mu_x = map_landmarks.landmark_list[j].x_f;
      mu_y = map_landmarks.landmark_list[j].y_f;
      id   = map_landmarks.landmark_list[j].id_i;
      
      // check if landmark is in sensor range
      if ((fabs(x_part - mu_x) < sensor_range) && (fabs(y_part - mu_y) < sensor_range)) {
      
        LandmarkObs obs;
        obs.x = mu_x;
        obs.y = mu_y;
        obs.id = id;
        predicted.push_back(obs);
        
        //cout << " id: " << id << " mu_x: " << mu_x << " mu_y: " << mu_y << endl;
      }
    }

    //cout << "number of candidates: " << predicted.size() << endl;
    
    if (predicted.size() > 0) {
      
      // call associations to find the nearest landmarks in the map
      dataAssociation(predicted, observations_m);
      
      // set associations
      vector<int> associations;
      vector<double> sense_x;
      vector<double> sense_y;
      
      associations.resize(num_observations);
      sense_x.resize(num_observations);
      sense_y.resize(num_observations);
      
      for (int j=0; j < num_observations; j++)  {
        associations[j] = observations_m[j].id;
        sense_x[j] = map_landmarks.landmark_list[observations_m[j].id-1].x_f; // landmark x-position in the map
        sense_y[j] = map_landmarks.landmark_list[observations_m[j].id-1].y_f; // landmark y-position in the map
      }
      particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);
      
      // update the weights of each particle using a multi-variate Gaussian distribution
      
      mvgp = 1.0;
      for (int j=0; j < num_observations; j++)  {
        // get coordinates for readibility
        x_obs = observations_m[j].x; // observation x in map coordinates
        y_obs = observations_m[j].y; // observation y in map coordinates
        id = observations_m[j].id;
        mu_x = map_landmarks.landmark_list[id-1].x_f; // landmark x-position in the map
        mu_y = map_landmarks.landmark_list[id-1].y_f; // landmark y-position in the map
      
        //cout << "observation_m: " << j << endl;
        //cout << "x_obs: " << x_obs << " y_obs: " << y_obs << endl;
        //cout << "id: " << id << " mu_x: " << mu_x << " mu_y: " << mu_y << endl;
        
        // calculate multivariate-Gaussian propability
        
        // calculate exponent
        exponent = (pow((x_obs - mu_x), 2) / (2 * pow(sig_x, 2))) + (pow((y_obs - mu_y), 2) / (2 * pow(sig_y, 2)));
    
        // calculate weight using normalization terms and exponent
        mvgp = gauss_norm * exp(-exponent);

        // to get the final weight multiply all the calculated measurement probabilities together
        particles[i].weight = particles[i].weight * mvgp;
      }
    }
    weights[i] = particles[i].weight;
    //cout << "weight: " << weights[i] << endl << endl;
  } // particles
  
} // updateWeights

    
void ParticleFilter::resample() {
  // Resamples from the updated set of particles to form the new set of particles
  
  // Resamples particles with replacement with probability proportional to their weight.
	
  cout << "resampling..." << endl;
  
  double sum_weight = 0.0;
  
  default_random_engine gen;
  discrete_distribution<int> dist_index(std::begin(weights), std::end(weights));
  
  // new set of particles
  vector<Particle> new_particles;
  new_particles.resize(num_particles);
  
  // selected index
  int index;
  
  for (int i=0; i < num_particles; i++)  {
    index = dist_index(gen);
    //cout << "selected index: " << index << " weight: " << weights[index] << endl;
    new_particles[i] = particles[index];
    new_particles[i].id = i;
  }
  
  particles = new_particles;
  
  // updates weights vector and sum of weights
  for (int i=0; i < num_particles; i++)  {
    weights[i] = particles[i].weight;
    sum_weight += particles[i].weight;
  }
  
  // normalize weights
  for (int i=0; i < num_particles; i++)  {
    particles[i].weight = particles[i].weight/sum_weight;
    weights[i] = weights[i]/sum_weight;
  }
  
} // resample


Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
  // Set a particles list of associations, along with the associations calculated world x,y coordinates
  // This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected

	// particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
