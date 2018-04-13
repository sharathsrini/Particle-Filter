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
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  number_of_particles = 100;

  default_random_engine gen;

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Generate particles with normal distribution with mean on GPS values.
  for (int i = 0; i < number_of_particles; i++) {
      Particle first_particle;

      first_particle.id = i;
      first_particle.x = dist_x(gen);
      first_particle.y = dist_y(gen);
      first_particle.theta = dist_theta(gen);
      first_particle.weight = 1.0;

      particles.push_back(first_particle);
  }

  is_initialized = true;

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

  for (int i=0; i < number_of_particles; i++){
      if ( fabs(yaw_rate) < 0.0001 ) { // When yaw is not changing.
	  particles[i].x += velocity * delta_t * cos( particles[i].theta );
	  particles[i].y += velocity * delta_t * sin( particles[i].theta );
	  // yaw continue to be the same.
      } else {
	  particles[i].x += velocity / yaw_rate * ( sin( particles[i].theta + yaw_rate * delta_t ) - sin( particles[i].theta ) );
	  particles[i].y += velocity / yaw_rate * ( cos( particles[i].theta ) - cos( particles[i].theta + yaw_rate * delta_t ) );
	  particles[i].theta += yaw_rate * delta_t;
      }

      particles[i].x += dist_x(gen);
      particles[i].y += dist_y(gen);
      particles[i].theta += dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

  for(int i=0; i < observations.size(); i++){
      double min_Dist = numeric_limits<double>::max();
      double obs_x = observations[i].x;
      double obs_y = observations[i].y;
      int matching_Landmark_ID = -1;

      for (int j=0; j < predicted.size(); j++){
	  double pred_x = predicted[j].x;
	  double pred_y = predicted[j].y;
	  double currentDist = dist(obs_x, obs_y, pred_x, pred_y);

	  if (currentDist < min_Dist){
	      min_Dist = currentDist;
	      matching_Landmark_ID = predicted[j].id;
	  }
      }
      observations[i].id = matching_Landmark_ID;
  }



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  for (int i = 0; i < number_of_particles; i++) {
      double x = particles[i].x;
      double y = particles[i].y;
      double theta = particles[i].theta;

      // Step 1 -- find landmarks within the sensor range
      vector<LandmarkObs> predicted;
      for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
	  LandmarkObs pred;
	  pred.id = map_landmarks.landmark_list[j].id_i;
	  pred.x = map_landmarks.landmark_list[j].x_f;
	  pred.y = map_landmarks.landmark_list[j].y_f;

	  if (fabs(pred.x - x) <= sensor_range && fabs(pred.y - y) <= sensor_range) {
	      predicted.push_back(pred);
	  }
      }


      // Transform each observations to map coordinates
      vector<LandmarkObs> transformedObservations;
      for (int j=0; j < observations.size(); j++){
	  LandmarkObs transformedObs;
	  transformedObs.id = j;
	  transformedObs.x = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
	  transformedObs.y = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
	  transformedObservations.push_back(transformedObs);
      }

      // Associate the landmark in range to landmark observations
      dataAssociation(predicted, transformedObservations);

      //Update the particle weight using Multivariate Gaussian distribution
      particles[i].weight = 1.0;


      double sigma_X_Squared = std_landmark[0]*std_landmark[0];
      double sigma_Y_Squared = std_landmark[2]*std_landmark[2];
      double normalizing_Factor = (1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[0]));

      for (int j = 0; j < transformedObservations.size(); j++){
	  double trans_Obs_X = transformedObservations[j].x;
	  double trans_Obs_Y = transformedObservations[j].y;
	  double trans_Obs_ID = transformedObservations[j].id;

	  for (int j_P = 0; j_P < predicted.size(); j_P++){
	      double pred_X = predicted[j_P].x;
	      double pred_Y = predicted[j_P].y;
	      double predID = predicted[j_P].id;

	      if (trans_Obs_ID == predID){
		  double dx_Squared = pow(trans_Obs_X - pred_X, 2);
		  double dy_Squared = pow(trans_Obs_Y - pred_Y, 2);
		  double weight = normalizing_Factor * exp(-(dx_Squared/(2*sigma_X_Squared) + dy_Squared/(2*sigma_Y_Squared)));
		  particles[i].weight *= weight;

		  break;
	      }
	  }
      }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  default_random_engine gen;


  vector<double> weights;
  double maxWeight = numeric_limits<double>::min();
  for (int i=0; i < number_of_particles; i++){
      weights.push_back(particles[i].weight);
      if ( particles[i].weight > maxWeight ) {
	  maxWeight = particles[i].weight;
      }
  }

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> maxW(0.0, maxWeight);

  uniform_int_distribution<int> particleIdx(0, number_of_particles - 1);
  int index = particleIdx(gen);

  // Re-sampling Step
  vector<Particle> resampledParticles;
  double beta = 0;
  for (int i=0; i < number_of_particles; i++){
      beta += maxW(gen) * 2.0;
      while( beta > weights[index]){
	  beta -= weights[index];
	  index = (index + 1) % number_of_particles;
      }
      resampledParticles.push_back(particles[index]);
  }
  particles = resampledParticles;

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

    return particle;
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
