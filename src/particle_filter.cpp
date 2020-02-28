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

static std::default_random_engine gen;
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
    if (is_initialized)
    {
        return;
    }
  num_particles = 200;  // TODO: Set the number of particles


  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for(int i = 0; i < num_particles; ++i)
  {
    Particle P;
    P.id = i;
    P.x = dist_x(gen);
    P.y = dist_y(gen);
    P.theta = dist_theta(gen);
    P.weight = 1.0;
    particles.push_back(P);
  }
  is_initialized = true;

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
  // define sensor random Gaussian noise
  normal_distribution<double> Noise_x(0, std_pos[0]);
  normal_distribution<double> Noise_y(0, std_pos[1]);
  normal_distribution<double> Noise_theta(0, std_pos[2]);

  for(int i = 0; i < num_particles; ++i)
  {
    // to prevent divided by 0
            if(fabs(yaw_rate) < 0.00001)
            {
              particles[i].x += delta_t * velocity * cos(particles[i].theta);
              particles[i].y += delta_t * velocity * sin(particles[i].theta);
            }
            else
            {
              particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
              particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
              particles[i].theta += yaw_rate * delta_t;
            }
    // Add Noise
      particles[i].x += Noise_x(gen);
      particles[i].y += Noise_y(gen);
      particles[i].theta += Noise_theta(gen);

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
    for (unsigned int i = 0; i < observations.size(); i++)
    {
        // init a closest Distance
        double ClosestDistance = std::numeric_limits<double>::max();

        // init id of landmark from map placeholder to be associated with the observation
        int map_id = -10;

        for (unsigned int j = 0; j < predicted.size(); j++)
        {
            // get distance between current/predicted landmarks
            double Distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            // Nearest Neighbor
            if (Distance < ClosestDistance)
            {
                ClosestDistance = Distance;
                map_id = predicted[j].id;
            }
        }
        observations[i].id = map_id;
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
  double x_part, y_part, x_obs, y_obs, theta;

  for(int i = 0; i < num_particles; ++i)
  {
      std::vector<int> associations;
      std::vector<double> sense_x;
      std::vector<double> sense_y;

      // coordinates Transformation
      x_part = particles[i].x;
      y_part = particles[i].y;
//      // All Landmark in senserange of this particle location
//      vector<LandmarkObs> predictions;
//      for (unsigned int q = 0; q < map_landmarks.landmark_list.size(); q++)
//      {
//          // get landmarks ID and location
//          float landmark_x = map_landmarks.landmark_list[q].x_f;
//          float landmark_y = map_landmarks.landmark_list[q].y_f;
//          int landmark_id = map_landmarks.landmark_list[q].id_i;
//          double dX = landmark_x - x_part;
//          double dY = landmark_y - y_part;
//
//          // get All landmark in sensor range
//          if (dX * dX + dY * dY < sensor_range * sensor_range)
//          {
//              predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
//          }
//      }
//
//      // Transform observation loaction from Vehicle coordination to Map coordination
//      vector<LandmarkObs> TF_Observation;
//      for (unsigned int j = 0; j < observations.size(); j++)
//      {
//          x_obs = observations[j].x;
//          y_obs = observations[j].y;
//          theta = particles[i].theta;
//          // transform to map x coordinate
//          double x_map;
//          x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
//          particles[i].sense_x.push_back(x_map);
//          // transform to map y coordinate
//          double y_map;
//          y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
//          TF_Observation.push_back(LandmarkObs{ observations[j].id, x_map, y_map });
//      }
//
//      // perform dataAssociation for the predictions and transformed observations on current particle
//      dataAssociation(predictions, TF_Observation);
//
//
//      // reinit weight
//      particles[i].weight = 1.0;
//
//      // Update Weight
//      for (unsigned int j = 0; j < TF_Observation.size(); ++j)
//      {
//
//          // placeholders for observation and associated prediction coordinates
//          double o_x, o_y;
//          o_x = TF_Observation[j].x;
//          o_y = TF_Observation[j].y;
//
//          int associated_prediction = TF_Observation[j].id;
////          double pr_x, pr_y;
////          // get the x,y coordinates of the prediction associated with the current observation
////          for (unsigned int k = 0; k < predictions.size(); ++k)
////          {
////              if (predictions[k].id == associated_prediction)
////              {
////                  pr_x = predictions[k].x;
////                  pr_y = predictions[k].y;
////              }
////          }
//
////          // calculate weight for this observation with multivariate Gaussian
////          double s_x = std_landmark[0];
////          double s_y = std_landmark[1];
////          double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );
//
//          double mu_x, mu_y;
//          // get the x,y coordinates of the prediction associated with the current observation
//          for (unsigned int k = 0; k < predictions.size(); ++k)
//          {
//              if (predictions[k].id == associated_prediction)
//              {
//                  mu_x = predictions[k].x;
//                  mu_y = predictions[k].y;
//              }
//          }
//          double gauss_norm;
//          gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
//          // calculate exponent
//          double exponent;
//          exponent = (pow(mu_x - o_x, 2) / (2 * pow(std_landmark[0], 2)))
//                     + (pow(mu_y - o_y, 2) / (2 * pow(std_landmark[1], 2)));
//
//          // calculate weight using normalization terms and exponent
//          double weight = gauss_norm * exp(-exponent);
//
//          if(weight == 0)
//          {
//              particles[i].weight *= 0.00001;
//          }
//          else
//          {
//              // product of this obersvation weight with total observations weight
//              particles[i].weight *= weight;
//          }
//      }

      // reinit weight
      particles[i].weight = 1.0;
      for(int j=0; j<observations.size(); ++j)
      {

          x_obs = observations[j].x;
          y_obs = observations[j].y;
          theta = particles[i].theta; // -90 degrees

          // transform to map x coordinate
          double x_map;
          x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
          sense_x.push_back(x_map);

          // transform to map y coordinate
          double y_map;
          y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
          sense_y.push_back(y_map);


          // Find Nearest Neighbor
          int NN = -1;
          double Distance;
          double ClosestDistance = 10000000;
          float x_f, y_f;
          int ID = -10;
          for(int q=0; q<map_landmarks.landmark_list.size(); ++q)
          {
              x_f = map_landmarks.landmark_list[q].x_f;
              y_f = map_landmarks.landmark_list[q].y_f;

              if(fabs(x_f - x_part) <= sensor_range && fabs(y_f - y_part) <= sensor_range)
                  // to save computing source, use sensor_range just in this form
              {
                  Distance = sqrt(pow((x_map - x_f), 2) + pow((y_map - y_f), 2));
                  if(Distance <= ClosestDistance)
                  {
                      ClosestDistance = Distance;
                      NN = q;
                      ID = map_landmarks.landmark_list[q].id_i;
                  }
              }
          }
          associations.push_back(ID);

          // Update Weight

          // calculate normalization term
          if(NN != -1)
          {
              double mu_x = map_landmarks.landmark_list[NN].x_f;
              double mu_y = map_landmarks.landmark_list[NN].y_f;
              double gauss_norm;
              gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

              // calculate exponent
              double exponent;
              exponent = (pow(x_map - mu_x, 2) / (2 * pow(std_landmark[0], 2)))
                         + (pow(y_map - mu_y, 2) / (2 * pow(std_landmark[1], 2)));

              // calculate weight using normalization terms and exponent
              particles[i].weight *= gauss_norm * exp(-exponent);
          }
          else
          {
              particles[i].weight *= 0.000001;
          }

          SetAssociations(particles[i], associations, sense_x, sense_y);
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

    vector<Particle> new_particles;
    vector<double> weights;
    for (int i = 0; i < num_particles; ++i)
    {
        weights.push_back(particles[i].weight);
    }
    // get Max weight
    double Max_weight = *max_element(weights.begin(), weights.end());

    // get start index of resampling wheel
//    std::uniform_int_distribution<int> RandomIndex(0, num_particles - 1);
    std::discrete_distribution<int> RandomIndex(0, num_particles -1);
    int index = RandomIndex(gen);

    std::uniform_real_distribution<double> RandNum(0.0, Max_weight);

    double beta = 0.0;

    for(int i=0; i<num_particles; ++i)
    {
        beta += RandNum(gen) * 2;
        while(weights[index]<beta)
        {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;

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