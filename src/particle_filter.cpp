#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Flag, if filter is initialized
  is_initialized = false;

	// Number of particles to draw
	num_particles = 1e3;

  std::random_device rd;
  std::mt19937 gen(rd());
	double std_x, std_y, std_theta; // standard deviations for x, y, and theta

	//sets standard deviations for x, y and theta.
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

	// creates a normal distribution for x, y and theta.
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// reserves memory for vectors of current particles and weights
	particles.reserve(num_particles);
	weights.reserve(num_particles);

	// samples from these normal distrubtions like this:
	// x = dist_x(gen);
	// where "gen" is the random engine initialized earlier.
	double weight = 1.0;//initial value for all the weigths
	for (int i = 0; i < num_particles; ++i) {
		Particle particle_temp;
		// initial x and y positions for particles will be initialized
    // with noisy gps coordiates of the first gps x and y
		particle_temp.id = i;
		particle_temp.x = dist_x(gen);
		particle_temp.y = dist_y(gen);
		particle_temp.theta = dist_theta(gen);
		particle_temp.weight = weight;
    particles.push_back(particle_temp);
    weights.push_back(weight);
		}
		// done initializing
		is_initialized = true;
}


//adds measurements to each particle and random Gaussian noise
// when adding noise were used std::normal_distribution and std::default_random_engine useful.
//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
//  http://www.cplusplus.com/reference/random/default_random_engine/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  //sets standard deviations for x, y, and theta.
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

	// initializes random generator
  std::random_device rd;
  std::mt19937 gen(rd());

	// normal (Gaussian) noise for x, y and theta.
	normal_distribution<double> dist_x(0.0, std_x);
	normal_distribution<double> dist_y(0.0, std_y);
	normal_distribution<double> dist_theta(0.0, std_theta);

	for (int i = 0; i < num_particles; ++i) {
    if (fabs(yaw_rate) > 1e-3) {
  		particles[i].x += velocity/yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
  		particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
	  }
    else{
      particles[i].x += velocity * cos(particles[i].theta) * delta_t;
      particles[i].y += velocity * sin(particles[i].theta) * delta_t;
    }
      particles[i].theta += yaw_rate * delta_t;
  		//adds noise to predicted x, y, theta
  		particles[i].x += dist_x(gen);
  		particles[i].y += dist_y(gen);
  		particles[i].theta += dist_theta(gen);
	}
}


// finds the predicted measurement that is closest to each observed measurement and assigns the
// observed measurement to this particular landmark
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
}


// updates the weights of each particle using a mult-variate Gaussian distribution. More information
// about this distribution is here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// following simplifies calculations
	const double sig_x = std_landmark[0];
	const double sig_y = std_landmark[1];
	const double gauss_norm = 2 * M_PI * sig_x * sig_y;

  // sum of weights, will be used later for normalizaion
	double w_sum = 0.0;

	// distances from observations to nearest landmark
	double dx = 0.0;
	double dy = 0.0;

  // nearest landmark variable
  Map::single_landmark_s nearest_landmark;

	// let's update weights
	for (int i = 0; i < num_particles; ++i) {
				// following simplifies calculations
				const double sin_theta = sin(particles[i].theta);
				const double cos_theta = cos(particles[i].theta);
				double w_tmp = 1.0;

        //clear associations between observation and map
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
				// iterates through obsterations
				for (int j = 0; j < observations.size(); j++) {
							//  transforms from car observation coordinates to map coord system,
							// taking in account rotation of the  coord. system
							LandmarkObs observation;
							observation.x = particles[i].x + (observations[j].x * cos_theta) - (observations[j].y * sin_theta);
							observation.y = particles[i].y + (observations[j].x * sin_theta) + (observations[j].y * cos_theta);

              // this values will be used to show observations with map ids in the simulator
              particles[i].sense_x.push_back(observation.x);
              particles[i].sense_y.push_back(observation.y);


              //flag to check wheater map object in radar range or not
							bool in_range = false;

							// initial value for the distance to the nearest landmark
							// should be big, since we do not know which landmark is the
              // closest
							double nearest_distance = 1e7;
              int obs_map_id = 1e5;//id of a map object associated to the observation
							// search for the closest landmarks
							for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
										// temporary landmark
										Map::single_landmark_s tmp_lm = map_landmarks.landmark_list[k];

										// distance between oberservations and landmarks
										double distance = dist(tmp_lm.x_f, tmp_lm.y_f, observation.x, observation.y);

										// distance comparison
										if (distance < nearest_distance) {
													// updates nearest_distance value
													nearest_distance = distance;
													// updates nearest landmark
													nearest_landmark = tmp_lm;
													// checks whether distance within sensor range
													if (distance < sensor_range) {
																in_range = true;
                                obs_map_id = map_landmarks.landmark_list[k].id_i;
													}
                          // distances from observation to nearest landmark
                          dx = observation.x - nearest_landmark.x_f;
                          dy = observation.y - nearest_landmark.y_f;
										}
							}
							// if distance < sensor_range, updates weight
							if (in_range) {

										// simplifies calculations
										double exponent = dx * dx / (2 * sig_x * sig_x) + dy * dy / (2 * sig_y * sig_y);
                    // weight of a particle is multiplication of weigthed probabilities
                    w_tmp *= exp(-exponent)/gauss_norm;
                    particles[i].associations.push_back(obs_map_id);
							}
							// if not in range, weighted sum should be big
							else {
										w_tmp *= 1e3;
							}
				}

        // updated particle weight
      	particles[i].weight = w_tmp;
        // is used later for normalization
        w_sum += particles[i].weight;
	}
	// weight normalization
	for (int i = 0; i < num_particles; ++i) {
				particles[i].weight /= (w_sum);
				weights[i] = particles[i].weight;
	}
 }


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// std::discrete_distribution produces random integers on the interval [0, n),
	// where the probability of each individual integer i is defined as w
  // i/S, that is the weight of the ith integer divided by the sum of all n weights.
	std::random_device rd_dev;
	std::mt19937 gen(rd_dev());
	std::discrete_distribution<int> dist_weights(weights.begin(), weights.end());
	// resampled particles:
	vector<Particle> new_particles;
  for(int i=0; i<num_particles; ++i) {
               Particle tmp_particle = particles[dist_weights(gen)];
               tmp_particle.id = i;
		           new_particles.push_back(tmp_particle);
               weights[i]=tmp_particle.weight;
		      }
	 // Set particles to resampled values
	 particles = new_particles;
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
