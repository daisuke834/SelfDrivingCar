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
#include <random> // Added

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles=1000;
	particles.clear();
	weights.clear();
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	for (int i = 0; i < num_particles; ++i) {
		Particle sample;
		sample.id = i;
		sample.x = dist_x(gen);
		sample.y = dist_y(gen);
		sample.theta = dist_theta(gen);
		sample.weight=1.0/((double)num_particles);
		particles.push_back(sample);
		weights.push_back(sample.weight);
	}
	is_initialized=true;
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
	for (int i = 0; i < num_particles; ++i) {
		double x_new, y_new, theta_new;
		if(abs(yaw_rate)>0.000001){
			x_new = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			y_new = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			theta_new = particles[i].theta + yaw_rate*delta_t;
		}
		else{
			x_new = particles[i].x + velocity *cos(particles[i].theta)*delta_t;
			y_new = particles[i].y + velocity *sin(particles[i].theta)*delta_t;
			theta_new = particles[i].theta;
		}
		particles[i].x = x_new + dist_x(gen);
		particles[i].y = y_new + dist_y(gen);
		particles[i].theta = theta_new + dist_theta(gen);
	}

}

double calc_norm(double x,double y)
{
	return sqrt(x*x+y*y);
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	vector<double> distances; 
	for (int i = 0; i < predicted.size(); ++i){
		double min_distance=10000.0;
		int min_obs_idx=-1;
		for(int j=0; j< observations.size(); ++j){
			double dx = predicted[i].x - observations[j].x;
			double dy = predicted[i].y - observations[j].y;
			double distance = calc_norm(dx,dy);
			if(distance<min_distance){
				min_obs_idx=j;
				min_distance=distance;
			}
		}
		observations[min_obs_idx].id = i;
	}

	/*
	for(int j=0; j< observations.size(); ++j){
		int id = observations[j].id;
		double dx = predicted[id].x - observations[j].x;
		double dy = predicted[id].y - observations[j].y;
		double delta = calc_norm(dx,dy);
		cout << id << "\t" << delta << "\t" << predicted[id].x << "\t" << predicted[id].y << "\t" << observations[j].x << "\t" << observations[j].y << endl;
		 << "\t" << 
	}
	cout <<"**************************"<<endl;
	*/
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

	double coef = 1.0/(2.0*M_PI*std_landmark[0]*std_landmark[1]);
	double weight_sum=0.0;

	for (int i = 0; i < num_particles; ++i) {
		double xp =  particles[i].x;
		double yp =  particles[i].y;
		double theta = particles[i].theta;
		double costheta = cos(theta);
		double sintheta = sin(theta);
		vector<LandmarkObs> predicted;
		predicted.clear();
		for(int j=0; j<map_landmarks.landmark_list.size(); ++j){
			LandmarkObs lnmk;
			lnmk.id=j;
			lnmk.x = (double)map_landmarks.landmark_list[j].x_f;
			lnmk.y = (double)map_landmarks.landmark_list[j].y_f;
			//predicted.push_back(lnmk);
			if(calc_norm(lnmk.x-xp, lnmk.y-yp) < sensor_range){
				predicted.push_back(lnmk);
			}
		}
		vector<LandmarkObs> observations_map;
		observations_map.clear();
		for(int j=0; j<observations.size(); ++j){
			double xc = observations[j].x;
			double yc = observations[j].y;
			// if(abs(xc)>sensor_range)continue;
			// if(abs(yc)>sensor_range)continue;
			if(calc_norm(xc, yc) < sensor_range){
				double xm = xc*costheta - yc*sintheta + xp;
				double ym = xc*sintheta + yc*costheta + yp;
				LandmarkObs c_observation;
				c_observation.x=xm;
				c_observation.y=ym;
				c_observation.id=-1;
				observations_map.push_back(c_observation);
			}
		}
		dataAssociation(predicted, observations_map);
		if(observations_map.size()==0){
			particles[i].weight=0.0;
		}
		else{
			particles[i].weight=1.0;
			for(int j=0; j<observations_map.size(); ++j){
				int landmark_id = observations_map[j].id;
				if(landmark_id<0)continue;
				double xm = observations_map[j].x;
				double ym = observations_map[j].y;
				double xl = predicted[landmark_id].x;
				double yl = predicted[landmark_id].y;

				double stderr_x = (xm - xl)*(xm - xl)/(2*std_landmark[0]*std_landmark[0]);
				double stderr_y = (ym - yl)*(ym - yl)/(2*std_landmark[1]*std_landmark[1]);
				double temp = - (stderr_x + stderr_y);
				// cout << j << "\t" << landmark_id << "\t" << coef * exp(temp) << "\t" << stderr_x << "\t" << stderr_y << "\t" << xm << "\t" << xl << "\t" << ym << "\t" << yl << endl;
				particles[i].weight *= coef * exp(temp);
			}
			weight_sum += particles[i].weight;
		}
	}
	//cout <<"******"<<weight_sum<<endl;
	for (int i = 0; i < num_particles; ++i) {
		particles[i].weight /= weight_sum;
		weights[i] = particles[i].weight;
		//cout << i << "\t" << particles[i].weight<<endl;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	random_device rd;
	mt19937 gen(rd());
	vector<Particle> particles_new;
	discrete_distribution<> dist(weights.begin(), weights.end());
	for (int i = 0; i < num_particles; ++i) {
		int particle_idx = dist(gen);
		//cout << particle_idx << endl;
		Particle sample = particles[particle_idx];
		sample.id=i;
		particles_new.push_back(sample);
	}
	particles = particles_new;
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
