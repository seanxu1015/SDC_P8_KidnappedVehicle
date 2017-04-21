#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

int main() {
    int time_steps_before_lock_required = 100;
    double max_runtime = 45;
    double max_translation_error = 1;
    double max_yaw_error = 0.05;

    int start = clock();
    double delta_t = 0.1;
    double sensor_range = 50;

    double sigma_pos[3] = {0.3, 0.3, 0.01};
    double sigma_landmark[2] = {0.3, 0.3};

    default_random_engine gen;
    normal_distribution<double> N_x_init(0, sigma_pos[0]);
    normal_distribution<double> N_x_init(0, sigma_pos[1]);
    normal_distribution<double> N_theta_init(0, sigma_pos[2]);
    normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
    normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
    double x_x, n_y, n_theta, n_range, n_heading;

    Map map;
    if (!read_map_data("data/map_data.txt", map)) {
        cout << "Error: Could not open map file" << endl;
        return -1;
    }

    vector<control_s> position_meas;
    if (!read_control_data("data/control_data.txt", position_meas)) {
        cout << "Error: Could not open position/control measurement file" << endl;
        return -1;
    }

    vector<ground_truth> gt;
    if (!read_gt_data("data/gt_data.txt", gt)) {
        cout << "Error: Could not open ground truth data file" << endl;
        return -1;
    }

    int num_time_steps = position_meas.size();
    ParticleFilter pf;
    double total_error[3] = {0, 0, 0};
    double cum_mean_error[3] = {0, 0, 0};

    for (int i=0; i<num_time_steps; ++i) {
        cout << "Time step: " << i << endl;
        ostringstream file;
        file << "data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
        vector<LandmarkObs> observations;
        if (!read_landmark_data(file.str(), observations)) {
            cout << "Error: Could not open observation file " << i+1 << endl;
            return -1;
        }

        if (!pf.initialized()) {
            n_x = N_x_init(gen);
            n_y = N_y_init(gen);
            n_theta = N_theta_init(gen);
            pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);
        } 
        else {
            pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, 
                          position_meas[i-1].yawrate)
        }
        vector<LandmarkObs> noisy_observations;
        LandmarkObs obs;
        for (int j=0; j<observations.size(); ++j) {
            n_x = N_obs_x(gen);
            n_y = N_obs_y(gen);
            obs = observations[j];
            obs.x = obs.x + n_x;
            obs.y = obs.y + n_y;
            noisy_observations.push_back(obs);
        }
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        vector<Particle> particles = pf.particles;
        int num_particles = particles.size();
        double highest_weight = 0.0;
        Particle best_particle;
        for (int i=0; i<num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
                highest_weight = particles[i].weight;
                best_particle = particle[i];
            }
        }
        double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, 
                best_particle.y, best_particle.theta);

        for (int j=0; j<3; ++j) {
            total_error[j] += avg_error[i];
            cum_mean_error[j] = total_error[j] / (double) (i+1);
        }
        cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << "y" <<
            cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;

        if (i>=time_steps_before_lock_required) {
            if (cum_mean_error[0] > max_translation_error || 
                cum_mean_error[1] > max_translation_error ||
                cum_mean_error[2] > max_yaw_error) {
                if (cum_mean_error[0] > max_translation_error) {
                    cout << "Your x error: " << cum_mean_error[0] << endl;
                }
                else if (cum_mean_error[1] > max_translation_error) {
                    cout << "Your y error: " << cum_mean_error[1] << endl;
                }
                else {
                    cout << "Your yaw error: " << cum_mean_error[2] << endl;
                }
                return -1;
            }
        }
    }
    int stop = clock();
    double runtime = (stop - start) / double(CLOCKS_PER_SEC);
    cout << "Runtime (sec): " << runtime << endl;


}
