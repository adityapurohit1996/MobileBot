#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

#include <cmath>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/particles_t.hpp>
#include <common/grid_utils.hpp>
#include <lcmtypes/lidar_t.hpp>

/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double UpdateGroundTruth(const pose_xyt_t& pose, const lidar_t& scan);
    double GroundTruthLikelihood(const lidar_t& scan, const OccupancyGrid& map);
    double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);
    double likelihood(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan, const OccupancyGrid& map);
    void ShowLaser(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan);
    void ShowHit(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan, const OccupancyGrid& map);
    bool JudgeMeasurement(double map_likelihood);
    // particles for debuging visialization
    std::vector<particle_t> debug_particles_;
    // Debug
    pose_xyt_t ground_true_pose_;
    pose_xyt_t previous_ground_pose_;
    particle_t map_particle_;

private:
    
    float z_max_ = 10.0;
    float variance_hit_ = 0.2;
    float lambda_short_ = 0.5;
    float weight_hit_   = 0.9;
    float weight_short_ = 0.0;
    float weight_0_   = 0.1;
    float weight_rand_  = 0.0;

    int8_t odds_threshold_ = 40;
    int8_t unknown_threshold_ = -40;
    int valid_threshold_ = 5;

    int likelihood_amplify = 12.0;
    // Remove bad measurements
    float likelihood_sum_ = 0.0;
    float likelihood_count_ = 0.0;

    float probability_sensor_ray(float z_t, float z_star);
    float get_hit_point(float x0_m, float y0_m, float theta, const OccupancyGrid& map);
};

#endif // SLAM_SENSOR_MODEL_HPP
