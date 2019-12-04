#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

#include <cmath>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
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
    double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);
 

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    float z_max_ = 6.0;
    float variance_hit_ = 1.0;
    float lambda_short_ = 0.5;
    float weight_hit_   = 0.25;
    float weight_short_ = 0.25;
    float weight_max_   = 0.25;
    float weight_rand_  = 0.25;

    int8_t odds_threshold_ = 8;
    int valid_threshold_ = 5;
    float probability_sensor_ray(float z_t, float z_star);
    float get_hit_point(float x0_m, float y0_m, float theta, const OccupancyGrid& map);
};

#endif // SLAM_SENSOR_MODEL_HPP
