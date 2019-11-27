#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
int64_t utime_previous = 0;
    int64_t utime_current = utime_previous;

    float x_previous = 0;
    float y_previous = 0;
    float theta_previous= 0;

    float x_current = x_previous;
    float y_current = y_previous;
    float theta_current = theta_previous;

    float alpha = 0;
    float delta_s = 0;
    float delta_s_2 = 0;
    float delta_theta = 0;
    float delta_theta_alpha = 0;

    float delta_y = 0;
    float delta_x = 0;

    float k1 = 0.1;
    float k2 = 0.1;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<float> d_alpha{0, k1*alpha};
    std::normal_distribution<float> d_delta_s{0, k2*delta_s};
    std::normal_distribution<float> d_delta_theta_alpha{0, k1*delta_theta_alpha};



    ActionModel(void);
    
    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& odometry);
    
    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);
    
private:
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
};

#endif // SLAM_ACTION_MODEL_HPP
