#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <random>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
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

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d_alpha(0, k1*alpha);
    std::normal_distribution<float> d_delta_s(0, k2*delta_s);
    std::normal_distribution<float> d_delta_theta_alpha(0, k1*delta_theta_alpha);

}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
 	x_current = odometry.x;
 	y_current = odometry.y;
 	theta_current = odometry.theta;

 	delta_x = x_current - x_previous;
 	delta_y = y_current - y_previous;
 	delta_theta = theta_current - theta_previous;
 	
 	alpha = atan2(delta_y, delta_x) - theta_previous;
 	delta_theta_alpha = delta_theta - alpha;
 	delta_s_2 = delta_x*delta_x + delta_y*delta_y;
 	delta_s = (delta_s_2)**0.5;
 	x_previous = x_current;
 	y_previous = y_current;
 	theta_previous = theta_current;

    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

	std::normal_distribution<float> d_alpha(0, k1*abs(alpha));
    std::normal_distribution<float> d_delta_s(0, k2*abs(delta_s));
    std::normal_distribution<float> d_delta_theta_alpha(0, k1*abs(delta_theta_alpha));

    float e1 = d_alpha(gen);
    float e2 = d_delta_s(gen));
	float e3 = d_delta_theta_alpha(gen);

    x = x + (delta_s + e2)*cos(theta_previous + alpha + e1);
    y = y + (delta_s + e2)*sin(theta_previous + alpha + e1);
    theta_current = theta_previous + e1 + e3;



    return sample;
}
