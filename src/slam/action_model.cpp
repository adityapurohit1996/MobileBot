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
 	delta_s = sqrt(delta_s_2);
 	x_previous = x_current;
 	y_previous = y_current;
 	theta_previous = theta_current;

    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

	

    float e1 = d_alpha(gen);
    float e2 = d_delta_s(gen);
	float e3 = d_delta_theta_alpha(gen);

	
    float x_particle = x_previous + (delta_s + e2)*cos(theta_previous + alpha + e1);
    float y_particle = y_previous + (delta_s + e2)*sin(theta_previous + alpha + e1);
    float theta_particle = theta_previous + e1 + e3;

	sample.pose = (x_particle, y_particle, theta_particle);
	sample.parent_pose = (x_previous, y_previous, theta_previous);
	



    return sample;
}
