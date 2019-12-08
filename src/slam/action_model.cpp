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
 	float x = odometry.x;
 	float y = odometry.y;
 	float theta = odometry.theta;

 	float dx = x - x_;
	float dy = y - y_;
	float dtheta = theta - theta_;

	ds_ = sqrt(dx * dx + dy * dy);
	alpha_ = atan2(dy, dx) - theta_;
	alpha2_ = dtheta - alpha_;
 	
	// update internal value
	x_ = x;
	y_ = y;
	theta_ = theta;
	u_time_ = odometry.utime;
	// std::cout <<"=======================================" << std::endl;
	// std::cout << "x: " << x << ", y:" << y << ", theta:" << theta << std::endl;
	// std::cout << "ds: " << ds_ << ", alpha:" << alpha_ << ", alpha2" << alpha2_ << std::endl;
	if((ds_ >= move_threshold_) || (abs(alpha_) >= move_threshold_) || (abs(alpha2_) >= move_threshold_)) {
		return true;
	}
	else{
		return false;
	}
}

particle_t ActionModel::applyAction(const particle_t& sample)
{
    // create distribution
	std::normal_distribution<> de1{0, k1_ * alpha_};
	std::normal_distribution<> de2{0, k2_ * ds_};
	std::normal_distribution<> de3{0, k1_ * alpha2_};
	float e1 = de1(gen);
    float e2 = de2(gen);
	float e3 = de3(gen);

    float new_x = sample.pose.x + (ds_ + e2) * cos(sample.pose.theta + alpha_ + e1);
	float new_y = sample.pose.y + (ds_ + e2) * sin(sample.pose.theta + alpha_ + e1);
	float new_theta = sample.pose.theta + alpha_ + alpha2_ + e1 + e3;

    // normalize new_theta
	while(new_theta <= 0) {
		new_theta += 2 * M_PI;
	}
	while(new_theta > 2 * M_PI) {
		new_theta -= 2 * M_PI;
	}
	particle_t new_sample;
	new_sample.pose.x = new_x;
	new_sample.pose.y = new_y;
	new_sample.pose.theta = new_theta;
	new_sample.pose.utime = u_time_;

	new_sample.parent_pose.x = sample.pose.x;
	new_sample.parent_pose.y = sample.pose.y;
	new_sample.parent_pose.theta = sample.pose.theta;
	new_sample.parent_pose.utime = sample.pose.utime;
	
    return new_sample;
}
