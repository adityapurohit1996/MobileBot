#include <slam/sensor_model.hpp>


SensorModel::SensorModel(void)
{
    // initialization for map
    map_particle_.pose.x = 0.0;
    map_particle_.pose.y = 0.0;
    map_particle_.pose.theta = 0.0;
    map_particle_.pose.utime = 0.0;
    map_particle_.parent_pose.x = 0.0;
    map_particle_.parent_pose.y = 0.0;
    map_particle_.parent_pose.theta = 0.0;
    map_particle_.parent_pose.utime = 0.0;
    map_particle_.weight = 0.0;
}

double SensorModel::UpdateGroundTruth(const pose_xyt_t& pose, const lidar_t& scan) {
    previous_ground_pose_.x = ground_true_pose_.x;
    previous_ground_pose_.y = ground_true_pose_.y;
    previous_ground_pose_.theta = ground_true_pose_.theta;
    previous_ground_pose_.utime = ground_true_pose_.utime;
    
    ground_true_pose_.x = pose.x;
    ground_true_pose_.y = pose.y;
    ground_true_pose_.theta = pose.theta;
    ground_true_pose_.utime = scan.times.back();
};

double SensorModel::GroundTruthLikelihood(const lidar_t& scan, const OccupancyGrid& map)
{
    float log_likelihood = 0;
    float log_likelihood_per_ray = 0;
    int valid_array_count = 0;
    // Update Debug particles everytime computing likelihood
    MovingLaserScan movingScan(scan, previous_ground_pose_, ground_true_pose_);     //full scan
    for(const auto& ray : movingScan) {
        float range_measure = ray.range;
        float range_map = get_hit_point(ray.origin.x, ray.origin.y, ray.theta, map);
        if(range_map > 0) {
            log_likelihood_per_ray = log(probability_sensor_ray(range_measure, range_map));
            // std::cout << "Zt:" << range_measure << ", Zstar:" << range_map << ",L:" << log_likelihood_per_ray << std::endl;
            log_likelihood += log_likelihood_per_ray;
            valid_array_count++;
        }
    }
    
    if(valid_array_count <= valid_threshold_) {
        // If not enough array, it's most likely the sample is not working
        printf("Valid Array Not Enough\n");
        return 1e-8;
    }
    
    valid_array_count /= likelihood_amplify;
    double likelihood = exp(log_likelihood / valid_array_count);
    // std::cout << "------------- Ground Truth ----------------" << std::endl;
    // std::cout << "x: " << ground_true_pose_.x << ", y: " << ground_true_pose_.y << ", t: " << ground_true_pose_.theta;
    // std::cout << ",L: " << likelihood << std::endl;
    return likelihood;
};

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    float log_likelihood = 1e-6;
    float log_likelihood_per_ray = 1e-6;
    double valid_array_count = 1e-6;
    // Start Intepolation over states
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);     //full scan
    for(const auto& ray : movingScan) {
        float range_measure = ray.range;
        float range_map = get_hit_point(ray.origin.x, ray.origin.y, ray.theta, map);
        // std::cout << "map: " << range_map << ", measure: " << range_measure << std::endl;
        if((range_map > 0) && std::abs(range_map) < 10) {
            log_likelihood_per_ray = log(probability_sensor_ray(range_measure, range_map));
            log_likelihood += log_likelihood_per_ray;
            valid_array_count++;
        }
    }

    if(valid_array_count <= valid_threshold_) {
        printf("Valid Array Not Enough\n");
        std::cout << "Alarm" << std::endl;
        return 1e-6;
    }
    valid_array_count /= likelihood_amplify;
    double likelihood = exp(log_likelihood / valid_array_count);
    // std::cout << "------------- Sample ----------------" << std::endl;
    // std::cout << "x: " << sample.pose.x << ", y: " << sample.pose.y << ", t: " << sample.pose.theta;
    // std::cout << "L1: " << log_likelihood << "Num: " << valid_array_count << std::endl;
    return likelihood;
}

double SensorModel::likelihood(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan, const OccupancyGrid& map)
{
    float log_likelihood = 1e-6;
    float log_likelihood_per_ray = 1e-6;
    double valid_array_count = 1e-6;
    // Start Intepolation over states
    MovingLaserScan movingScan(scan, start_pose, end_pose);     //full scan
    for(const auto& ray : movingScan) {
        float range_measure = ray.range;
        float range_map = get_hit_point(ray.origin.x, ray.origin.y, ray.theta, map);
        if((range_map > 0) && std::abs(range_map) < 10) {
            log_likelihood_per_ray = log(probability_sensor_ray(range_measure, range_map));
            log_likelihood += log_likelihood_per_ray;
            valid_array_count++;
        }
    }

    if(valid_array_count <= valid_threshold_) {
        printf("Valid Array Not Enough");
    }
    valid_array_count /= likelihood_amplify;
    double likelihood = exp(log_likelihood / valid_array_count);
    // std::cout << "------------- Sample ----------------" << std::endl;
    // std::cout << "x: " << sample.pose.x << ", y: " << sample.pose.y << ", t: " << sample.pose.theta;
    // std::cout << ",L: " << likelihood << std::endl;
    return likelihood;
}

void SensorModel::ShowLaser(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan) {
    debug_particles_.clear();
    // std::cout << "St:" << start_pose.utime << ", Et:" << end_pose.utime << std::endl;
    // std::cout << "Sx:" << start_pose.x << ", Ex:" << end_pose.x << std::endl;
    // std::cout << "Sy:" << start_pose.y << ", Ey:" << end_pose.y << std::endl;
    MovingLaserScan movingScan(scan, start_pose, end_pose);     //full scan
    int time_i = 0;
    for(const auto& ray : movingScan) {
        // std::cout << "rt:" << scan.times[time_i] << ", rx:" << ray.origin.x << ", ry:" << ray.origin.y << std::endl;
        time_i ++;
        float range_measure = ray.range;
        float laser_angle =  ray.theta;
        particle_t particle;
        particle.pose.x = ray.origin.x + range_measure * cos(laser_angle);
        particle.pose.y = ray.origin.y + range_measure * sin(laser_angle);
        particle.pose.theta = laser_angle;
        particle.weight = 1.0;
        debug_particles_.emplace_back(particle);
    }
}

void SensorModel::ShowHit(const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, const lidar_t& scan, const OccupancyGrid& map) {
    debug_particles_.clear();
    MovingLaserScan movingScan(scan, start_pose, end_pose);     //full scan
    for(const auto& ray : movingScan) {
        float range_map = get_hit_point(ray.origin.x, ray.origin.y, ray.theta, map);
        float laser_angle =  ray.theta;
        particle_t particle;
        particle.pose.x = ray.origin.x + range_map * cos(laser_angle);
        particle.pose.y = ray.origin.y + range_map * sin(laser_angle);
        particle.pose.theta = laser_angle;
        particle.weight = 1.0;
        debug_particles_.emplace_back(particle);
    }
}

// float SensorModel::probability_sensor_ray(float z_t, float z_star) {
//     // p_hit
//     float p_hit;
//     if((z_t <= z_max_) && (z_t >= 0)) {
//         p_hit = 1.0 / sqrt(2 * M_PI * variance_hit_) * 
//             exp(-0.5 * pow(z_t - z_star, 2) / variance_hit_);
//     }
//     else {
//         p_hit = 0.0;
//     }
//     // p_short
//     float p_short;
//     if((z_t <= z_star) && (z_t >= 0)) {
//         p_short = lambda_short_ * exp(-lambda_short_ * z_t) /
//                 (1.0 - exp(-lambda_short_ * z_star));
//     }
//     else {
//         p_short = 0.0;
//     }
//     // p_0
//     float p_0;
//     if(z_t == 0) {
//         p_0 = 1.0;
//     }
//     else {
//         p_0 = 0.0;
//     }
//     // p_rand
//     float p_rand;
//     if((z_t <= z_max_) && (z_t >= 0)) {
//         p_rand = 1.0 / z_max_;
//     }
//     else {
//         p_rand = 0.0;
//     }

//     return weight_hit_ * p_hit + weight_0_ * p_0 
//          + weight_short_ * p_short + weight_rand_ * p_rand;

// }

float SensorModel::probability_sensor_ray(float z_t, float z_star) {
    // discrete probability
    if(std::abs(z_t - z_star) < 2e-1) {
        return 1.0;
    }
    else if(z_t < z_star) {
        return 0.5;
    }
    else if((z_t - z_star) < 1) {
        return 0.3;
    }
    else {
        return 0.1;
    }
}

float SensorModel::get_hit_point(float x0_m, float y0_m, float theta, const OccupancyGrid& map) {
    /*
    x0_m, y0_m : position of robot in meter
    theta : theta angle for array
    */
    // change theta into (0, 2*pi]
    while(theta <= 0) {
        theta += 2 * M_PI;
    }
    while(theta > 2 * M_PI) {
        theta -= 2 * M_PI;
    }

    Point<int> coord = map.GlobalFrameToCoord(x0_m, y0_m);
    Point<int> boundary_point = map.BoundaryCellForArray(x0_m, y0_m, theta);
    int x0 = coord.x;
    int y0 = coord.y;
    int x1 = boundary_point.x;
    int y1 = boundary_point.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    int8_t cell_odd;
    int e2 = 0;
    while(x != x1 || y != y1) {
        e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
        // check if this is occupied:
        if (std::abs(x) > 10000) {
            return -1;
        }
            
        cell_odd = map(x, y);
        
        if(cell_odd > odds_threshold_) {
            // meet with the obstacle
            Point<float> hit_global_frame = map.CoordToGlobalFrame(x, y);
            return sqrt(pow(hit_global_frame.x - x0_m, 2) + pow(hit_global_frame.y - y0_m, 2));
        }
        else if (cell_odd >= unknown_threshold_) 
        {
            // If this ray has an unknown obstacle, ignore it
            return -1;
        }
    }
    // if not hit after all rounds?
    return -1;  // -1 means no hits.
    
}

bool SensorModel::JudgeMeasurement(double map_likelihood) {
    likelihood_count_ += 1.0;
    likelihood_sum_ += map_likelihood;
    if(likelihood_sum_ / likelihood_count_ > 1.5 * map_likelihood) {
        std::cout << "================" << std::endl;
        std::cout << "Bad Measurements" << std::endl;
        return false;
    }
    else {
        // if good
        likelihood_count_ = 0.0;
        likelihood_sum_= 0.0;
        return true;
    }
}