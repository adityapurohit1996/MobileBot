#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    float log_likelihood = 0;
    int valid_array_count = 0;
    for(int i = 0; i < scan.num_ranges; i++) {
        float range_measure = scan.ranges[i];
        float range_map = get_hit_point(sample.pose.x, sample.pose.y, sample.pose.theta, map);
        if(range_map > 0) {
            log_likelihood += log(probability_sensor_ray(range_measure, range_map));
            valid_array_count++;
        }
    }
    if(valid_array_count <= valid_threshold_) {
        printf("Valid Array Not Enough");
    }
    return exp(log_likelihood / valid_array_count);
}

double SensorModel::probability_sensor_ray(double z_t, double z_star) {
    // p_hit
    float p_hit;
    if((z_t <= z_max_) && (z_t >= 0)) {
        p_hit = 1.0 / sqrt(2 * M_PI * variance_hit_) * 
            exp(-0.5 * pow(z_t - z_star, 2) / variance_hit_);
    }
    else {
        p_hit = 0.0;
    }
    // p_short
    float p_short;
    if((z_t <= z_star) && (z_t >= 0)) {
        p_short = lambda_short_ * exp(-lambda_short_ * z_t) /
                (1.0 - exp(-lambda_short_ * z_star));
    }
    else {
        p_short = 0.0;
    }
    // p_max
    float p_max;
    if(z_t == z_max_) {
        p_max = 1.0;
    }
    else {
        p_max = 0.0;
    }
    // p_rand
    float p_rand;
    if((z_t <= z_max_) && (z_t >= 0)) {
        p_rand = 1.0 / z_max_;
    }
    else {
        p_rand = 0.0;
    }

    return weight_hit_ * p_hit + weight_max_ * p_max 
         + weight_short_ * p_short + weight_rand_ * p_rand;

}

double SensorModel::get_hit_point(float x0_m, float y0_m, float theta, const OccupancyGrid& map) {
    /*
    x0_m, y0_m : position of robot in meter
    theta : theta angle for array
    */
    Point<int> coord = map->GlobalFrameToCoord(x0_m, y0_m);
    Point<int> boundary_point = map->BoundaryCellForArray(x0_m, y0_m, theta);
    int x0 = coord.x;
    int y0 = coord.y;
    int x1 = boundary_point.x;
    int y1 = boundary_point.y;
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    sx = x0<x1 ? 1 : -1;
    sy = y0<y1 ? 1 : -1;
    err = dx - dy;
    x = x0;
    y = y0;
    int8_t cell_odd;
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
        cell_odd = map(x, y);
        if(cell_odd > odds_threshold_) {
            // meet with the obstacle
            Point<float> hit_global_frame = map->CoordToGlobalFrame(x, y);
            return sqrt(pow(hit_global_frame.x - x0_m, 2) + pow(hit_global_frame.y - y0_m, 2));
        }
    }
    // if not hit after all rounds?
    return -1;  // -1 means no hits.
    
}

double RangeInMap(int x, int y, double dx, double theta, const OccupancyGrid& map) {
    // Get the range at (x, y) in direction theta in occupy map
    
}

void NextGrid(int x, int y, double theta, int& nx, int& dy) {

}