#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& start_pose, const pose_xyt_t& end_pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    MovingLaserScan movingScan(scan, start_pose, end_pose)     //full scan

    adjusted_ray_t ray      //single ray

    float cellSize = 0.05;      

    int8_t rayInitial_x, rayInitial_y, rayFinal_x, rayFinal_y, hitCell_x, hitCell_y, dx, dy, sx, sy, err, x, y, e2; 

    for(const auto&ray : movingScan){

        rayInitial_x = ray.origin.x;
        rayInitial_y = ray.origin.y;
        originCell_x = rayInitial_x/cellSize;
        originCell_y = rayInitial_y/cellSize;

        if (ray.range < kMaxLaserDistance){
            rayFinal_x = rayInitial_x + ray.range*cos(ray.theta + end_pose.theta);      //check!
            rayFinal_y = rayInitial_y + ray.range*sin(ray.theta + end_pose.theta);

            hitCell_x = round(rayFinal_x/cellSize);
            hitCell_y = round(rayFinal_y/cellSize);          
            
            if (map.logOdds(hitCell_x, hitCell_y) < 127){
                map.setLogOdds(hitCell_x, hitCell_y, map.logOdds(hitCell_x, hitCell_y) + kHitOdds_);
            }
            else
            {
                map.setLogOdds(hitCell_x, hitCell_y, 127);
            }
            

            // Breshenham's to update as free every cell along the line
            
            dx = hitCell_x - originCell_x;
            dy = hitCell_y - originCell_y;

            sx = originCell_x < hitCell_x ? 1 : -1;
            sy = originCell_y < hitCell_y ? 1 : -1;

            err = dx - dy;
            x = originCell_x;
            y = originCell_y;

            while(x != hitCell_x || y != hitCell_y){
            if (map.logOdds(x, y) > -127){
                map.setLogOdds(x, y, map.logOdds(x, y) - kMissOdds_);
            }
            else
            {
                map.setLogOdds(x, y, -127);
            }
            e2 = 2*err;
            if (e2 >= -dy){
            err -= dy;
            x += sx;
            }
            if (e2 <= dx){
            err += dx
            y += sy
            }

            }

        }

    }

}