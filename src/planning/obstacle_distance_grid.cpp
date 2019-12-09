#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <math.h>
#include <queue> 

using namespace std; 

struct cell_xy{
    int x;
    int y;
    cell_xy(int a_x, int a_y) : x(a_x), y(a_y) {}
};

// //create queue
// queue<cell_xy> L;


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    cout << "entered function" << endl;
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    
    // vector<cell_xy> obstacleVector;      //dynamic vector for obstacle cell coords

    int cells2Update = (width_ )*(height_);
    // int counter = 0;
 
    // looping through entire grid to find obstacles
    for(int y = 0; y < height_; ++y){

        for (int x = 0; x < width_; ++x){
            
            // mark the obstacles as 1
            if (map.logOdds(x,y) > 0) {
                operator()(x, y) = 1;
                cells2Update--;
            }
            // mark the borders as 1
            else if((x==0) || (x == width_ - 1) || (y==0) || (y == height_ - 1) ){
                operator()(x, y) = 1;
                cells2Update--;
            }
           // mark the rest as free
            else {
                operator()(x, y) = 0;
            }

        }
    }
       
        int curr_distance = 1;

        // updating the map
        while (cells2Update > 0){
            for(int cell_y = 0; cell_y < height_; ++cell_y)
            {
                for (int cell_x = 0; cell_x < width_; ++cell_x)
                {
                    if (distance(cell_x, cell_y) == curr_distance)
                    {
                        //eight-point connectivity
                        if ((cell_x - 1 >= 0) && (distance(cell_x - 1, cell_y) == 0)){
                            cells_[cellIndex(cell_x-1, cell_y)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_x - 1 >= 0) && (cell_y + 1 <= height_ - 1) && (distance(cell_x-1, cell_y+1) == 0)){
                            cells_[cellIndex(cell_x-1, cell_y+1)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_x - 1 >= 0) && (cell_y - 1 >= 0) && (distance(cell_x - 1, cell_y - 1) == 0)){
                            cells_[cellIndex(cell_x - 1, cell_y - 1)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_y + 1 <= height_ - 1) && (distance(cell_x, cell_y + 1) == 0)){
                            cells_[cellIndex(cell_x, cell_y)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_y - 1 >= 0) && (distance(cell_x, cell_y - 1) == 0)){
                            cells_[cellIndex(cell_x, cell_y - 1)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_x + 1 <= width_ - 1) && (distance(cell_x + 1, cell_y) == 0)){
                            cells_[cellIndex(cell_x + 1, cell_y)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_x + 1 <= width_ - 1) && (cell_y + 1 <= height_ - 1) && (distance(cell_x + 1, cell_y + 1) == 0)){
                            cells_[cellIndex(cell_x + 1, cell_y + 1)] = curr_distance + 1;
                            cells2Update--;
                        }
                        if ((cell_x + 1 <= width_ - 1) && (cell_y - 1 >= 0) && (distance(cell_x + 1, cell_y - 1) == 0)){
                            cells_[cellIndex(cell_x + 1, cell_y - 1)] = curr_distance + 1;
                            cells2Update--;
                        }

                    }
                   // cout << cell_x<< cell_y << endl;
                }
            }

            curr_distance+=1;

        }

        for(int grid_y = 0; grid_y < height_; ++grid_y)
        {
            for (int grid_x = 0; grid_x < width_; ++grid_x)
            {
                cells_[cellIndex(grid_x, grid_y)] = (distance(grid_x, grid_y) - 1)*metersPerCell_;
            }
        }
        cout<<"exiting obstacle distance grid"<<endl;
}



bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
