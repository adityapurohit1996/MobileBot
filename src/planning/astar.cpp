#include "astar.hpp"
#include <planning/obstacle_distance_grid.hpp>
#include <array>
#include <cmath>
#include<common/grid_utils.hpp>

#define FLT_MAX 10000
#define MAXIMUM_X 1000
#define MAXIMUM_Y 1000
#define MIN_DISTANCE 0.15


using namespace std;
struct Node
{
     int y;
    int x;
    int parentX;
    int parentY;
     float hCost;
};


bool isValid(int x,int y, float distances_x_y,int X_MAX, int Y_MAX, double minDistanceToObstacle) 
{ //If our Node is an obstacle it is not valid
       // cout<<"goal distance"<<distances_x_y<<endl;
        if (distances_x_y < 1.2*minDistanceToObstacle) 
        {
            return false;
        }
        else
        {
            if (x < 0 || y < 0 || x >= X_MAX || y >= Y_MAX) 
            {
                return false;
            }
            else
            {
                return true;
            }
            
        }        
}

bool isDestination(int x, int y, pose_xyt_t goal_cell) 
{
    if (x == goal_cell.x && y == goal_cell.y) 
    {
        return true;
    }
    return false;
}

double calculateH(int x, int y, pose_xyt_t goal_cell) //Simole heuristics I am not adding information of closeness to obstacle in the heurisitics
{
        double H = (sqrt((x - goal_cell.x)*(x - goal_cell.x)
            + (y - goal_cell.y)*(y - goal_cell.y)));
        return H;
}
/*
//Function directly implemented in the main function because of some parameter passing issues.
robot_path_t makePath(Node map[][MAXIMUM_Y], pose_xyt_t goal_cell,pose_xyt_t goal, ObstacleDistanceGrid& distances) 
{
            int x = goal_cell.x;
            int y = goal_cell.y;
            pose_xyt_t pose;
            Node current;
            Node previous;
            robot_path_t solution;
            pose.x = goal.x;
            pose.y = goal.y;
            pose.theta = goal.theta;
            solution.path.insert(solution.path.begin(),pose); // insert goal first
            while (map[x][y].parentX != map[x][y].x && map[x][y].parentY != map[x][y].y)
                
            {
                current = map[x][y];
                int tempX = map[x][y].parentX;
                int tempY = map[x][y].parentY;
                x = tempX;
                y = tempY;
                previous = map[x][y];
                if(current.x!=previous.x && current.y!=previous.y) // excluding start point
                { 
                    Point<double> temp;
                    temp = grid_position_to_global_position(Point<double>(previous.x,previous.y),distances);
                    pose.x = temp.x;
                    pose.y = temp.y;
                    //cout<<current.y<<" "<<current.x<<" "<<previous.y<<" "<<previous.x<<endl;
                    pose.theta = atan((current.y-previous.y)/(current.x-previous.x));
                    solution.path.insert(solution.path.begin(),pose);
                }

            }
            
            return solution;
}
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    robot_path_t path; // Variable with all poses of the path
    
    unsigned int X_MAX = distances.widthInCells();
    unsigned int Y_MAX = distances.heightInCells();

    //conversion from global coordinates to cell coordinates
    pose_xyt_t start_cell, goal_cell;
    Point<int> temp;
    temp = global_position_to_grid_cell(Point<float>(start.x,start.y),distances);
    start_cell.x = temp.x;
    start_cell.y = temp.y;
    temp = global_position_to_grid_cell(Point<float>(goal.x,goal.y),distances);
    goal_cell.x = temp.x;
    goal_cell.y = temp.y;

    //////////////// Sanity checks //////////////////////////
    
    if (isValid(goal_cell.x,goal_cell.y,distances(goal_cell.x,goal_cell.y),X_MAX,Y_MAX,params.minDistanceToObstacle) == false) 
    {
        cout << "Destination is an obstacle" << endl;
        //Destination is invalid
    }
    else if(isDestination(start_cell.x,start_cell.y,goal_cell))
    {
        cout<<"You are at destination"<<endl;
    }
    else
    {
        //////////// start A_star //////////////////
        bool closedList[X_MAX][Y_MAX];
        Node allMap[X_MAX][MAXIMUM_Y]; 
        
        ///////////// initialize everything to max value first //////////////////
        for (int x = 0; x < X_MAX; x++) {
            for (int y = 0; y < Y_MAX; y++) {
                
                allMap[x][y].hCost = FLT_MAX;
                allMap[x][y].parentX = -1;
                allMap[x][y].parentY = -1;
                allMap[x][y].x = x;
                allMap[x][y].y = y;
                closedList[x][y] = false;
            }
        }
        
        //Initialize our start list
        int x = start_cell.x;
        int y = start_cell.y;
        allMap[x][y].hCost = 0.0;
        allMap[x][y].parentX = x;
        allMap[x][y].parentY = y;

        vector<Node> openList;  
        openList.emplace_back(allMap[x][y]);
        bool destinationFound = false;
        cout<<openList.size()<<endl;

        
        while (!openList.empty()&&openList.size()<X_MAX*Y_MAX) 
        {
            if(destinationFound)break; 
            Node node;
            //// Insert start node to open list //////
            node = *openList.begin();
            openList.erase(openList.begin());

            x = node.x;
            y = node.y;
            closedList[x][y] = true;
            //For each neighbour starting from North-West to South-East
            for (int newX = -1; newX <= 1; newX++) 
            {
                if(destinationFound)break;

                for (int newY = -1; newY <= 1; newY++) 
                {
                    if(destinationFound)break;

                    double gNew, hNew, fNew;
                    if(x+newX>=0 && y+newY>=0)
                    { 
                        if (isValid(x + newX, y + newY,distances(x + newX, y + newY), X_MAX, Y_MAX, params.minDistanceToObstacle)) 
                        {
                            if (isDestination(x + newX, y + newY, goal_cell))
                            {
                                //Destination found - make path
                                allMap[x + newX][y + newY].parentX = x;
                                allMap[x + newX][y + newY].parentY = y;
                                destinationFound = true;
                                //cout<<"Found path"<<endl;

                                int x_g = goal_cell.x;
                                int y_g = goal_cell.y;
                                pose_xyt_t pose;
                                Node current;
                                Node previous;
                                pose.x = goal.x;
                                pose.y = goal.y;
                                pose.theta = goal.theta;
                                path.path.insert(path.path.begin(),pose); // insert goal first
                                while (allMap[x_g][y].parentX != allMap[x_g][y_g].x && allMap[x_g][y_g].parentY != allMap[x_g][y_g].y)
                                    
                                {
                                    current = allMap[x_g][y_g];
                                    int tempX = allMap[x_g][y_g].parentX;
                                    int tempY = allMap[x_g][y_g].parentY;
                                    x_g = tempX;
                                    y_g = tempY;
                                    previous = allMap[x_g][y_g];
                                    if(current.x!=previous.x && current.y!=previous.y) // excluding start point
                                    { 
                                        Point<double> temp;
                                        temp = grid_position_to_global_position(Point<double>(previous.x,previous.y),distances);
                                        pose.x = temp.x;
                                        pose.y = temp.y;
                                        //cout<<current.y<<" "<<current.x<<" "<<previous.y<<" "<<previous.x<<endl;
                                        pose.theta = atan((current.y-previous.y)/(current.x-previous.x));
                                        path.path.insert(path.path.begin(),pose);
                                    }

                                }
                            }
                            else if (closedList[x + newX][y + newY] == false)
                            {
                                //gNew = node.gCost + 1.0;
                                hNew = calculateH(x + newX, y + newY, goal_cell);
                                // Check if this path is better than the one already present
                                if (allMap[x + newX][y + newY].hCost == FLT_MAX ||
                                    allMap[x + newX][y + newY].hCost > hNew)
                                {
                                    // Update the details of this neighbour node
                                    
                                    allMap[x + newX][y + newY].hCost = hNew;
                                    allMap[x + newX][y + newY].parentX = x;
                                    allMap[x + newX][y + newY].parentY = y;
                                    openList.emplace_back(allMap[x + newX][y + newY]);
                                }
                            }
                        }
                    }
                    
                }
                
            }
            
        }
            if (destinationFound == false) 
            {
                cout << "Destination not found" << endl;
            }
            
    }
    
    path.utime = start.utime;
    path.path.insert(path.path.begin(),start);    
    path.path_length = path.path.size();
    
    return path;
}
