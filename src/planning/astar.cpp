#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <array>

#define FLT_MAX 10000


using namespace std;
struct Node
{
     int y;
    int x;
    int parentX;
    int parentY;
     float hCost;
};
inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.hCost < rhs.hCost;
}

static bool isValid(int x,int y, ObstacleDistanceGrid* distances) 
{ //If our Node is an obstacle it is not valid

    long int Y_MAX = distances->heightInCells;
    long int X_MAX = distances->widthInCells;
        int id = x + y * X_MAX; 
        if (distances->(x,y)== 0) 
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

static bool isDestination(pose_xyt_t pose, pose_xyt_t dest) 
{
    if (pose.x == dest.x && pose.y == dest.y) 
    {
        return true;
    }
    return false;
}

static bool (pose_xyt_t pose, pose_xyt_t)
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    long int X_MAX = distances.widthInCells;
    long int Y_MAX = distances.heightInCells;


    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    if (isValid(goal,&distance) == false) 
    {
        cout << "Destination is an obstacle" << endl;
        //Destination is invalid
    }
    else if(isDestination(start,goal))
    {
        cout<<"You are at destination"<<endl;
    }
    else
    {
        bool closedList[X_MAX][Y_MAX];
        array<array < Node, Y_MAX>, X_MAX> allMap;
        for (int x = 0; x < X_MAX; x++) {
            for (int y = 0; y < Y_MAX; y++) {
                allMap[x][y].hCost = FLT_MAX;
                allMap[x][y].parent.x = -1;
                allMap[x][y].parent.y = -1;
                allMap[x][y].pose.x = x;
                allMap[x][y].pose.y = y;

                closedList[x][y] = false;
            }
        }

        //Initialize our starting list
        int x = start.x;
        int y = start.y;
        allMap[x][y].hCost = 0.0;
        allMap[x][y].parent.x = x;
        allMap[x][y].parent.y = y;

        vector<Node> openList;  
        openList.emplace_back(allMap[x][y]);
        bool destinationFound = false;

        while (!openList.empty()&&openList.size()<X_MAX*Y_MAX) 
        {
            Node node;
            do 
            {
                //This do-while loop could be replaced with extracting the first
                //element from a set, but you'd have to make the openList a set.
                //To be completely honest, I don't remember the reason why I do
                //it with a vector, but for now it's still an option, although
                //not as good as a set performance wise.
                float temp = FLT_MAX;
                vector<Node>::iterator itNode;
                for (vector<Node>::iterator it = openList.begin();it != openList.end(); it = next(it)) 
                    {
                    Node n = *it;
                    if (n.hCost < temp) {
                        temp = n.hCost;
                        itNode = it;
                    }
                }
                node = *itNode;
                openList.erase(itNode);
            } while (isValid(node.x, node.y) == false);

            x = node.x;
            y = node.y;
            closedList[x][y] = true;

            //For each neighbour starting from North-West to South-East
            for (int newX = -1; newX <= 1; newX++) {
                for (int newY = -1; newY <= 1; newY++) {
                    double gNew, hNew, fNew;
                    if (isValid(x + newX, y + newY)) {
                        if (isDestination(x + newX, y + newY, dest))
                        {
                            //Destination found - make path
                            allMap[x + newX][y + newY].parentX = x;
                            allMap[x + newX][y + newY].parentY = y;
                            destinationFound = true;
                            return makePath(allMap, dest);
                        }
                        else if (closedList[x + newX][y + newY] == false)
                        {
                            gNew = node.gCost + 1.0;
                            hNew = calculateH(x + newX, y + newY, dest);
                            fNew = gNew + hNew;
                            // Check if this path is better than the one already present
                            if (allMap[x + newX][y + newY].fCost == FLT_MAX ||
                                allMap[x + newX][y + newY].fCost > fNew)
                            {
                                // Update the details of this neighbour node
                                allMap[x + newX][y + newY].fCost = fNew;
                                allMap[x + newX][y + newY].gCost = gNew;
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
            if (destinationFound == false) 
            {
                cout << "Destination not found" << endl;
        }
    }

    }

    
    return path;
}
