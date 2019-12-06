#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <array>
#include <cmath>

#define FLT_MAX 10000
#define MAXIMUM_X 1000
#define MAXIMUM_Y 1000


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

static bool isValid(int x,int y, float distances_x_y,int X_MAX, int Y_MAX) 
{ //If our Node is an obstacle it is not valid
        if ((int)(distances_x_y) == 0) 
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

static bool isDestination(int x, int y, pose_xyt_t goal) 
{
    if (x == goal.x && y == goal.y) 
    {
        return true;
    }
    return false;
}

static double calculateH(int x, int y, pose_xyt_t goal) 
{
        double H = (sqrt((x - goal.x)*(x - goal.x)
            + (y - goal.y)*(y - goal.y)));
        return H;
}

static robot_path_t makePath(array<array<Node, MAXIMUM_Y>, MAXIMUM_X> map, pose_xyt_t goal) 
{
            cout << "Found a path" << endl;
            int x = goal.x;
            int y = goal.y;
            pose_xyt_t pose;
            Node current;
            Node previous;
            robot_path_t solution;
            pose.x = x;
            pose.y = y;
            pose.theta = goal.theta;
            solution.path.insert(solution.path.begin(),pose); // insert goal first
            while (!(map[x][y].parentX == x && map[x][y].parentY == y)
                && map[x][y].x != -1 && map[x][y].y != -1) 
            {
                current = map[x][y];
                int tempX = map[x][y].parentX;
                int tempY = map[x][y].parentY;
                x = tempX;
                y = tempY;
                previous = map[x][y]; // this is goal at start of the loop
                pose.x = previous.x;
                pose.y = previous.y;
                pose.theta = atan((current.y-previous.y)/(current.x-previous.x));
                solution.path.insert(solution.path.begin(),pose);

            }
            
            return solution;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    cout<<"Finding path by A_star"<<endl;
    const int X_MAX = distances.widthInCells();
    const int Y_MAX = distances.heightInCells();


    robot_path_t path;
    
    if (isValid(goal.x,goal.y,distances(goal.x,goal.y),X_MAX,Y_MAX) == false) 
    {
        cout << "Destination is an obstacle" << endl;
        //Destination is invalid
    }
    else if(isDestination(start.x,start.y,goal))
    {
        cout<<"You are at destination"<<endl;
    }
    else
    {
        bool closedList[X_MAX][Y_MAX];
        array<array < Node, MAXIMUM_Y>, MAXIMUM_X> allMap; //used because c++ does not like variables in template declaration
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

        //Initialize our starting list
        int x = start.x;
        int y = start.y;
        allMap[x][y].hCost = 0.0;
        allMap[x][y].parentX = x;
        allMap[x][y].parentY = y;

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
            } while (isValid(node.x, node.y,distances(node.x,node.y),X_MAX,Y_MAX) == false);

            x = node.x;
            y = node.y;
            closedList[x][y] = true;

            //For each neighbour starting from North-West to South-East
            for (int newX = -1; newX <= 1; newX++) 
            {
                for (int newY = -1; newY <= 1; newY++) 
                {
                    double gNew, hNew, fNew;
                    if (isValid(x + newX, y + newY,distances(x + newX, y + newY),X_MAX,Y_MAX)) {
                        if (isDestination(x + newX, y + newY, goal))
                        {
                            //Destination found - make path
                            allMap[x + newX][y + newY].parentX = x;
                            allMap[x + newX][y + newY].parentY = y;
                            destinationFound = true;
                            path =  makePath(allMap, goal);
                        }
                        else if (closedList[x + newX][y + newY] == false)
                        {
                            //gNew = node.gCost + 1.0;
                            hNew = calculateH(x + newX, y + newY, goal);
                            //fNew = gNew + hNew;
                            // Check if this path is better than the one already present
                            if (allMap[x + newX][y + newY].hCost == FLT_MAX ||
                                allMap[x + newX][y + newY].hCost > hNew)
                            {
                                // Update the details of this neighbour node
                                //allMap[x + newX][y + newY].fCost = fNew;
                                //allMap[x + newX][y + newY].gCost = gNew;
                                allMap[x + newX][y + newY].hCost = hNew;
                                allMap[x + newX][y + newY].parentX = x;
                                allMap[x + newX][y + newY].parentY = y;
                                openList.emplace_back(allMap[x + newX][y + newY]);
                            }
                        }
                    }
                }
                if(destinationFound)break;
            }
            if(destinationFound)break;
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
