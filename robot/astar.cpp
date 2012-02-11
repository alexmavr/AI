/*
 * =====================================================================================
 *
 *       Filename:  astar.cpp
 *
 *    Description:  A* Algorithm for two agents with a common goal node
 *
 *        Version:  1.0
 *        Created:  02/09/2012 06:58:20 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Alex Mavrogiannis (afein), nalfemp@gmail.com
 *   Organization:  National Technical University of Athens 
 *
 * =====================================================================================
 */

/*   Paradoxi: Theorithike pos stin periptosi conflict,  to ena robot kanei stall gia na perasei to allo.
*              Se autin tin periptosi, to robot pou ekane stall to emfanizei sto teliko monopati 
*/

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <cstdio>
#include <cstdlib>
using namespace std;

int n; // max in the x direction  
int m; //  max in the y direction  
int** map;
int** past_nodes; // map of closed nodes
int** frontier; // map of open nodes
int** directions;// map of directions
const int dir=4; // number of directions to go from a position

unsigned int node_counter=0; // Counter for number of nodes opened

/* Possible Directions */
static int dx[dir]={1,  0,  -1,  0 };
static int dy[dir]={0,  1,   0,  -1 };
     /* Up, Down, Left, Right Only */

class node
{
     // current position
     int current_x;
     int current_y;
     // total distance already travelled 
     int g_score;
     // f_score=g_score+ remaining distance heuristic
     int f_score;  // smaller means higher priority

     public:
     node(int x,  int y,  int g,  int f){
          current_x=x;
          current_y=y;
          g_score=g;
          f_score=f;
     }

     int getcurrent_x() const {return current_x;}
     int getcurrent_y() const {return current_y;}
     int getG_score() const {return g_score;}
     int getF_score() const {return f_score;}

     void updatePriority(const int & xDest,  const int & yDest){
          f_score=g_score+heuristic(xDest, yDest)*10; 
     }

     void next(const int & i) // i: direction
     {
          g_score+=10;
     }

     // Heuristic function for the remaining distance to the goal.
     const int & heuristic(const int & xDest,  const int & yDest) const
     {
          static int xd,  yd,  d;
          xd=xDest-current_x;
          yd=yDest-current_y;         

          // Euclidian Distance
          d=static_cast<int>(sqrt(xd*xd+yd*yd));

          // Manhattan distance +1
          //d=abs(xd)+abs(yd)+1;

          return(d);
     }
};

// Determine f_score in priority queue)
bool operator<(const node & a,  const node & b)
{
     return a.getF_score() > b.getF_score();
}

// A*
// The route is a string of direction numbers from the possible directions set.
string A_star( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish ){
     static priority_queue<node> queue[2]; // available nodes in priority queue
     static int index; // priority queue index
     static node* node_n;
     static node* node_m;
     static int i,  j,  x,  y,  xdx,  ydy;
     static char c;
     index=0;

     // reset the node maps
     for(y=0;y<m;y++){
          for(x=0;x<n;x++){
               past_nodes[x][y]=0;
               frontier[x][y]=0;
          }
     }

     // create the start node and push into list of open nodes
     node_n=new node(xStart,  yStart,  0,  0);
     node_n->updatePriority(xFinish,  yFinish);
     queue[index].push(*node_n);
     //frontier[x][y]=node_n->getF_score(); // mark it on the open nodes map
     delete node_n;

     // A* search
     while(!queue[index].empty())
     {
          // get the node with the lowest f_score
          node_n=new node( queue[index].top().getcurrent_x(),  queue[index].top().getcurrent_y(),  
                    queue[index].top().getG_score(),  queue[index].top().getF_score());

          x=node_n->getcurrent_x(); y=node_n->getcurrent_y();

          queue[index].pop(); // remove the node from the open list
          frontier[x][y]=0;
          // mark it on the closed nodes map
          past_nodes[x][y]=1;

          // stop the search when the goal state is reached
          if(x==xFinish && y==yFinish) 
          {
               // generate the path from finish to start
               string path="";
               while(!(x==xStart && y==yStart))
               {
                    j=directions[x][y];
                    c='0'+(j+dir/2)%dir;
                    path=c+path;
                    x+=dx[j];
                    y+=dy[j];
               }

               delete node_n;
               while(!queue[index].empty()) queue[index].pop();           
               return path;
          }

          // create child nodes for all directions
          for(i=0;i<dir;i++)
          {
               xdx=x+dx[i]; ydy=y+dy[i];

               if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
                              || past_nodes[xdx][ydy]==1))
               {
                    // create child
                    node_counter++;       // counter for total number of nodes
                    node_m=new node( xdx,  ydy,  node_n->getG_score(),  
                              node_n->getF_score());
                    node_m->next(i);
                    node_m->updatePriority(xFinish,  yFinish);

                    // add to frontier 
                    if(frontier[xdx][ydy]==0)
                    {
                         frontier[xdx][ydy]=node_m->getF_score();
                         queue[index].push(*node_m);
                         // mark parent direction
                         delete node_m;
                         directions[xdx][ydy]=(i+dir/2)%dir;
                    }
                    else if(frontier[xdx][ydy]>node_m->getF_score())
                    {
                         // update f_score 
                         frontier[xdx][ydy]=node_m->getF_score();
                         // update parent direction 
                         directions[xdx][ydy]=(i+dir/2)%dir;

                         // empty one priority queue to the other one
                         while(!(queue[index].top().getcurrent_x()==xdx && 
                                        queue[index].top().getcurrent_y()==ydy))
                         {                
                              queue[1-index].push(queue[index].top());
                              queue[index].pop();       
                         }
                         queue[index].pop(); // remove the wanted node

                         // exchange priority queues
                         if(queue[index].size()>queue[1-index].size()) index=1-index;
                         while(!queue[index].empty())
                         {                
                              queue[1-index].push(queue[index].top());
                              queue[index].pop();       
                         }
                         index=1-index;
                         queue[index].push(*node_m); // add new node
                         delete node_m;
                    }
                    else delete node_m; 
               }
          }
          delete node_n;
     }
     return ""; // empty string if no route was found
}

int main()
{
     int  x_start[2],  y_start[2],  x_goal,  y_goal;  // two start and one finish location
     char temp;
     srand(time(NULL));

     /* NOTE: All dimensions are reversed in the program */

     /*   Input Constants*/
     scanf("%d %d", &m, &n);
     scanf("%d %d", &y_start[0], &x_start[0]);
     scanf("%d %d", &y_start[1], &x_start[1]);
     scanf("%d %d", &y_goal, &x_goal);

     /*  Allocations and Initialization */
     map = (int**) malloc(n * sizeof(int));
     past_nodes= (int**) malloc(n * sizeof(int));
     frontier= (int**) malloc(n * sizeof(int));
     directions= (int**) malloc(n * sizeof(int));
     for (int i=0;i<n;i++){
               map[i] = (int*) malloc(m * sizeof(int));
               past_nodes[i] = (int*) malloc(m * sizeof(int));
               frontier[i] = (int*) malloc(m * sizeof(int));
               directions[i] = (int*) malloc(m * sizeof(int));
               for (int j=0;j<m;j++){
                    past_nodes[i][j]=0;
                    frontier[i][j]=0;
                    directions[i][j]=0;
               }
     }
     /* Map Input */
     for(int x=0;x<n;x++) {
          for(int y=0;y<m;y++){
               cin >> temp;
               if (temp =='O'){
                    map[x][y]=0;
               } else if ( temp == 'X'){
                    map[x][y]=1;
               } else{
                    printf("Error! Invalid Input File\n");
                    return 1;
               }
          }
     }
     cout<<"Total Size (X, Y): "<<m<<" x "<<n<<endl;
     cout<<"Robot 1 Start: "<<y_start[0]<<", "<<x_start[0]<<endl;
     cout<<"Robot 2 Start: "<<y_start[1]<<", "<<x_start[1]<<endl;
     cout<<"Meeting Point: "<<y_goal<<", "<<x_goal<<endl;

     /* Correction for 0-indexed arrays */
     x_start[0]--; y_start[0]--;
     x_start[1]--; y_start[1]--;
     x_goal--; y_goal--;

     /* Find the Routes Individually */
     string route1=A_star(x_start[0], y_start[0],  x_goal,  y_goal);
     if(route1=="") cout<<"Empty route for robot 1 !"<<endl;
     string route2=A_star(x_start[1], y_start[1],  x_goal,  y_goal);
     if(route2=="") cout<<"Empty route for robot 2 !"<<endl;

     cout<<"Route 1:"<<endl;
     cout<<route1<<endl<<endl;
     cout<<"Route 2:"<<endl;
     cout<<route2<<endl<<endl;

     // Follow the route and display 
     if ((route1.length()>0) && (route2.length()>0))
     {
          /*   list of locations for cross-checking for conflicts */
          char c;
          int* x= (int*) malloc((route1.length()+1)*sizeof(int));
          x[0]=x_start[0];
          int* y= (int*) malloc((route1.length()+1)*sizeof(int));
          y[0]=y_start[0];
          int* x2=(int*) malloc((route2.length()+1)*sizeof(int));
          x2[0]=x_start[1];
          int* y2=(int*) malloc((route2.length()+1)*sizeof(int));
          y2[0]=y_start[1];
          map[x[0]][y[0]]=2;        // Initial points
          map[x2[0]][y2[0]]=2;

          for(int i=0;i<(int)route1.length();i++)
          {
               c =route1.at(i);
               x[i+1]=x[i]+dx[atoi(&c)];
               y[i+1]=y[i]+dy[atoi(&c)];
               map[x[i+1]][y[i+1]]=3;        // Mark Distance on Map for robot 1
          }
          int max= route1.length();
          map[x[max]][y[max]]=4;             // Add goal point
          /* Mark on map for robot 2 as well */
          for(int i=0;i<(int)route2.length();i++)
          {
               c =route2.at(i);
               x2[i+1]=x2[i]+dx[atoi(&c)];
               y2[i+1]=y2[i]+dy[atoi(&c)];
               if (map[x2[i+1]][y2[i+1]]==3){     // if already checked, add overlapping symbol
                   map[x2[i+1]][y2[i+1]]=5;
               }else if (map[x2[i+1]][y2[i+1]]!=4){
                    map[x2[i+1]][y2[i+1]]=3;        
               }
          }
          for (int i=0;i<(int) min(route1.length(),route2.length());i++){ 
               if ((x[i]==x2[i]) && (y[i]==y2[i])){         // Conflict Resolution
                    printf("Robot 2 Needs to Stall before Move %d\n", i);
               }
          }
          free(x); free(x2); free(y); free(y2);
          
          cout << "\n--------------------\
                   \n    @: Obstacle         \
                   \n    S: Start            \
                   \n    .: Single Path      \
                   \n    *: Mutual Path      \
                   \n    F: Finish           \
                   \n--------------------\n\n";

          // display the map and the routes
          for(int x=0;x<n;x++){
               for(int y=0;y<m;y++){
                    if(map[x][y]==0)
                         cout<<" "; //empty 
                    else if(map[x][y]==1)
                         cout<<"@"; //obstacle
                    else if(map[x][y]==2)
                         cout<<"S"; //start
                    else if(map[x][y]==3)
                         cout<<"."; //route
                    else if(map[x][y]==4)
                         cout<<"F"; //finish
                    else if(map[x][y]==5)
                         cout<<"*"; //overlap
               }
               cout<<endl;
          }
     }
     cout << endl <<"Total Nodes Expanded: "<< node_counter<< endl;
     return(0);
}
