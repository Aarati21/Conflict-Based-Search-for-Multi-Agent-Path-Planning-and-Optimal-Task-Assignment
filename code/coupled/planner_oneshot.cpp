
/*=================================================================
 *
 * planner_oneshot.cpp
 *
 *=================================================================*/

/* Include Required Header Files */
#include <iostream> 
#include <math.h>
#include <mex.h>
#include <cstdio>
#include <time.h>  

#include <algorithm>
#include <limits>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "lowlevel.h"

#include "param.h"
#include "taskassignment.h"


using namespace std;

/* Input Arguments */
#define	NUM_OF_AGENTS           prhs[0]
#define	NUM_OF_PICKUP           prhs[1]
#define	MAP_DIM                 prhs[2]
#define	COLLISION_THRESH        prhs[3]
#define	ROBOT_POS               prhs[4]
#define	PICKUP_POS              prhs[5]
#define MAP_IN                  prhs[6]
#define	CURR_TIME               prhs[7]
#define	DELIVERY_POS            prhs[8]
#define	NUM_OF_DELIVERY         prhs[9]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]
#define	ASSIGN_PICKUP           plhs[1]
#define	ASSIGN_DELIVERY         plhs[2]

/* access to the map is shifted to account for 0-based indexing in the map,
whereas 1-based indexing in matlab (so, robotpose and pickuppose are 1-indexed) */
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

/* Define MAX and MIN as a function */
#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

// Conflict Search Node
class Node {
private:
    
    double cost;
    bool root;
    vector <tuple<int, Point, int>> constraints;
    vector <Path> solutions;
    vector <Path> solution_p;
    vector <Path> solution_d;
    double* assignment;
    vector<int> assignmentVect;
    double* assignment_d;
    vector<int> assignmentVect_d;

public:  
    

    void set_cost(double c) { cost = c; }
    double get_cost() { return cost; }

    void set_assignmentvect(vector<int> a) { assignmentVect = a; }
    vector<int> get_assignmentvect() { return assignmentVect;}
    
    void set_assignmentvect_d(vector<int> a) { assignmentVect_d = a; }
    vector<int> get_assignmentvect_d() { return assignmentVect_d;}

    void set_root(bool r) { root = r; }
    bool get_root() { return root; }

    void push_constraints(int agent, Point index, int time) { constraints.push_back(make_tuple(agent, index, time)); }
    void set_constraints(vector<tuple<int, Point, int>> con) { constraints = con; }
    vector<tuple<int, Point, int>> get_constraints() { return constraints; }

    void set_assignment(double* a) { assignment = a; }
    double* get_assignment() { return assignment; }
    
    void set_assignment_d(double* a) { assignment_d = a; }
    double* get_assignment_d() { return assignment_d; }

    void set_solution(vector<Path> s) { solutions = s; }
    vector<Path> get_solution() { return solutions; }
    
    void set_solution_p(vector<Path> s) { solution_p = s; }
    vector<Path> get_solution_p() { return solution_p; }
    
    void set_solution_d(vector<Path> s) { solution_d = s; }
    vector<Path> get_solution_d() { return solution_d; }

};

// Define final_node as a global variable to store the final solution
static Node final_node;

// Define the comparator function to compare two CBS nodes
struct min_heap {
    bool operator()(Node p1, Node p2)
    {
        return p1.get_cost() > p2.get_cost();
    }
};

// Function to check if there is a conflict in the current search iteration
bool check_conflict(Node node, int numofagents, tuple<int,  Point, int> &conflict1, tuple<int,  Point, int>& conflict2) {
    // Get node's solution 
    vector<Path> sol = node.get_solution();
    for (int i = 0; i < numofagents; i++)
    {
        for (int j = i+1; j < numofagents ; j++) {
            int m, m1, m2;
            vector<Point> agent1 = sol[i].pathVect;
            vector<Point> agent2 = sol[j].pathVect;
            m1 = agent1.size();
            m2 = agent2.size();
            m = min(m1,m2);
            for (int k = 0; k < m; k++) 
            {                  
                // Point Collision
                if (agent1[k] == agent2[k]) 
                {   
                    conflict1 = make_tuple(i, agent1[k], k);
                    conflict2 = make_tuple(j, agent1[k], k);
                    return 0;
                }
                if (k > 0)
                {
                    // Edge Collision
                    if ((agent1[k].x_pos - agent2[k].x_pos)==0 && (agent1[k].y_pos - agent2[k].y_pos) == 0 && (agent1[k - 1].x_pos - agent2[k - 1].x_pos) == 0 && (agent1[k - 1].y_pos - agent2[k - 1].y_pos) == 0)
                    {
                        conflict1 = make_tuple(i, agent1[k], k);
                        conflict2 = make_tuple(j, agent2[k], k);
                        return 0;
                    }
                    // Diagonal Collision
                    if (((agent1[k].x_pos + agent1[k-1].x_pos) / 2 == (agent2[k].x_pos + agent2[k - 1].x_pos) / 2 ) &&
                        ((agent1[k].y_pos + agent1[k - 1].y_pos) / 2 == (agent2[k].y_pos + agent2[k - 1].y_pos) / 2))
                    {
                        conflict1 = make_tuple(i, agent1[k], k);
                        conflict2 = make_tuple(j, agent2[k], k);
                        return 0;
                        
                    }
                }
            }
        }
    }
    return 1;
}

// Function to calculate the cost of the solution
double get_SIC(Node node, int numofagents) {
    double cost = 0;
    vector<Path> solutions = node.get_solution();
    for (int i = 0; i < numofagents; i++)
    {
        cost += solutions[i].cost;
    }
    return cost;
}

// Function to convert grid of State_map class to a cost matrix
vector<vector<double>> gridmap_to_costmatrix(int numofagents, int numofpickup,vector<vector<State_map> > gridmap, vector<Point> robotPosns) 
{
    vector<vector<double> > temp;
    for(int i=0; i<numofagents; i++)
    {
        vector<double> tempTemp;
        for(int j=0; j<numofpickup;j++)
        {
            tempTemp.push_back(gridmap[robotPosns[i].y_pos - 1][robotPosns[i].x_pos - 1].getH()[j]);  
        }   
        temp.push_back(tempTemp);
    }
    return temp;
}

// Function to convert a double array to vector of Point
vector<Point> double2pointvector(double* pos, int numofagents) {
    vector<Point> pos_vector;
    for (int i = 0; i < numofagents; i++) {
        Point start(pos[i], pos[i + numofagents]); // in terms of x and y positions
        pos_vector.push_back(start);
    }
    return pos_vector;
}

// Print the paths
void print_solutions(Node start_node, int numofagents) 
{    
    for (int i = 0; i < numofagents; i++) 
    {
        printf("\n Agent  %d:   ", i);
        vector<Path> paths = (start_node.get_solution());
        vector<Point> each_path = paths[i].pathVect;
        for (int j = 0; j < each_path.size(); j++)
            printf("(%d, %d),   ", each_path[j].x_pos, each_path[j].y_pos);
    }
}

// Check if two vectors are equal
int equal_vectors(vector<int> pass, vector<int> newvec) 
{
    for (int i = 0; i < pass.size(); i++) 
    {
        if (pass[i] != newvec[i]) 
        {
            printf("unequal %d %d", pass[i], newvec[i]);
            return 0;
        }
    }
    return 1;
}

// Print a double array
void print_dble(double* pickuppos_new, int numofagents) 
{
    for (int i = 0; i < numofagents; i++)
        printf("Agent pickup positions \n %f %f \n", pickuppos_new[i], pickuppos_new[i + numofagents]);
}

// Print vector of Point class
void print_vector_of_Points(vector<Point> pickup_new_tree, int numofagents) 
{
    for (int i = 0; i < numofagents; i++)
        printf("Agent pickup  positions as vector of Point class variable %d %d \n", pickup_new_tree[i].x_pos, pickup_new_tree[i].y_pos);
}

// Print the constraint tuple
void print_constraint(vector<tuple<int, Point, int>> con, int numofagents) 
{
    for (int i = 0; i < con.size(); i++) 
    {
        tuple<int, Point, int> c = con[i];
        printf(" \nConstraint for child node agent %d, Point(%d %d), time %d) \n", get<0>(c), get<1>(c).x_pos, get<1>(c).y_pos, get<2>(c));
    }
}

// Lengthen all the solutions to be equal
void lengthen_solution(vector<Path> &y, int numofagents) 
{
    //if (y.size() >= 1)
    //{
        int j = 0;
        for (int i = 0; i < numofagents; i++)
            if (y[i].pathVect.size() > j)
                j = y[i].pathVect.size();

        for (int i = 0; i < numofagents; i++) 
        {
            int m = y[i].pathVect.size();
            //if (m > 1)
            //{
                Point last = y[i].pathVect[m - 1];
                while (y[i].pathVect.size() < j)
                    y[i].pathVect.push_back(last);
            //}
        }
    // }
}

int delivery_reached = 0;

// Main planner code
static void planner(
        int numofagents,
        int numofpickup,
        int numofdelivery,
        int x_size,
        int y_size,
        int collision_thresh,
        double* robotpos,
        double* pickuppos,
        double* deliverypos,
        double*	map,
        int curr_time,
        double* action_ptr,
        double* assign_pickup,
        double* assign_delivery
        )
{   
    // Flag to see if end goal is reached   
    bool time_exceeded = false;
    // Clock variable to keep track of time taken by planner
    clock_t start_time = clock();
    
    // Plan only if current time is zero - oneshot planning
    if (curr_time == 0) 
    {
        cout << endl;
    
        // Define priority queues for the search tree and assignment tree
        priority_queue<Node, vector<Node>, min_heap> OPEN;
        priority_queue<ASG, vector<ASG>, ASG_Comparator> ASG_OPEN;

        //  Defines start, pickup and delivery position - convert to vector of Point class
        vector<Point> starts = double2pointvector(robotpos, numofagents);
        vector<Point> pickup = double2pointvector(pickuppos, numofpickup);
        vector<Point> delivery = double2pointvector(deliverypos, numofdelivery);
        
        // Vector of assignments 
        vector<vector<int>> PAST_ASSIGNMENTS_PICKUP;
        vector<vector<int>> PAST_ASSIGNMENTS_DELIVERY;
        
        //  Define gridmap for heuristic calculation for Dijkstra expansions and cost matrix - pickup
        State_map state_init_map_pickup(numofpickup);
        vector<vector<State_map> > gridmap_pickup(y_size, vector<State_map>(x_size, state_init_map_pickup));
        backDijkstra(gridmap_pickup, pickup, map, x_size, y_size, collision_thresh);
        
        //  Define gridmap for heuristic calculation for Dijkstra expansions and cost matrix - delivery
        State_map state_init_map_delivery(numofdelivery);
        vector<vector<State_map> > gridmap_delivery(y_size, vector<State_map>(x_size, state_init_map_delivery));
        backDijkstra(gridmap_delivery, delivery, map, x_size, y_size, collision_thresh);

        // Convert the gridmap to cost_matrix to feed the assignment function
        vector<vector<double>> cost_matrix_pickup = gridmap_to_costmatrix(numofagents, numofpickup, gridmap_pickup, starts);
        vector<vector<double>> cost_matrix_delivery = gridmap_to_costmatrix(numofpickup, numofdelivery, gridmap_delivery, pickup);
        
        // Define a start node for CBS search
        Node start_node;
        start_node.set_root(1);
        
        // Call first assignment which returns sorted pickup positions and delivery positions
        vector<int> assignmentVectpickup;
        vector<int> assignmentVectdelivery;
        double* pickuppos_new;
        double* deliverypos_new;
        first_assignment(robotpos, pickuppos, deliverypos, cost_matrix_pickup, cost_matrix_delivery, 
                assignmentVectpickup, assignmentVectdelivery, pickuppos_new, deliverypos_new, ASG_OPEN);
        PAST_ASSIGNMENTS_PICKUP.push_back(assignmentVectpickup);
        PAST_ASSIGNMENTS_DELIVERY.push_back(assignmentVectdelivery);
        
        // Set the assignment attributes of CBS start search node
        start_node.set_assignment(pickuppos_new);
        start_node.set_assignment_d(deliverypos_new);
        start_node.set_assignmentvect(assignmentVectpickup);
        start_node.set_assignmentvect_d(assignmentVectdelivery);
        
        // Call the unconstrained search function to get the unconstrained solution
        vector<Path> s1 = unconstrainedSearch(gridmap_pickup, starts, assignmentVectpickup, pickup, x_size, y_size);
        vector<Path> s2 = unconstrainedSearch(gridmap_delivery, pickup, assignmentVectdelivery, delivery, x_size, y_size);
        vector<Path> s(s1);
        for(int ii = 0; ii < s1.size(); ii++)
            s[ii].pathVect.insert(s[ii].pathVect.end(),s2[assignmentVectpickup[ii]].pathVect.begin(),s2[assignmentVectpickup[ii]].pathVect.end());
        lengthen_solution(s, numofagents);
        start_node.set_solution(s);
        start_node.set_cost(get_SIC(start_node, numofagents));
        
        // Push the start node in to the OPEN list
        OPEN.push(start_node);
        
        // This is set to show the unconstrained search work for the oneshot planning case
        final_node = start_node;
        
        int cnt = 0;
        while (!OPEN.empty()) 
        {
            
            Node curr = OPEN.top(); // set the current node as the top of the OPEN list
            OPEN.pop(); // pop the top node
            
            tuple<int, Point, int> conflict1; // set the tuple to define the first conflict
            tuple<int, Point, int> conflict2; // set the tuple to define the second conflict
            
            // Get assignment of current node and check if conflict exists
            vector<int> assignmentVectpickup = curr.get_assignmentvect();
            vector<int> assignmentVectdelivery = curr.get_assignmentvect_d();
            
            int no_conflict = check_conflict(curr, numofagents, conflict1, conflict2);
            
            // If no conflixt exists return the current node as the final solution
            if (no_conflict)
            {
                delivery_reached = 1;
                final_node = curr;
                clock_t t = clock() - start_time;
                printf("\nTime Taken  %f seconds.\n", ((float)t) / CLOCKS_PER_SEC);
                // printf("\nFINAL COST is %f \n", final_node.get_cost());
                printf("This is the final solution:\n");
                print_solutions(final_node, numofagents);
                break;
            }
            
            double* pickuppos_child = curr.get_assignment();
            double* deliverypos_child = curr.get_assignment_d();
            vector<Point> pickuppose_child = double2pointvector(pickuppos_child, numofpickup);
            vector<Point> deliverypose_child = double2pointvector(deliverypos_child, numofdelivery);

            // create child node for conflicting agent 1
            Node child_node1;
            child_node1.set_root(0);
            child_node1.set_constraints(curr.get_constraints());
            child_node1.push_constraints(get<0>(conflict1), get<1>(conflict1), get<2>(conflict1));
            child_node1.set_assignment(pickuppos_child);
            child_node1.set_assignmentvect(assignmentVectpickup);
            child_node1.set_assignment_d(deliverypos_child);
            child_node1.set_assignmentvect_d(assignmentVectdelivery);
            
            vector<Path> x;
            vector<tuple<int, Point, int>> cd1_constraints = child_node1.get_constraints();
            for (int i = 0; i < numofagents; i++) 
            {
                if (time_exceeded == false)
                {
                    vector<tuple<int, Point, int>> constraints_per_agent;
                    for (int j = 0; j < cd1_constraints.size(); j++){
                        if (i == get<0>(cd1_constraints[j])) {
                            constraints_per_agent.push_back(cd1_constraints[j]);
                        }
                    }
                    if (constraints_per_agent.empty()) {
                        x.push_back(curr.get_solution()[i]);
                    }
                    else {
                        x.push_back(constrainedSearch( gridmap_pickup, gridmap_delivery, starts[i], i, assignmentVectpickup, 
                            assignmentVectdelivery, pickup[assignmentVectpickup[i]], delivery[assignmentVectdelivery[i]], 
                            constraints_per_agent, x_size, y_size, map, collision_thresh, time_exceeded));
                    }
                }
            }
            
            if (time_exceeded == true)
            {
                break;
            }
            
            lengthen_solution(x, numofagents);
            child_node1.set_solution(x);
            child_node1.set_cost(get_SIC(child_node1, numofagents));
            OPEN.push(child_node1);
            // print_solutions(child_node1, numofagents);
            
           
            // create child node for conflicting agent 2
            Node child_node2;
            child_node2.set_root(0);
            child_node2.set_constraints(curr.get_constraints());
            child_node2.push_constraints(get<0>(conflict2), get<1>(conflict2), get<2>(conflict2));
            child_node2.set_assignment(pickuppos_child);
            child_node2.set_assignmentvect(assignmentVectpickup);
            child_node2.set_assignment_d(deliverypos_child);
            child_node2.set_assignmentvect_d(assignmentVectdelivery);

            vector<Path> y;
            vector<tuple<int, Point, int>> cd2_constraints = child_node2.get_constraints();
            for (int i = 0; i < numofagents; i++) 
            {
                if (time_exceeded == false)
                {
                    vector<tuple<int, Point, int>> constraints_per_agent;
                    for (int j = 0; j < cd2_constraints.size(); j++) {
                        if (i == get<0>(cd2_constraints[j])) {
                            constraints_per_agent.push_back(cd2_constraints[j]);
                        }
                    }
                    if (constraints_per_agent.empty()) {
                        y.push_back(curr.get_solution()[i]);
                    }
                    else {
                        y.push_back(constrainedSearch( gridmap_pickup, gridmap_delivery, starts[i], i, assignmentVectpickup, 
                            assignmentVectdelivery, pickup[assignmentVectpickup[i]], delivery[assignmentVectdelivery[i]], 
                            constraints_per_agent, x_size, y_size, map, collision_thresh, time_exceeded) );
                    }
                }
            }
            
            if (time_exceeded == true)
            {
                break;
            }
            
            lengthen_solution(y, numofagents);
            child_node2.set_solution(y);      
            child_node2.set_cost(get_SIC(child_node2, numofagents));
            OPEN.push(child_node2);
            // print_solutions(child_node2, numofagents);

            clock_t t = clock() - start_time;
            if (((float)t) / CLOCKS_PER_SEC > 150)
            {
                break;
            }
            
        }   
    }

    // Get the solution from the final node in the search
    vector<Path> set_of_sol = final_node.get_solution();
    // Set the assignment vector for the pickup
    vector<int> assg = final_node.get_assignmentvect();
    for (int i = 0; i < numofagents; i++)
    {
        assign_pickup[i] = assg[i];
        //mexPrintf("%f\n",assign_pickup[i]);
    }
    // Set the assignment vector for the delivery
    vector<int> assg_d = final_node.get_assignmentvect_d();
    for (int i = 0; i < numofagents; i++)
    {
        assign_delivery[i] = assg_d[i];
        //mexPrintf("%f\n",assign_delivery[i]);
    }
    
    // Set delivery point reached (only if the constrained search is not being run) 
    // delivery_reached = 1;
    
    if (delivery_reached == 0)
       mexPrintf("\n\n TIME EXCEEDED. SOLUTION NOT FOUND \n\n");
    
    // print_solutions(final_node, numofagents);
    
    // If delivery is reached, then set action ptr to the planned action 
    if (delivery_reached == 1)
    {
        // mexPrintf("\nGGGGGG %d\n",curr_time);
        for (int i = 0; i < numofagents; i++) 
        {
            vector<Point> sol = set_of_sol[i].pathVect;
            action_ptr[i] = sol[curr_time].x_pos;
            action_ptr[i + numofagents] = sol[curr_time].y_pos;
            //mexPrintf("%f %f\n",action_ptr[i], action_ptr[i + numofagents]);
        }   
    }
    
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction(int nlhs, mxArray* plhs[],
    int nrhs, const mxArray* prhs[])
{
    /* Check for proper number of arguments */
    if (nrhs != 10) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
            "Ten input arguments required.");
    }
    else if (nlhs != 3) {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
            "Three output argument required.");
    }

    int numofagents = (int)mxGetScalar(NUM_OF_AGENTS);
    int numofpickup = (int)mxGetScalar(NUM_OF_PICKUP);
    int numofdelivery = (int)mxGetScalar(NUM_OF_DELIVERY);

    /* get the dimensions of the map*/
    int dim_M = mxGetM(MAP_DIM);
    int dim_N = mxGetN(MAP_DIM);
    if (dim_M != 1 || dim_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidmapdimensions",
            "mapdimensions vector should be 1 by 2.");
    }
    double* mapdims = mxGetPr(MAP_DIM);
    int x_size = (int)mapdims[0];
    int y_size = (int)mapdims[1];

    /* get the dimensions of the map and the map matrix itself*/
    double* map = mxGetPr(MAP_IN);

    /* Get collision threshold for problem */
    int collision_thresh = (int)mxGetScalar(COLLISION_THRESH);

    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);

    /* get the current robot pose*/
    int robotpose_M = mxGetM(ROBOT_POS);
    int robotpose_N = mxGetN(ROBOT_POS);

    if (robotpose_M != numofagents || robotpose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
            "robotpose vector should be NUMOFAGENTS by 2 matrix.");
    }
    double* robotpose = mxGetPr(ROBOT_POS);

    /* get the pickup pose*/
    int pickuppose_M = mxGetM(PICKUP_POS);
    int pickuppose_N = mxGetN(PICKUP_POS);

    if (pickuppose_M != numofpickup || pickuppose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidpickuppose",
            "pickuppose vector should be NUMOFPICKUP by 2 matrix.");
    }
    double* pickuppose = mxGetPr(PICKUP_POS);
    
    /* get the delivery pose*/
    int deliverypose_M = mxGetM(DELIVERY_POS);
    int deliverypose_N = mxGetN(DELIVERY_POS);

    if (deliverypose_M != numofdelivery || deliverypose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidpickuppose",
            "deliverypose vector should be NUMOFDELIVERY by 2 matrix.");
    }
    double* deliverypose = mxGetPr(DELIVERY_POS);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize)numofagents, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
    double* action_ptr = (double*)mxGetData(ACTION_OUT);

    /* Create a matrix for the return assignment */
    ASSIGN_PICKUP = mxCreateNumericMatrix((mwSize)numofagents, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
    double* assign_pickup = (double*)mxGetData(ASSIGN_PICKUP);
    
    /* Create a matrix for the return assignment */
    ASSIGN_DELIVERY = mxCreateNumericMatrix((mwSize)numofpickup, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
    double* assign_delivery = (double*)mxGetData(ASSIGN_DELIVERY);

    /* Do the actual planning in a subroutine */
    planner(numofagents, numofpickup, numofdelivery, x_size, y_size, collision_thresh, robotpose, pickuppose, deliverypose, map, curr_time, &action_ptr[0], &assign_pickup[0], &assign_delivery[0]);
    return;
}