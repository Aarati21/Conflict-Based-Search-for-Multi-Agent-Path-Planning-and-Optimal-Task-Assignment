
/*=================================================================
 *
 * planner_unlinked.cpp
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
#define	NUM_OF_GOALS            prhs[1]
#define	MAP_DIM                 prhs[2]
#define	COLLISION_THRESH        prhs[3]
#define	ROBOT_POS               prhs[4]
#define	GOAL_POS                prhs[5]
#define MAP_IN                  prhs[6]
#define	CURR_TIME               prhs[7]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]
#define	ASSIGN                  plhs[1]
/* access to the map is shifted to account for 0-based indexing in the map,
whereas 1-based indexing in matlab (so, robotpose and goalpose are 1-indexed) */
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

/* Define MAX and MIN as a function */
#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Number of directions possible to move along */
// #define numOfDirs 9

// 9-Connected Grid
// int dX[numOfDirs] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
// int dY[numOfDirs] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

class Node {
private:
    
    double cost;
    bool root;
    vector <tuple<int, Point, int>> constraints;
    vector <Path> solutions;
    double* assignment;
    vector<int> assignmentVect;

public:  
    

    void set_cost(double c) { cost = c; }
    double get_cost() { return cost; }

    void set_assignmentvect(vector<int> a) { assignmentVect = a; }
    vector<int> get_assignmentvect() { return assignmentVect; }

    void set_root(bool r) { root = r; }
    bool get_root() { return root; }

    void push_constraints(int agent, Point index, int time) { constraints.push_back(make_tuple(agent, index, time)); }
    void set_constraints(vector<tuple<int, Point, int>> con) { constraints = con; }
    vector<tuple<int, Point, int>> get_constraints() { return constraints; }

    void set_assignment(double* a) { assignment = a; }
    double* get_assignment() { return assignment; }

    void set_solution(vector<Path> s) { solutions = s; }
    vector<Path> get_solution() { return solutions; }

};
static Node final_node;

struct min_heap {
    bool operator()(Node p1, Node p2)
    {
        return p1.get_cost() > p2.get_cost();
    }
};


bool check_conflict(Node node, int numofagents, tuple<int,  Point, int> &conflict1, tuple<int,  Point, int>& conflict2) {
    vector<Path> sol = node.get_solution();
    for (int i = 0; i < numofagents; i++)
    {
        for (int j = i+1; j < numofagents ; j++) {
            int m, m1, m2;
            vector<Point> agent1 = sol[i].pathVect;
            vector<Point> agent2 = sol[j].pathVect;
            m = min(agent1.size(), agent2.size());
            m1 = agent1.size();
            m2 = agent2.size();
            for (int k = 0; k < m; k++) {
                   
                if (agent1[k] == agent2[k]) {
                   
                    conflict1 = make_tuple(i, agent1[k], k);
                    conflict2 = make_tuple(j, agent1[k], k);
                    return 0;
                }
                if (k > 0) {
                    if ((agent1[k].x_pos - agent2[k].x_pos + agent1[k].y_pos - agent2[k].y_pos) + (agent1[k - 1].x_pos - agent2[k - 1].x_pos + agent1[k - 1].y_pos - agent2[k - 1].y_pos) == 0) {
                        conflict1 = make_tuple(i, agent1[k], k);
                        conflict2 = make_tuple(j, agent2[k], k);
                        return 0;
                    }
                    if (((agent1[k].x_pos + agent1[k-1].x_pos) / 2 == (agent2[k].x_pos + agent2[k - 1].x_pos) / 2 ) &&
                        ((agent1[k].y_pos + agent1[k - 1].y_pos) / 2 == (agent2[k].y_pos + agent2[k - 1].y_pos) / 2)){
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




double get_SIC(Node node, int numofagents) {
    double cost = 0;
    vector<Path> solutions = node.get_solution();
    for (int i = 0; i < numofagents; i++)
    {
        cost += solutions[i].cost;
    }
    return cost;
}

vector<vector<double>> gridmap_to_costmatrix(int numofagents, int numofgoals,vector<vector<State_map> > gridmap, vector<Point> robotPosns) {
    vector<vector<double> > temp;
    for(int i=0; i<numofagents; i++){

        vector<double> tempTemp;
        for(int j=0; j<numofgoals;j++){
            tempTemp.push_back(gridmap[robotPosns[i].y_pos - 1][robotPosns[i].x_pos - 1].getH()[j]);
          
        }
        
        temp.push_back(tempTemp);
    }
    return temp;
}

vector<Point> Guru_to_Roshan(double* pos, int numofagents) {
    vector<Point> pos_vector;
    for (int i = 0; i < numofagents; i++) {
        Point start(pos[i], pos[i + numofagents]); // in terms of x and y positions
        pos_vector.push_back(start);
    }
    return pos_vector;
}


void print_solutions(Node start_node, int numofagents) {
    
    for (int i = 0; i < numofagents; i++) {
        printf("\n Agent  %d:   ", i);
        vector<Path> paths = (start_node.get_solution());
        vector<Point> each_path = paths[i].pathVect;
        for (int j = 0; j < each_path.size(); j++) {
            printf("(%d, %d),   ", each_path[j].x_pos, each_path[j].y_pos);
        }

    }
}

int equal_vectors(vector<int> pass, vector<int> newvec) {
    for (int i = 0; i < pass.size(); i++) {
        if (pass[i] != newvec[i]) {
            printf("unequal %d %d", pass[i], newvec[i]);
            return 0;
        }
    }


    return 1;
}

void print_dble(double* goalpos_new, int numofagents) {
    for (int i = 0; i < numofagents; i++) {
        printf("Agent goal positions \n %f %f \n", goalpos_new[i], goalpos_new[i + numofagents]);

    }
}

void print_vector_of_Points(vector<Point> goals_new_tree, int numofagents) {
    for (int i = 0; i < numofagents; i++) {
        printf("Agent goal  positions for Roshan  %d %d \n", goals_new_tree[i].x_pos, goals_new_tree[i].y_pos);
    }

}

void print_constraint(vector<tuple<int, Point, int>> con, int numofagents) {
    for (int i = 0; i < con.size(); i++) {
        tuple<int, Point, int> c = con[i];
        printf(" \nConstraint for child node agent %d, Point(%d %d), time %d) \n", get<0>(c), get<1>(c).x_pos, get<1>(c).y_pos, get<2>(c));
    }
  
}
void lengthen_solution(vector<Path> &y, int numofagents) {
    int j = 0;
    for (int i = 0; i < numofagents; i++) {
        if (y[i].pathVect.size() > j) {
            j = y[i].pathVect.size();
        }
        
    }
    for (int i = 0; i < numofagents; i++) {

        int m = y[i].pathVect.size();
        Point last = y[i].pathVect[m - 1];
        while (y[i].pathVect.size() < j) {
            y[i].pathVect.push_back(last);
        }

    }

}

static void planner(
        int numofagents,
        int numofgoals,
        int x_size,
        int y_size,
        int collision_thresh,
        double* robotpos,
        double* goalpos,
        double*	map,
        int curr_time,
        double* action_ptr,
        double* assign
        )
{   


       
    int goals_reached = 0;
    clock_t start_time = clock();
    if (curr_time == 0) {
        cout << endl;
        
        priority_queue<Node, vector<Node>, min_heap> OPEN;
        priority_queue<ASG, vector<ASG>, ASG_Comparator> ASG_OPEN;

        //  Defines start and goal position for Roshan's code
        vector<Point> goals = Guru_to_Roshan(goalpos, numofagents);
        vector<Point> starts = Guru_to_Roshan(robotpos, numofagents);
        //  For Guru's part, start and goals are still defined as arrays - double*

        vector<vector<int>> PAST_ASSIGNMENTS;
        //  Define gridmap for heuristic calculation for Dijkstra expansions and cost matrix
        State_map state_init_map(numofgoals);
        vector<vector<State_map> > gridmap(y_size, vector<State_map>(x_size, state_init_map));
        backDijkstra(gridmap, goals, map, x_size, y_size, collision_thresh);

        // cout<<"gridmap for zeroth goal is "<<endl;
        for(int i=0; i< y_size; i++){
            for(int j=0; j< x_size; j++ ){
                //cout<<gridmap[i][j].getH()[0]<<", ";
            }
            // cout<<endl;
        }

        vector<int> assignmentVectstart;
        vector<vector<double>> cost_matrix = gridmap_to_costmatrix(numofagents, numofgoals, gridmap, starts);

        
        Node start_node;
        start_node.set_root(1);
        // get first assignment call to Guru's initial assignment function. Should return goal positions of type double*.         
        double* goalpos_new = first_assignment(robotpos, goalpos, cost_matrix, ASG_OPEN, assignmentVectstart);
        PAST_ASSIGNMENTS.push_back(assignmentVectstart);
        start_node.set_assignment(goalpos_new);
        start_node.set_assignmentvect(assignmentVectstart);
        vector<Path> s = unconstrainedSearch(gridmap, starts, assignmentVectstart, goals, x_size, y_size);
        lengthen_solution(s, numofagents);
        start_node.set_solution(s);
        //print_solutions(start_node, numofagents);
        start_node.set_cost(get_SIC(start_node, numofagents));
        OPEN.push(start_node);
        
       
              

       
       
        cout << endl;
       
        
        while (!OPEN.empty()  ) {
            
            Node curr = OPEN.top();
            OPEN.pop();
            tuple<int, Point, int> conflict1;
            tuple<int, Point, int> conflict2;

            vector<int> assignmentVect = curr.get_assignmentvect();
            int no_conflict = check_conflict(curr, numofagents, conflict1, conflict2);
            
            if (no_conflict) {
                goals_reached = 1;
                final_node = curr;
                clock_t t = clock() - start_time;
                printf("\nTime Taken  %f seconds.\n", ((float)t) / CLOCKS_PER_SEC);
                //printf("\nFINAL COST is %f \n", final_node.get_cost());
                printf("This is the final solution:\n");
                print_solutions(final_node, numofagents);
                // printf("\n goals reached\n");
                break;
            }
            
           
            //create root node of new tree
            if (curr.get_root() ) {
                
                // printf("Trying new assignment \n");
                Node new_node;
                new_node.set_root(1);
                vector<int> new_assignmentVect = assignmentVect;
                vector<vector<double>> cost_matrix_new = gridmap_to_costmatrix(numofagents, numofgoals, gridmap, starts);
                vector<Path>sol = curr.get_solution();

                
                for (int i = 0; i < numofagents; i++) {
                    cost_matrix_new[new_assignmentVect[i]][i] = sol[i].cost;

                }

              
                
                double* goalpos_new = next_assignment(robotpos, goalpos, cost_matrix_new, ASG_OPEN, new_assignmentVect);
                // printf("New Assignment vector");
                for (int i = 0; i < new_assignmentVect.size(); i++) {
                    // printf("%d", new_assignmentVect[i]);
                }
                
                int m = 0;
                for (int i = 0; i < PAST_ASSIGNMENTS.size(); i++) {
                    if (equal_vectors(PAST_ASSIGNMENTS[i], new_assignmentVect)) {
                       
                        m = 1;
                        break;
                    }
                    
                }
                if (m == 0){
                    // printf("\nDifferent from old assignment \n");
                    PAST_ASSIGNMENTS.push_back(assignmentVectstart);
                
                    new_node.set_assignment(goalpos_new);
                    new_node.set_assignmentvect(new_assignmentVect);
                    vector<Path> z = unconstrainedSearch(gridmap, starts, new_assignmentVect, goals, x_size, y_size);
                    lengthen_solution(z, numofagents);
                    new_node.set_solution(z); 
                    new_node.set_cost(get_SIC(new_node, numofagents));
                    OPEN.push(new_node);
                }
                else {
                    // printf("\nSame as old assignment");
                }
            }
            
            
            
            double* goalpos_child = curr.get_assignment();
            vector<Point> goals_child = Guru_to_Roshan(goalpos_child, numofgoals);

            



            Node child_node1;
            child_node1.set_root(0);
            child_node1.set_constraints(curr.get_constraints());
            child_node1.push_constraints(get<0>(conflict1), get<1>(conflict1), get<2>(conflict1));
            child_node1.set_assignment(goalpos_child);
            child_node1.set_assignmentvect(assignmentVect);
            
            vector<Path> x;
            vector<tuple<int, Point, int>> cd1_constraints = child_node1.get_constraints();
            for (int i = 0; i < numofagents; i++) {
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
                    x.push_back(constrainedSearch(gridmap, starts[i], i, assignmentVect, goals_child, constraints_per_agent, x_size, y_size, map, collision_thresh));

                }
            }
            lengthen_solution(x, numofagents);
            child_node1.set_solution(x);
            //print_solutions(child_node1, numofagents);
            
            child_node1.set_cost(get_SIC(child_node1, numofagents));
            OPEN.push(child_node1);
            

           
            // create child node for conflicting agent 2
            Node child_node2;
            child_node2.set_root(0);
            child_node2.set_constraints(curr.get_constraints());
            child_node2.push_constraints(get<0>(conflict2), get<1>(conflict2), get<2>(conflict2));
            child_node2.set_assignment(goalpos_child);
            child_node2.set_assignmentvect(assignmentVect);

            vector<Path> y;
            vector<tuple<int, Point, int>> cd2_constraints = child_node2.get_constraints();
            for (int i = 0; i < numofagents; i++) {
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
                    y.push_back(constrainedSearch(gridmap, starts[i], i, assignmentVect, goals_child, 
                        constraints_per_agent, x_size, y_size, map, collision_thresh));

                }
            }
            lengthen_solution(y, numofagents);
            child_node2.set_solution(y);      
            
            child_node2.set_cost(get_SIC(child_node2, numofagents));
            OPEN.push(child_node2);
           
            clock_t curr_t = clock() - start_time;
            float curr_time = ((float)curr_t) / CLOCKS_PER_SEC;
            if (curr_time > 60)
                break;
          
        }
       
    }

    
    vector<Path> set_of_sol = final_node.get_solution();
    //print_solutions(final_node, numofagents);
    vector<int> assg = final_node.get_assignmentvect();
    for (int i = 0; i < numofagents; i++)
    {
        assign[i] = assg[i];
    }
   
    goals_reached = 1;
    //print_solutions(final_node, numofagents);
    if (goals_reached) {
        for (int i = 0; i < numofagents; i++) {
            vector<Point> sol = set_of_sol[i].pathVect;
            
            action_ptr[i] = sol[curr_time].x_pos;
            action_ptr[i + numofagents] = sol[curr_time].y_pos;
            
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
    if (nrhs != 8) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
            "Eight input arguments required.");
    }
    else if (nlhs != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
            "One output argument required.");
    }

    int numofagents = (int)mxGetScalar(NUM_OF_AGENTS);
    int numofgoals = (int)mxGetScalar(NUM_OF_GOALS);

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

    /* get the goal pose*/
    int goalpose_M = mxGetM(GOAL_POS);
    int goalpose_N = mxGetN(GOAL_POS);

    if (goalpose_M != numofgoals || goalpose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidgoalpose",
            "goalpose vector should be NUMOFGOALS by 2 matrix.");
    }
    double* goalpose = mxGetPr(GOAL_POS);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize)numofagents, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
    double* action_ptr = (double*)mxGetData(ACTION_OUT);

    /* Create a matrix for the return assignment */
    ASSIGN = mxCreateNumericMatrix((mwSize)numofagents, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
    double* assign = (double*)mxGetData(ASSIGN);

    /* Do the actual planning in a subroutine */
    planner(numofagents, numofgoals, x_size, y_size, collision_thresh, robotpose, goalpose, map, curr_time, &action_ptr[0], &assign[0]);
    return;
}