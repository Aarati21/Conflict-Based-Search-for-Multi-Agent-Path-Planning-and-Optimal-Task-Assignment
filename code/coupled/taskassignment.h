#include "hungarian.h"
#include <queue>

class ASG
{
private:
	vector<vector<int>> solution;
    vector<vector<int>> solution_d;
	vector<vector<int>> constraintsI;
	vector<vector<int>> constraintsO;
    vector<vector<int>> constraintsI_d;
	vector<vector<int>> constraintsO_d;
	double cost;
    double cost_d;

public:
	ASG(vector<vector<int>> new_assign, vector<vector<double>> costmatrix, vector<vector<int>> new_assign_d, vector<vector<double>> costmatrix_d)
	{
		solution = new_assign;
		cost = costofassignment(costmatrix);
        solution_d = new_assign_d;
		cost_d = costofassignment(costmatrix_d);
	}
	
	ASG(){}

	vector<vector<int>> getsolution(){return solution;}
    vector<vector<int>> getsolution_d(){return solution_d;}
	vector<vector<int>> getconstraintsI(){return constraintsI;}
	vector<vector<int>> getconstraintsO(){return constraintsO;}
	vector<vector<int>> getconstraintsI_d(){return constraintsI_d;}
	vector<vector<int>> getconstraintsO_d(){return constraintsO_d;}
	double getcost(){return cost;}
    double getcost_d(){return cost_d;}
	
    void setsolution(vector<vector<int>> solution){this->solution = solution;}
	void setconstraintsI(vector<vector<int>> constraintsI){this->constraintsI = constraintsI;}
	void setconstraintsO(vector<vector<int>> constraintsO){this->constraintsO = constraintsO;}
	void setcost(double cost){this->cost = cost;}
    void setsolution_d(vector<vector<int>> solution_d){this->solution_d = solution_d;}
	void setconstraintsI_d(vector<vector<int>> constraintsI_d){this->constraintsI_d = constraintsI_d;}
	void setconstraintsO_d(vector<vector<int>> constraintsO_d){this->constraintsO_d = constraintsO_d;}
	void setcost_d(double cost_d){this->cost_d = cost_d;}
    
	double costofassignment(vector<vector<double>> cost_matrix)
	{
		int n = cost_matrix.size();
		int m = cost_matrix[0].size();
		double totalcost = 0;
		for(int i = 0; i < n; i++)
			for(int j = 0; j < m; j++)
				totalcost += cost_matrix[i][j]*solution[i][j];
		return totalcost;
	}
	int getgoalindex(int agentindex)
	{
		int goalindex = -1;
		for(int i = 0; i < solution.size(); i++)
		{
			if (solution[agentindex][i] == 1)
				goalindex = i;
		}
		return goalindex;
	}
    int getgoalindex_d(int agentindex)
	{
		int goalindex = -1;
		for(int i = 0; i < solution_d.size(); i++)
		{
			if (solution_d[agentindex][i] == 1)
				goalindex = i;
		}
		return goalindex;
	}
	int solutionempty()
	{
		if (solution[0][0] < 0)
			return 1;
		return 0;
	}
};

struct ASG_Comparator {
    bool operator()(ASG p1, ASG p2)
    {
        return (p1.getcost()+p1.getcost_d()) > (p2.getcost()+p2.getcost_d());
    }
};

void goalsort(double* goalpos, vector<vector<int>> assignmentmatrix, double* goalnew, int n)
{
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
		{
			if(assignmentmatrix[i][j] == 1)
			{
				goalnew[i] = goalpos[j];
				goalnew[i+n] = goalpos[j+n];
			}
		}
	}
}

vector<vector<int>> constrainedassignment(vector<vector<int>> constraintsI, vector<vector<int>> constraintsO, vector<vector<double>> costmatrix)
{
	int n = costmatrix.size();

	for(int i = 0; i < constraintsI.size(); i++)
	{
		costmatrix[constraintsI[i][0]][constraintsI[i][1]] = 0;
	}
	for(int i = 0; i < constraintsO.size(); i++)
	{
		costmatrix[constraintsO[i][0]][constraintsO[i][1]] = std::numeric_limits<int>::max();
	}

	taskassignment t1(costmatrix,n);
	vector<vector<int>> assignment = t1.hungarian();

	return assignment;
}

void first_assignment(double* robotpos, double* pickuppos, double* deliverypos, vector<vector<double>> costmatrix_pickup, 
	 vector<vector<double>> costmatrix_delivery, vector<int> &assignmentVectpickup, vector<int> &assignmentVectdelivery, 
        double* &pickuppos_new, double* &deliverypos_new, priority_queue<ASG, vector<ASG>, ASG_Comparator> &ASG_OPEN)
{
	int n = costmatrix_pickup.size();
	static double pickupnew[200];
    static double deliverynew[200];
    
	ASG R;
	
    R.setsolution(constrainedassignment(R.getconstraintsI(), R.getconstraintsO(), costmatrix_pickup));
	R.setcost(R.costofassignment(costmatrix_pickup));
    R.setsolution_d(constrainedassignment(R.getconstraintsI_d(), R.getconstraintsO_d(), costmatrix_delivery));
	R.setcost_d(R.costofassignment(costmatrix_delivery));
    
	ASG_OPEN.push(R);

    for(int i = 0; i < n; i++)
    {
        assignmentVectpickup.push_back(R.getgoalindex(i));
        assignmentVectdelivery.push_back(R.getgoalindex_d(i));
    }
    
	goalsort(pickuppos, R.getsolution(), pickupnew, n);
    goalsort(deliverypos, R.getsolution_d(), deliverynew, n);
    
    pickuppos_new = pickupnew;
    deliverypos_new = deliverynew;
}

void next_assignment(double* robotpos, double* pickuppos, double* deliverypos, vector<vector<double>> costmatrix_pickup, 
	 vector<vector<double>> costmatrix_delivery, vector<int> &assignmentVectpickup, vector<int> &assignmentVectdelivery, 
        double* &pickuppos_new, double* &deliverypos_new, priority_queue<ASG, vector<ASG>, ASG_Comparator> &ASG_OPEN)
{
// 	int n = costmatrix.size();
// 	static double pickupnew[200];
//     static double deliverynew[200];
//     vector<int> assignnew_pickup;
//     vector<int> assignnew_delivery;
//     
// 	// When ASG_OPEN is empty and no next assignment is possible
// 	if(ASG_OPEN.empty())
// 	{
// 		pickupnew[0] = -1;
// 		mexPrintf("\nASG_OPEN is empty.\n");
// 	}
// 
// 	ASG P = ASG_OPEN.top();
// 	ASG_OPEN.pop();
// 	vector<vector<int>>	PconstraintsI = P.getconstraintsI();
// 	vector<vector<int>>	PconstraintsO = P.getconstraintsO();
//     vector<vector<int>>	PconstraintsI_d = P.getconstraintsI_d();
// 	vector<vector<int>>	PconstraintsO_d = P.getconstraintsO_d();
// 
// 	for(int i = 0; i < n; i++)
// 	{
// 		int foundconstraint = 0;
// 		for(int j = 0; j < PconstraintsI.size(); j++)
// 		{
// 			if(PconstraintsI[j][0] == i)
// 				foundconstraint = 1;
// 		}
// 		if(!foundconstraint)
// 		{
// 			ASG Q;
// 			vector<vector<int>> QconstraintsI = PconstraintsI;
// 			vector<vector<int>> QconstraintsO = PconstraintsO;
// 			QconstraintsO.push_back({i,P.getgoalindex(i)});
// 			for(int j = 0; j < i; j++)
// 			{
// 				QconstraintsI.push_back({j,P.getgoalindex(j)});
// 			}
// 			Q.setconstraintsI(QconstraintsI);
// 			Q.setconstraintsO(QconstraintsO);
// 			Q.setsolution(constrainedassignment(Q.getconstraintsI(), Q.getconstraintsO(), costmatrix));
// 			Q.setcost(Q.costofassignment(costmatrix));
// 			if (!Q.solutionempty())
// 			{
// 				ASG_OPEN.push(Q);
// 			}
// 		}
// 	}
//    
//     for(int i = 0; i < n; i++)
//     {
//         assignnew.push_back(P.getgoalindex(i));
//     }
//     assignmentvect = assignnew;
//     
// 	goalsort(goalpos, P.getsolution(), goalnew, n);
// 
// 	return goalnew;
}

// Create root node of new tree
//             if (curr.get_root()) 
//             {
//                 // Create a new start node with a new assignment
//                 printf("Trying new assignment \n");
//                 Node new_node;
//                 new_node.set_root(1);
//                 
//                 vector<int> new_assignmentVect_pickup = assignmentVectpickup;
//                 vector<int> new_assignmentVect_delivery = assignmentVectdelivery;
//                 
//                 vector<vector<double>> cost_matrix_pickup_new = gridmap_to_costmatrix(numofagents, numofpickup, gridmap_pickup, starts);
//                 vector<vector<double>> cost_matrix_delivery_new = gridmap_to_costmatrix(numofpickup, numofdelivery, gridmap_delivery, pickup);
//                 
//                 vector<Path> sol_p = curr.get_solution_p();
//                 vector<Path> sol_d = curr.get_solution_d();
//                 vector<Path> sol = curr.get_solution();
//    
//                 for (int i = 0; i < numofagents; i++)
//                 {
//                     cost_matrix_pickup_new[new_assignmentVect_pickup[i]][i] = sol_p[i].cost;
//                     cost_matrix_delivery_new[new_assignmentVect_delivery[i]][i] = sol_d[i].cost;
//                 }
//                 
//                 double* pickup_new;
//                 double* delivery_new;
//                 
// //                 next_assignment(robotpos, pickuppos, deliverypos, cost_matrix_pickup_new, cost_matrix_delivery_new, 
// //                     new_assignmentVect_pickup, new_assignmentVect_delivery, pickup_new, delivery_new, ASG_OPEN);
//                 
//                 printf("New Assignment vector");
//                 for (int i = 0; i < new_assignmentVect.size(); i++) {
//                     printf("%d", new_assignmentVect[i]);
//                 }
//                 
//                 int m = 0;
//                 for (int i = 0; i < PAST_ASSIGNMENTS.size(); i++) {
//                     if (equal_vectors(PAST_ASSIGNMENTS[i], new_assignmentVect)) {
//                        
//                         m = 1;
//                         break;
//                     }
//                     
//                 }
//                 if (m == 0){
//                     printf("\nDifferent from old assignment \n");
//                     PAST_ASSIGNMENTS.push_back(assignmentVectstart);
//                 
//                     new_node.set_assignment(goalpos_new);
//                     new_node.set_assignmentvect(new_assignmentVect);
//                     vector<Path> z = unconstrainedSearch(gridmap, starts, new_assignmentVect, goals, x_size, y_size);
//                     lengthen_solution(z, numofagents);
//                     new_node.set_solution(z); 
//                     new_node.set_cost(get_SIC(new_node, numofagents));
//                     OPEN.push(new_node);
//                 }
//                 else {
//                     printf("\nSame as old assignment");
//                 }
//             }