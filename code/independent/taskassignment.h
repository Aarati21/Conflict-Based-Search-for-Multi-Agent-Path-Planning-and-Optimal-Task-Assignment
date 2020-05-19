#include "hungarian.h"
#include <queue>

class ASG
{
private:
	vector<vector<int>> solution;
	vector<vector<int>> constraintsI;
	vector<vector<int>> constraintsO;
	int cost;
public:
	ASG(vector<vector<int>> new_assign, vector<vector<double>> costmatrix)
	{
		solution = new_assign;
		cost = costofassignment(costmatrix);
	}
	
	ASG(){}

	vector<vector<int>> getsolution(){return solution;}
	vector<vector<int>> getconstraintsI(){return constraintsI;}
	vector<vector<int>> getconstraintsO(){return constraintsO;}
	double getcost(){return cost;}
	void setsolution(vector<vector<int>> solution){this->solution = solution;}
	void setconstraintsI(vector<vector<int>> constraintsI){this->constraintsI = constraintsI;}
	void setconstraintsO(vector<vector<int>> constraintsO){this->constraintsO = constraintsO;}
	void setcost(double cost){this->cost = cost;}
	int costofassignment(vector<vector<double>> cost_matrix)
	{
		int n = cost_matrix.size();
		int m = cost_matrix[0].size();
		int totalcost = 0;
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
        return p1.getcost() > p2.getcost();
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

double* first_assignment(double* robotpos, double* goalpos, vector<vector<double>> costmatrix, 
	priority_queue<ASG, vector<ASG>, ASG_Comparator> &ASG_OPEN, vector<int> &assignmentvect)
{
	int n = costmatrix.size();
	static double goalnew[100]; 
    vector<int> assignnew;
    
	ASG R;
	R.setsolution(constrainedassignment(R.getconstraintsI(), R.getconstraintsO(), costmatrix));
	R.setcost(R.costofassignment(costmatrix));
	ASG_OPEN.push(R);

    for(int i = 0; i < n; i++)
    {
        assignnew.push_back(R.getgoalindex(i));
    }
    assignmentvect = assignnew;
	goalsort(goalpos, R.getsolution(), goalnew, n);

	return goalnew;
}

double* next_assignment(double* robotpos, double* goalpos, vector<vector<double>> costmatrix, priority_queue<ASG, vector<ASG>, ASG_Comparator> &ASG_OPEN, vector<int> &assignmentvect)
{
	int n = costmatrix.size();
	static double goalnew[100]; 
    vector<int> assignnew;
	// When ASG_OPEN is empty and no next assignment is possible
	if(ASG_OPEN.empty())
	{
		goalnew[0] = -1;
		return goalnew;
	}

	ASG P = ASG_OPEN.top();
	ASG_OPEN.pop();
	vector<vector<int>>	PconstraintsI = P.getconstraintsI();
	vector<vector<int>>	PconstraintsO = P.getconstraintsO();

	for(int i = 0; i < n; i++)
	{
		int foundconstraint = 0;
		for(int j = 0; j < PconstraintsI.size(); j++)
		{
			if(PconstraintsI[j][0] == i)
				foundconstraint = 1;
		}
		if(!foundconstraint)
		{
			ASG Q;
			vector<vector<int>> QconstraintsI = PconstraintsI;
			vector<vector<int>> QconstraintsO = PconstraintsO;
			QconstraintsO.push_back({i,P.getgoalindex(i)});
			for(int j = 0; j < i; j++)
			{
				QconstraintsI.push_back({j,P.getgoalindex(j)});
			}
			Q.setconstraintsI(QconstraintsI);
			Q.setconstraintsO(QconstraintsO);
			Q.setsolution(constrainedassignment(Q.getconstraintsI(), Q.getconstraintsO(), costmatrix));
			Q.setcost(Q.costofassignment(costmatrix));
			if (!Q.solutionempty())
			{
				ASG_OPEN.push(Q);
			}
		}
	}
    
    for(int i = 0; i < n; i++)
    {
        assignnew.push_back(P.getgoalindex(i));
    }
    assignmentvect = assignnew;
    
	goalsort(goalpos, P.getsolution(), goalnew, n);

	return goalnew;
}
