#include "taskassignment.h"

int main()
{
	priority_queue<ASG, vector<ASG>, ASG_Comparator> ASG_OPEN;
	
	double robotpos[] = {1,2,3,4,5,6};
	double goalpos[] = {4,3,2,5,6,1};
	double goalpos1[] = {4,3,2,5,6,1};
	vector<vector<double>> cost = {{1,2,3},{2,4,6},{3,6,9}};
	vector<vector<int>> ass = {{1,0,0},{0,0,1},{0,1,0}};

	double* goalnew = first_assignment(robotpos,goalpos,cost,ASG_OPEN);
	ASG a1 = ASG_OPEN.top();
	cout << a1.getcost() << endl;
	// double* goalnew = goalpos1;
	// goalsort(goalpos,ass,goalnew,3);

	for(int i = 0; i < 3; i++)
		cout << goalnew[i] << " " << goalnew[i+3] << endl;

	return 0;
}