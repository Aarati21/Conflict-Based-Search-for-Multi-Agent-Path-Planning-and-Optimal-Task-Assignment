// Derived from the implementation in 
// http://csclab.murraystate.edu/~bob.pilgrim/445/munkres.html

#include "hungarian.h"
#include <queue>

int main()
{
	cout << "\nHungarian Method for nxn optimal assignment\n\n";

	vector<vector<double>> cost_matrix = {{11,7,10,17,10},{13,21,7,11,13},{13,13,15,13,14},{18,10,13,16,14},{12,8,16,19,10}};
	// vector<vector<int>> cost_matrix = {{1,2,3},{2,4,6},{3,6,9}};
	taskassignment t1(cost_matrix,5);
	vector<vector<int>> assignment = t1.hungarian();

	cout << "The optimal assignment is: \n\n";
	t1.print_matrix(assignment);
	cout << endl;

	return 0;
}