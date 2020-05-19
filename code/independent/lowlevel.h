#include <iostream>
#include <math.h>
#include <limits>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>
// #include <map>
#include "param.h"

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
// #define NUMOFDIRS 9

#ifndef LOWLEVEL_H
#define LOWLEVEL_H

///// common class definitions //////
struct Point {
	int x_pos, y_pos;
	Point() {}
	Point(int xIn, int yIn) : x_pos(xIn), y_pos(yIn) {}
	bool operator==(const Point& rhs)
	{
		return (x_pos == rhs.x_pos) && (y_pos == rhs.y_pos);
	}
};

struct Path {
	vector<Point> pathVect;
	double cost;
};

// defines a point on the gridmap
class State_map {

private:
	Point position;
	vector<double> h_vals;
	bool expanded;
	int goalIndex;

public:
	State_map(int numofgoalsIn);

	Point getPoint() const { return position; }
	vector<double> getH() const { return h_vals; }
	int getGoalIndex() const { return goalIndex; }
	bool isExpanded() const { return expanded; }
	void setPoint(Point positionIn) { position = positionIn; }
	void setX(int x_posIn) { position.x_pos = x_posIn; }
	void setY(int y_posIn) { position.y_pos = y_posIn; }
	void setH(int index, double h_valIn) { h_vals[index] = h_valIn; }
	void setGoalIndex(int goalIndexIn) { goalIndex = goalIndexIn; }
	void expand() { expanded = true; }
	void contract() { expanded = false; }
};

// compare struct for the priority queue
struct CompareF_map {
	bool operator()(State_map const& s1, State_map const& s2) {
		// return "true" if "p1" is ordered before "p2", for example:
		return s1.getH()[s1.getGoalIndex()] > s2.getH()[s2.getGoalIndex()];
	}
};

// defines a point for 3D search
class Node_time {

private:
	Point position;
	int time_step;
	double g_val;
	double h_val;
	Node_time* parent;
	vector<Node_time*> successors;
	bool expanded;

public:
	Node_time() : g_val(numeric_limits<double>::infinity()), expanded(false), parent(nullptr) {}

	Point getPoint() const { return position; }
	Node_time* getParent() const { return parent; }
	vector<Node_time*> getSuccessors() const { return successors; }
	int getTime() const { return time_step; }
	double getG() const { return g_val; }
	double getH() const { return h_val; }
	// int getGoalIndex() const {return goalIndex;}
	bool isExpanded() const { return expanded; }

	void setPoint(Point positionIn) { position = positionIn; }
	void setParent(Node_time* parentIn) { parent = parentIn; }
	void addSuccessor(Node_time* succNode) { successors.push_back(succNode); }
	void setX(int x_posIn) { position.x_pos = x_posIn; }
	void setY(int y_posIn) { position.y_pos = y_posIn; }
	void setT(int time_stepIn) { time_step = time_stepIn; }
	void setG(double g_valIn) { g_val = g_valIn; }
	void setH(double h_valIn) { h_val = h_valIn; }
	// void setGoalIndex(int goalIndexIn){goalIndex = goalIndexIn;}
	void expand() { expanded = true; }
	void contract() { expanded = false; }
};

// compare struct for the priority queue
struct CompareF_time {
	bool operator()(Node_time* const& s1, Node_time* const& s2) {
		// return "true" if "p1" is ordered before "p2", for example:
		long eps = 1;
		return eps * s1->getH() + s1->getG() > eps * s2->getH() + s1->getG();
	}
};


void backDijkstra(vector<vector<State_map> >& gridmapIn, const vector<Point>& goals, double* map,
	int x_size, int y_size, int collision_thresh);

vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn, const vector<Point>& robotPosnsIn,
	const vector<int>& assignment, const vector<Point>& goalsIn, int x_size, int y_size);

Path constrainedSearch(const vector< vector<State_map> >& gridmapIn, const Point& robotPosnIn, int robotIndex,
	const vector<int>& assignment, const vector<Point>& goalsIn, const vector<tuple<int, Point, int> >& tempConstr,
	int x_size, int y_size, double* map, int collision_thresh);

unsigned long long GetIndex(int x, int y, int t);

bool CBSOkay(const vector<tuple<int, Point, int> >& tempConstr, int newx, int newy, int newt, int i_agent);

double diagonalCost(int dir);

#endif
