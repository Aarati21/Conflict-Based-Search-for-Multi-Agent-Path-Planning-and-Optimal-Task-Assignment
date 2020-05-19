function [action,assign] = robotplanner(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time)


[action,assign] = planner_unlinked(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time);

end