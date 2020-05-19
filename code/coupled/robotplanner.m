function [action,assign_pickup, assign_delivery] = robotplanner(numofagents, numofpickup, map_dim, collision_thresh, robotpose, pickuppose, map, curr_time, deliverypose, numofdelivery)

[action,assign_pickup, assign_delivery] = planner_oneshot(numofagents, numofpickup, map_dim, collision_thresh, robotpose, pickuppose, map, curr_time, deliverypose, numofdelivery);

end