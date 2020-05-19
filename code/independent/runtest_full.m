function runtest_full(problemfile)

%% Simulation Setup
% Read the map text file and store the variables
[numofagents, numofgoals, mapdims , C, robotstart, pickuppose, deliverypose, envmap] = readproblem(problemfile);

% Close all previous windows
close all;

% Draw the Environment
figure('units','normalized','outerposition',[0 0 1 1]);
ccc = gray;
ccc = flipud(ccc);
colormap(ccc);
imagesc(envmap'); axis square; hold on;

col_arr = zeros(numofagents,3);
for cc = 1:numofagents
    col_arr(cc,:) = [rand(),0.7,1];
end
col_arr = hsv2rgb(col_arr);

%% Pickup Simulation
% Starting Positions of the Robot and Pickup Location
time = 0;
robotpos = robotstart;
goalpos = pickuppose;
delpos = deliverypose;

% Logging required metrics for each agent
numofmoves = zeros(1,numofagents);
caught = false(1,numofgoals);
pathcost = zeros(1,numofagents);

% Draw the agent's starting positions
for ii = 1:numofagents
    hr = -1;
    if (hr ~= -1)
        delete(hr);
    end
    hr = text(robotpos(ii,1), robotpos(ii,2), 'S', 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    hr = scatter(robotpos(ii,1), robotpos(ii,2), 250, 'b');
end
% Draw the agent's pickup positions
for ii = 1:numofgoals
    ht = -1;
    if (ht ~= -1)
        delete(ht);
    end
    ht = text(goalpos(ii,1), goalpos(ii,2), 'P', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    ht = scatter(goalpos(ii,1), goalpos(ii,2), 250, 'r');
end
% Draw the agent's delivery positions
for ii = 1:numofgoals
    hd = -1;
    if (hd ~= -1)
        delete(hd);
    end
    hd = text(delpos(ii,1), delpos(ii,2), 'D', 'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    hd = scatter(delpos(ii,1), delpos(ii,2), 250, 'g');
end

pause(1.0);

% Vector of plot objects to draw trajectory lines
hr_vec = zeros(numofagents);
hline_vec = [];

while (~all(caught))
    
    % Call robot planner to get assignment and actions
    [newrobotpos,assign] = robotplanner(numofagents, numofgoals, mapdims, C, robotpos, goalpos, envmap, time);
    
    % Throw an error if the assignment has invalid size
    if (size(assign, 1) ~= numofagents)
        fprintf(1, 'ERROR: invalid assignment, check size in output\n');
        return;
    end
    
    % Throw an error if the robotpos has invalid size
    if (size(newrobotpos, 1) ~= numofagents  || size(newrobotpos, 2) ~= size(goalpos, 2))
        fprintf(1, 'ERROR: invalid action, check action size in output\n');
        return;
    end
    
    newrobotpos = cast(newrobotpos, 'like', robotpos);
    
    % Throw an error if the action is invalid or leads to a collision
    for kk = 1:numofagents
        if (newrobotpos(kk,1) < 1 || newrobotpos(kk,1) > size(envmap, 1) || ...
                newrobotpos(kk,2) < 1 || newrobotpos(kk,2) > size(envmap, 2))
            fprintf(1, '\nERROR: out-of-map robot position commanded\n');
            return;
        elseif (envmap(newrobotpos(kk,1), newrobotpos(kk,2)) >= C)
            fprintf(1, '\nERROR: planned action leads to collision\n');
            return;
        elseif (abs(newrobotpos(kk,1)-robotpos(kk,1)) > 1 || abs(newrobotpos(kk,2)-robotpos(kk,2)) > 1)
            fprintf(1, '\nERROR: invalid action commanded. robot must move on 8-connected grid.\n');
            return;
        end
    end
    
    time = time + 1;
    prev_robotpos = robotpos;
    robotpos = newrobotpos;
    
    % Throw an error if agents collide
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if (newrobotpos(pp,1) == newrobotpos(kk,1) && newrobotpos(pp,2) == newrobotpos(kk,2))
                fprintf(1, 'ERROR: Position Collision Detected.\n');
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if (newrobotpos(pp,1) == prev_robotpos(kk,1) && newrobotpos(pp,2) == prev_robotpos(kk,2))
                if (newrobotpos(kk,1) == prev_robotpos(pp,1) && newrobotpos(kk,2) == prev_robotpos(pp,2))
                    fprintf(1, 'ERROR: Edge Collision Detected.\n');
                end
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if ((newrobotpos(pp,1) + prev_robotpos(pp,1))/2 == (newrobotpos(kk,1) + prev_robotpos(kk,1))/2)
                if ((newrobotpos(pp,2) + prev_robotpos(pp,2))/2 == (newrobotpos(kk,2) + prev_robotpos(kk,2))/2)
                    fprintf(1, 'ERROR: Diagonal Collision Detected.\n');
                end
            end
        end
    end
    
    for ii = 1:numofagents
        % Add cost and number of moves iteratively
        if(caught(ii) ~= true)
            pathcost(ii) = pathcost(ii) + envmap(robotpos(ii,1), robotpos(ii,2));
            numofmoves(ii) = numofmoves(ii) + 1;
        end
        
        % Plot the robot position and the trajectory
        hr_vec(ii) = scatter(robotpos(ii,1), robotpos(ii,2), 100, 'b', 'filled');
        h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'black');
        hline_vec = [hline_vec, h_line];
        
        % Check if goal is reached
        thresh = 0.5;
        if (abs(robotpos(ii,1)-goalpos(assign(ii)+1,1)) <= thresh && abs(robotpos(ii,2)-goalpos(assign(ii)+1,2)) <= thresh)
            caught(ii) = true;
        end
    end
    
    pause(0.3);
    
    % Delete the agent position plot after every iteration
    for jj = 1:numofagents
        if (caught(jj)==0)
            delete(hr_vec(jj));
        else
            hr_vec(jj) = scatter(robotpos(jj,1), robotpos(jj,2), 100, 'r', 'filled');
        end
    end
    
end

pause(1);

for i = 1:size(hline_vec,2)
    delete(hline_vec(i));
end

fprintf(1, '\n\nRESULT FOR PICKUP:\n');
fprintf('\tGoals Completed : [');fprintf('%g ', caught);fprintf(']\n');
fprintf('\tTime Taken : %d\n', time);
fprintf('\tMoves Made : [');fprintf('%g ', numofmoves);fprintf(']\n');
fprintf('\tPath Cost : [');fprintf('%g ', pathcost);fprintf(']\n');

pause(1);
%% Delivery Simulation
% Starting Positions of the Pickup and Delivery Location
time = 0;
robotpos = pickuppose;
goalpos = deliverypose;

% Logging required metrics for each agent
numofmoves = zeros(1,numofagents);
caught = false(1,numofgoals);
pathcost = zeros(1,numofagents);

% Vector of plot objects to draw trajectory lines
hr_vec = zeros(numofagents);
hline_vec = [];

while (~all(caught))
    
    % Call robot planner to get assignment and actions
    [newrobotpos,assign] = robotplanner(numofagents, numofgoals, mapdims, C, robotpos, goalpos, envmap, time);
    
    % Throw an error if the assignment has invalid size
    if (size(assign, 1) ~= numofagents)
        fprintf(1, 'ERROR: invalid assignment, check size in output\n');
        return;
    end
    
    % Throw an error if the robotpos has invalid size
    if (size(newrobotpos, 1) ~= numofagents  || size(newrobotpos, 2) ~= size(goalpos, 2))
        fprintf(1, 'ERROR: invalid action, check action size in output\n');
        return;
    end
    
    newrobotpos = cast(newrobotpos, 'like', robotpos);
    
    % Throw an error if the action is invalid or leads to a collision
    for kk = 1:numofagents
        if (newrobotpos(kk,1) < 1 || newrobotpos(kk,1) > size(envmap, 1) || ...
                newrobotpos(kk,2) < 1 || newrobotpos(kk,2) > size(envmap, 2))
            fprintf(1, '\nERROR: out-of-map robot position commanded\n');
            return;
        elseif (envmap(newrobotpos(kk,1), newrobotpos(kk,2)) >= C)
            fprintf(1, '\nERROR: planned action leads to collision\n');
            return;
        elseif (abs(newrobotpos(kk,1)-robotpos(kk,1)) > 1 || abs(newrobotpos(kk,2)-robotpos(kk,2)) > 1)
            fprintf(1, '\nERROR: invalid action commanded. robot must move on 8-connected grid.\n');
            return;
        end
    end
    
    time = time + 1;
    prev_robotpos = robotpos;
    robotpos = newrobotpos;
    
    % Throw an error if agents collide
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if (newrobotpos(pp,1) == newrobotpos(kk,1) && newrobotpos(pp,2) == newrobotpos(kk,2))
                fprintf(1, 'ERROR: Position Collision Detected.\n');
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if (newrobotpos(pp,1) == prev_robotpos(kk,1) && newrobotpos(pp,2) == prev_robotpos(kk,2))
                if (newrobotpos(kk,1) == prev_robotpos(pp,1) && newrobotpos(kk,2) == prev_robotpos(pp,2))
                    fprintf(1, 'ERROR: Edge Collision Detected.\n');
                end
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if ((newrobotpos(pp,1) + prev_robotpos(pp,1))/2 == (newrobotpos(kk,1) + prev_robotpos(kk,1))/2)
                if ((newrobotpos(pp,2) + prev_robotpos(pp,2))/2 == (newrobotpos(kk,2) + prev_robotpos(kk,2))/2)
                    fprintf(1, 'ERROR: Diagonal Collision Detected.\n');
                end
            end
        end
    end
    
    for ii = 1:numofagents
        % Add cost and number of moves iteratively
        if(caught(ii) ~= true)
            pathcost(ii) = pathcost(ii) + envmap(robotpos(ii,1), robotpos(ii,2));
            numofmoves(ii) = numofmoves(ii) + 1;
        end
        
        % Plot the robot position and the trajectory
        hr_vec(ii) = scatter(robotpos(ii,1), robotpos(ii,2), 100, 'b', 'filled');
        h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'black');
        hline_vec = [hline_vec, h_line];
        
        % Check if goal is reached
        thresh = 0.5;
        if (abs(robotpos(ii,1)-goalpos(assign(ii)+1,1)) <= thresh && abs(robotpos(ii,2)-goalpos(assign(ii)+1,2)) <= thresh)
            caught(ii) = true;
        end
    end
    
    pause(0.3);
    
    % Delete the agent position plot after every iteration
    for jj = 1:numofagents
        if (caught(jj)==0)
            delete(hr_vec(jj));
        else
            hr_vec(jj) = scatter(robotpos(jj,1), robotpos(jj,2), 100, 'g', 'filled');
        end
    end
    
end

pause(1);

for i = 1:size(hline_vec,2)
    delete(hline_vec(i));
end

fprintf(1, '\n\nRESULT FOR DELIVERY:\n');
fprintf('\tGoals Completed : [');fprintf('%g ', caught);fprintf(']\n');
fprintf('\tTime Taken : %d\n', time);
fprintf('\tMoves Made : [');fprintf('%g ', numofmoves);fprintf(']\n');
fprintf('\tPath Cost : [');fprintf('%g ', pathcost);fprintf(']\n');