function runtest_full(problemfile)

%% Simulation Setup
% Read the map text file and store the variables
[numofagents, numofgoals, numofdelivery, mapdims , C, robotstart, pickuppose, deliverypose, envmap] = readproblem(problemfile);

% Close all previous windows
close all;

% Draw the Environment
figure('units','normalized','outerposition',[0 0 1 1]);
ccc = gray;
ccc = flipud(ccc);
colormap(ccc);
imagesc(envmap'); axis square; hold on;

plotsize1 = 800/2;
plotsize2 = 300/2;
plotsize3 = 100/2;

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
pathcost = zeros(1,numofagents);
picked = false(1,numofgoals);
delivered = false(1,numofgoals);

% Draw the agent's starting positions
for ii = 1:numofagents
    hr = -1;
    if (hr ~= -1)
        delete(hr);
    end
    %hr = scatter(robotpos(ii,1), robotpos(ii,2), plotsize1, [1,0.2,0.2], 'Filled', 'square');
    hr = text(robotpos(ii,1), robotpos(ii,2), 'S', 'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end
% Draw the agent's pickup positions
for ii = 1:numofgoals
    ht = -1;
    if (ht ~= -1)
        delete(ht);
    end
    ht = scatter(goalpos(ii,1), goalpos(ii,2), plotsize1, [0.8,0.8,0.8], 'Filled', 'square');
    ht = text(goalpos(ii,1), goalpos(ii,2), 'P', 'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end
% Draw the agent's delivery positions
for ii = 1:numofgoals
    hd = -1;
    if (hd ~= -1)
        delete(hd);
    end
    hd = scatter(delpos(ii,1), delpos(ii,2), plotsize1, [0.8,0.8,0.8], 'square');
    % hd = text(delpos(ii,1), delpos(ii,2), 'D', 'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end
% 
pause(1.0);

% Vector of plot objects to draw trajectory lines
hr_vec = zeros(numofagents);
hr_vec_pick = zeros(numofagents);
hline_vec_full = [];

while (~all(delivered) || ~all(picked))
    
    % Call robot planner to get assignment and actions
    [newrobotpos,assign_pickup, assign_delivery] = robotplanner(numofagents, numofgoals, mapdims, C, robotpos, goalpos, envmap, time, delpos, numofdelivery);

    % Throw an error if the assignment has invalid size
    if (size(assign_pickup, 1) ~= numofagents)
        fprintf(1, '\nERROR: invalid assignment, check size in output\n');
        return;
    end
    
    % Throw an error if the robotpos has invalid size
    if (size(newrobotpos, 1) ~= numofagents  || size(newrobotpos, 2) ~= size(goalpos, 2))
        fprintf(1, '\nERROR: invalid action, check action size in output\n');
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
                fprintf(1, '\nERROR: Position Collision Detected.\n');
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if (newrobotpos(pp,1) == prev_robotpos(kk,1) && newrobotpos(pp,2) == prev_robotpos(kk,2))
                if (newrobotpos(kk,1) == prev_robotpos(pp,1) && newrobotpos(kk,2) == prev_robotpos(pp,2))
                    fprintf(1, '\nERROR: Edge Collision Detected.\n');
                end
            end
        end
    end
    for pp = 1:numofagents
        for kk = (pp+1):numofagents
            if ((newrobotpos(pp,1) + prev_robotpos(pp,1))/2 == (newrobotpos(kk,1) + prev_robotpos(kk,1))/2)
                if ((newrobotpos(pp,2) + prev_robotpos(pp,2))/2 == (newrobotpos(kk,2) + prev_robotpos(kk,2))/2)
                    fprintf(1, '\nERROR: Diagonal Collision Detected.\n');
                end
            end
        end
    end
    
    hline_vec = [];
    for ii = 1:numofagents
        % Add cost and number of moves iteratively
        if(delivered(ii) ~= true)
            pathcost(ii) = pathcost(ii) + envmap(robotpos(ii,1), robotpos(ii,2)) + distance([robotpos(ii,1), robotpos(ii,2)],[prev_robotpos(ii,1), prev_robotpos(ii,2)]);
            numofmoves(ii) = numofmoves(ii) + 1;
        end
        
        % Plot the robot position and the trajectory
        % h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'black--');
        if picked(ii) == 0
%             h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'black--');
            hr_vec(ii) = scatter(robotpos(ii,1), robotpos(ii,2), plotsize2, col_arr(ii,:), 'filled');
            hr_vec_pick(ii) = scatter(robotpos(ii,1), robotpos(ii,2), plotsize3, 'w', 'filled');
        else
            h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'black--');
            hr_vec_pick(ii) = scatter(robotpos(ii,1), robotpos(ii,2), plotsize3, 'y', 'filled');
            hr_vec(ii) = scatter(robotpos(ii,1), robotpos(ii,2), plotsize2, col_arr(ii,:), 'filled');
        end
%         hline_vec = [hline_vec; h_line];
        
        % Check if goal is reached
        thresh = 0.5;
        if (abs(robotpos(ii,1)-goalpos(assign_pickup(ii)+1,1)) <= thresh && abs(robotpos(ii,2)-goalpos(assign_pickup(ii)+1,2)) <= thresh)
            picked(ii) = true;
        end
        
        if (abs(robotpos(ii,1)-delpos(assign_delivery(assign_pickup(ii)+1)+1,1)) <= thresh && abs(robotpos(ii,2)-delpos(assign_delivery(assign_pickup(ii)+1)+1,2)) <= thresh)
            delivered(ii) = true;
        end
    end
%     hline_vec_full = [hline_vec_full, hline_vec];
    pause(0.3);
    
    % Delete the agent position plot after every iteration
    for jj = 1:numofagents
        delete(hr_vec(jj));
        delete(hr_vec_pick(jj));
        if picked(jj) == 1
            %delete(hr_vec_pick(jj));
        end
        %delete(hline_vec(jj));
    end
    
    for jj = 1:numofagents
        if ((abs(robotpos(jj,1)-goalpos(assign_pickup(jj)+1,1)) <= thresh && abs(robotpos(jj,2)-goalpos(assign_pickup(jj)+1,2)) <= thresh))
            hr = scatter(robotpos(jj,1), robotpos(jj,2), plotsize1*0.85, 'w', 'Filled', 'square');
            %hr_vec(jj) = scatter(robotpos(jj,1), robotpos(jj,2), plotsize3, 'y', 'filled');
        end
    end
    
    for jj = 1:numofagents
        if ((abs(robotpos(jj,1)-delpos(assign_delivery(assign_pickup(jj)+1)+1,1)) <= thresh && abs(robotpos(jj,2)-delpos(assign_delivery(assign_pickup(jj)+1)+1,2)) <= thresh))
            hr = scatter(robotpos(jj,1), robotpos(jj,2), plotsize1*0.85, [0.8,0.8,0.8], 'Filled', 'square');
            hr_vec_pick(jj) = scatter(robotpos(jj,1), robotpos(jj,2), plotsize2, col_arr(jj,:), 'filled');
            hr_vec(jj) = scatter(robotpos(jj,1), robotpos(jj,2), plotsize3, col_arr(jj,:), 'filled');
        end
    end
    
end

% pause(1);


fprintf(1, '\n\nRESULT FOR PICKUP & DELIVERY:\n');
fprintf('\tGoals Completed : [');fprintf('%g ', delivered);fprintf(']\n');
fprintf('\tTime Taken : %d\n', time);
fprintf('\tMoves Made : [');fprintf('%g ', numofmoves);fprintf(']\n');
fprintf('\tPath Cost : [');fprintf('%g ', pathcost);fprintf(']\n');