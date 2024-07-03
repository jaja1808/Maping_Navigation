%% MAP
nrows = 500;
ncols = 500;
obstacle = false(nrows, ncols);
[x, y] = meshgrid (1:ncols, 1:nrows);

%% Generate some obstacles
obstacle(1:20, :) = true;               % Top wall
obstacle(480:500, :) = true;            % Bottom wall
obstacle(:, 1:20) = true;               % Left wall
obstacle(:, 480:500) = true;            % Right wall
obstacle(200:300, 60:80) = true;        % Middle left division
obstacle(400:415, 200:300) = true;      % Horizontal wall
obstacle(100:150, 360:375) = true;      % Middle right division
obstacle(1:200, 250:265) = true;        % Top-divider wall

%% Create a binary occupancy map from the obstacle matrix
resolution = 1;
map = binaryOccupancyMap(obstacle, resolution);

%% Define start and goal points
startPoint = [15, 45];
goalPoint = [450, 450];

%% Create a D* planner object
planner = Dstar(map);

%% Plan the path
% Set the start and goal points
planner.start = startPoint;
planner.goal = goalPoint;

%% Query the planned path
% Plan the path
planner.plan(goalPoint);
planner.query(startPoint);
show(planner);

%% Get the waypoints
 % Get the waypoints
waypoints = planner.getWaypoints();
% Shift the waypoints slightly to account for the robot size
waypoints = waypoints + repmat([0.25, 0.25], size(waypoints, 1), 1);


%% Plot the map and waypoints
figure;
show(map);
hold on;
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPoint(1), goalPoint(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 2);
legend('Map', 'Start', 'Goal', 'Waypoints');
xlabel('X');
ylabel('Y');
title('Map with Start, Goal, and Waypoints');

%% Bicycle Model
% Define covariance matrices
V = diag([0.02, 0.5*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);
% Create Bicycle object
veh = Bicycle('covar', V);

%% Plot the Vehicle at a position
initPose = [startPoint, 0];  % Starting position
veh.x = initPose;
H = veh.plotv(veh.x);

%% Move the bicycle model along the waypoints
for idx = 1:size(waypoints, 1)
    % Update the vehicle position based on the waypoints
    veh.x(1:2) = waypoints(idx, :);
    % Update visualization
    veh.plotv(H, veh.x);
    drawnow;
end

