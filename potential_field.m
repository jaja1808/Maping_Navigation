%% Generate some points
nrows = 500;
ncols = 500;
obstacle = false(nrows, ncols);
[x, y] = meshgrid (1:ncols, 1:nrows);

%% Generate some obstacles
obstacle(1:20, :) = true;               % Top wall
obstacle(480:500, :) = true;           % Bottom wall
obstacle(:, 1:20) = true;               % Left wall
obstacle(:, 480:500) = true;           % Right wall
obstacle(200:300, 60:80) = true;          % Middle left division
obstacle(400:415, 200:300) = true;         % Horizontal wall
obstacle(100:150, 360:375) = true;     % Middle right division
obstacle(1:200, 250:265) = true;           % Top-divider wall

%% Create a binary occupancy map from the obstacle matrix
resolution = 1;
map = binaryOccupancyMap(obstacle, resolution);

%% Compute distance transform
d = bwdist(obstacle);

% Rescale and transform distances
d2 = (d/100) + 1;
d0 = 2;
nu = 800;
repulsive = nu*((1./d2 - 1/d0).^2);
repulsive(d2 > d0) = 0;

%% Display repulsive potential
figure(1);
m = mesh(repulsive);
m.FaceLighting = 'phong';
axis equal;
title('Repulsive Potential');

%% Compute attractive force
goal = [450, 450];
xi = 1/700;
attractive = xi * ((x - goal(1)).^2 + (y - goal(2)).^2);

figure(2);
m = mesh(attractive);
m.FaceLighting = 'phong';
axis equal;
title('Attractive Potential');

%% Display 2D configuration space
figure(3);
imshow(~obstacle);
hold on;
plot(goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold off;
axis([0 ncols 0 nrows]);
axis xy;
axis on;
xlabel('x');
ylabel('y');
title('Configuration Space');

%% Combine terms
f = attractive + repulsive;

figure(4);
m = mesh(f);
m.FaceLighting = 'phong';
axis equal;
title('Total Potential');

%% Plan route
start = [25, 45];  % Higher start point

%% Bicycle Model
% Define covariance matrices
V = diag([0.02, 0.5*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);
% Create Bicycle object
veh = Bicycle('speedmax',50,'steermax',5,'covar', V);

%% Plot the Vehicle at a position
startPose = [25; 35; 0.2];  % Starting position at the higher point
veh.x = startPose;

%% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:100;        % Time array
% Initial condition
pose = nan(3, numel(tVec));   % Initialize pose matrix with NaN values
pose(:, 1) = startPose;

%% Simulation loop
figure;
show(map);
hold on;

r = rateControl(1/sampleTime);
H = veh.plotv(veh.x);
vRef = 30;

for idx = 2:numel(tVec) 

    angle = findLowestNeighborAngle(f, [veh.x(1),veh.x(2)]);
    wRef = 5 * angle;

    % Compute the velocities
    veh.step(vRef, wRef);

    % Update pose
    pose(:,idx) = veh.x;

    % Update visualization
    veh.plotv(H, veh.x); % Add this line to visualize the bicycle model
    waitfor(r);

end