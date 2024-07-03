%% Map Description
% Size of the map
n = 50;
radius = 1.75;
myMap = Make_map(n, radius);

%% Bicycle Model
% Define covariance matrices
V_bike = diag([0.03, 0.6*pi/180].^2);
V_car = diag([0.02, 0.5*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);

% Create 1st Bicycle object
bike = Bicycle('covar', V_bike);

% Create 2nd Bicycle object
car = Bicycle('covar', V_car);

%% Sample time and time array
sampleTime = 0.1; % Sample time [s]
tVec = 0:sampleTime:30; % Time array

%% Initial conditions
initPose_bike = [2; 2; 0];
initPose_car = [3; 3; 0];

% Initial pose (x y theta)
pose_bike = zeros(3, numel(tVec)); % Pose matrix for bicycle
pose_car = zeros(3, numel(tVec)); % Pose matrix for car

pose_bike(:, 1) = initPose_bike;
pose_car(:, 1) = initPose_car;

bike.x = initPose_bike;
car.x = initPose_car;

%% Probability Road Map
[waypoints_prm, planner_prm] = PRM_planner(myMap, [2, 2], [8, 8]);

%% RRT Planner
waypoints_rrt = RRT_planner(myMap, [2, 2, 0], [8, 8, 0]);

%% Define goal positions
goal_bike = waypoints_rrt(end, :); % Last waypoint of RRT planner
goal_car = waypoints_prm(end, :); % Last waypoint of PRM planner

%% Pure Pursuit Controllers
controller_bike = controllerPurePursuit;
controller_bike.Waypoints = waypoints_rrt;
controller_bike.LookaheadDistance = 0.5;
controller_bike.DesiredLinearVelocity = 0.35;
controller_bike.MaxAngularVelocity = 1.5;

controller_car = controllerPurePursuit;
controller_car.Waypoints = waypoints_prm;
controller_car.LookaheadDistance = 0.5;
controller_car.DesiredLinearVelocity = 0.35;
controller_car.MaxAngularVelocity = 1.5;

%% Create a figure to plot the map and trajectory
figure(2);
show(planner_prm);
figure(3);
show(myMap); % Display the map
hold on;

H_bike = bike.plotv(bike.x);
H_car = car.plotv(car.x);

%% Simulation loop
r = rateControl(1/sampleTime);

for idx = 2:numel(tVec)
    % Run the Pure Pursuit controller for the bicycle and convert output to wheel speeds
    [vRef_bike, wRef_bike] = controller_bike(pose_bike(:, idx-1));

    % Compute the velocities for the bicycle
    bike.step(vRef_bike, wRef_bike);
    
    % Update pose for the bicycle
    pose_bike(:, idx) = bike.x;
    
    % Update visualization for the bicycle
    bike.plotv(H_bike, bike.x);
    
    % Run the Pure Pursuit controller for the car and convert output to wheel speeds
    [vRef_car, wRef_car] = controller_car(pose_car(:, idx-1));
    
    % Compute the velocities for the car
    car.step(vRef_car, wRef_car);
    
    % Update pose for the car
    pose_car(:, idx) = car.x;
    
    % Update visualization for the car
    car.plotv(H_car, car.x);
    
    waitfor(r);
end