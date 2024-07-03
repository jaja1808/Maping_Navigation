%% Map Description
% Size of the map
n = 50;
radius = 1.75;
myMap = Make_map(n, radius);
occupied = checkOccupancy(myMap,[5,6]);
fprintf('Occupancy : %d',occupied);

%% Bicycle Model
% Define covariance matrices
V = diag([0.02, 0.5*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);
% Create Bicycle object
veh = Bicycle('covar', V);

%% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:30;        % Time array
%disp(tVec);

%% Initial conditions
initPose = [2;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
veh.x = initPose;

%% Probability Road Map 
%[waypoints,planner] = PRM_planner(myMap,[2, 2],[8, 8]);
%show(planner);

%% RRT Planner
waypoints = RRT_planner(myMap,[2, 2, 0],[8, 8, 0]);

%% Display waypoints
disp(waypoints);

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.35;
controller.MaxAngularVelocity = 1.5;

%% Create a figure to plot the map and trajectory
figure;
show(myMap); % Display the map
hold on;
H = veh.plotv(veh.x);

%% Simulation loop
r = rateControl(1/sampleTime);

for idx = 2:numel(tVec) 
    
    %Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef, wRef] = controller(pose(:,idx-1));

    % Compute the velocities
    veh.step(vRef, wRef);

    % Update pose
    pose(:,idx) = veh.x;

    % Update visualization
    veh.plotv(H, veh.x); % Add this line to visualize the bicycle model
    waitfor(r);
end