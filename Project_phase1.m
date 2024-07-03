%% Map Description
% Size of the map
n = 50;
radius = 1;
Map = Make_map(n, radius);
occupancy = checkOccupancy(Map,[4, 2]);
fprintf('Occupancy : %d',occupancy);

%% Define covariance matrices
V = diag([0.02, 0.5*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);
startPose = [2; 2; 0.2];

%% Create Bicycle object
bike = Bicycle('speedmax',5,'steermax',5,'covar', V);
bike.add_driver(RandomPath(10));
bike.x = startPose;

%% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:100;        % Time array
% Initial condition
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = startPose;

%% Create a figure to plot the map and trajectory
figure;
show(Map); % Display the map
hold on;
H = bike.plotv(bike.x);

%% Simulation loop
r = rateControl(1/sampleTime);

for idx = 2:numel(tVec) 
    
    [obstacles, scanPoints] = Scan(Map, bike);  % Scan for obstacles
    % Initial values of speed and steerAngle
    vRef = 0.6;
    wRef = 0;

    % If obstacles are detected, turn by a fixed angle
    if ~isempty(obstacles)
      
        % Drive function
        [distance, angle] = drive(bike.x, obstacles);
        % Displays
        fprintf('my X position %.2f \n',bike.x(1));
        fprintf('my Y position %.2f \n',bike.x(2));
      
        % Close to the obstacle
        [vRef,wRef] = navigate(distance, angle);
        % Move the robot
        bike.step(vRef, wRef);
    else
        % Move the robot
        bike.step(vRef, wRef);
    end
    
    % Update pose
    pose(:,idx) = bike.x; % Update the position
    p = plot(scanPoints(:,1), scanPoints(:,2), '.g'); % visualize the scanner
    % Update visualization
    bike.plotv(H, bike.x); % Add this line to visualize the bicycle model
    waitfor(r); % delay
    delete(p); % delete the previous scanner visualization
end
