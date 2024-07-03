%% Map Description
% Size of the map
n = 50;
radius = 1;
Map = Make_map(n, radius);
occupancy = checkOccupancy(Map, [4, 2]);
fprintf('Occupancy : %d', occupancy);

%% Define covariance matrices
V_bike = diag([0.02, 0.5*pi/180].^2);
V_car = diag([0.03, 0.6*pi/180].^2);
W = diag([0.1, 1*pi/180].^2);

startPose_bike = [2; 2; 0.2];
startPose_car = [4; 4; 0.4];

%% Create Bicycle objects
bike = Bicycle('speedmax', 5, 'steermax', 5, 'covar', V_bike);
bike.add_driver(RandomPath(10));
bike.x = startPose_bike;

car = Bicycle('speedmax', 5, 'steermax', 5, 'covar', V_car);
car.add_driver(RandomPath(10));
car.x = startPose_car;

%% Sample time and time array
sampleTime = 0.1; % Sample time [s]
tVec = 0:sampleTime:40; % Time array

% Initial condition
pose_bike = zeros(3, numel(tVec)); % Pose matrix for bike
pose_car = zeros(3, numel(tVec)); % Pose matrix for car

pose_bike(:, 1) = startPose_bike;
pose_car(:, 1) = startPose_car;

%% Create a figure to plot the map and trajectory
figure;
show(Map); % Display the map
hold on;

H_bike = bike.plotv(bike.x);
H_car = car.plotv(car.x);

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)

    % Scan for obstacles
    [obstacles_bike, scanPoints_bike] = Scan(Map, bike); % Scan for obstacles for bike
    [obstacles_car, scanPoints_car] = Scan(Map, car); % Scan for obstacles for car

    % Initial values of speed and steerAngle
    vRef_bike = 0.6;
    wRef_bike = 0;
    vRef_car = 0.6;
    wRef_car = 0;

    % If obstacles are detected, turn by a fixed angle
    if ~isempty(obstacles_bike)
        % Drive function
        [distance_bike, angle_bike] = drive(bike.x, obstacles_bike);
        fprintf('Bike X position %.2f \n', bike.x(1));
        fprintf('Bike Y position %.2f \n', bike.x(2));
        % Close to the obstacle
        [vRef_bike, wRef_bike] = navigate(distance_bike, angle_bike);
    end

    if ~isempty(obstacles_car)
        % Drive function
        [distance_car, angle_car] = drive(car.x, obstacles_car);
        fprintf('Car X position %.2f \n', car.x(1));
        fprintf('Car Y position %.2f \n', car.x(2));
        % Close to the obstacle
        [vRef_car, wRef_car] = navigate(distance_car, angle_car);
    end

    % Move the robots
    bike.step(vRef_bike, wRef_bike);
    car.step(vRef_car, wRef_car);

    % Update pose
    pose_bike(:, idx) = bike.x; % Update the position for bike
    pose_car(:, idx) = car.x; % Update the position for car

    % Visualize the scanners
    p_bike = plot(scanPoints_bike(:, 1), scanPoints_bike(:, 2), '.g'); % visualize the scanner for bike
    p_car = plot(scanPoints_car(:, 1), scanPoints_car(:, 2), '.r'); % visualize the scanner for car

    % Update visualization
    bike.plotv(H_bike, bike.x); % Add this line to visualize the bicycle model for bike
    car.plotv(H_car, car.x); % Add this line to visualize the bicycle model for car

    waitfor(r); % delay
    delete(p_bike); % delete the previous scanner visualization for bike
    delete(p_car); % delete the previous scanner visualization for car
end