function [linear, angular] = drive(position, obstacles)
    % Vehicle position
    x_veh = position(1);
    y_veh = position(2);
    
    distances = [];
    angles = [];
    
    for i = 1:(numel(obstacles)/2)

        % Obstacle position
        x_obs = obstacles(i,1);
        y_obs = obstacles(i,2);

        % finding the orientation
        x = x_obs - x_veh;
        y = y_obs - y_veh;
    
        % Calculate distance
        linear_dist = sqrt(x^2 + y^2);
        % Angle
        angle = atan2(y,x);
        
        % Append the distance and angle calculated
        distances = [distances, linear_dist];
        angles = [angles, angle];

    end 
    
    % Linear distance and its index
    [linear,idx] = min(distances);
    % Angular speed
    angular = angles(idx);