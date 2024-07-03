function [vRef,wRef] = navigate(distance,angle)
    % Close to the obstacle
    if distance <= 1
    % Compute the velocities
    vRef = - 0.3 * distance;
    wRef = 6 * randn;

    else
    % Compute the velocities
    vRef = 0.3 * distance;
    wRef = 5 * - angle;
    end
end