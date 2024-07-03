function [waypoints, planners]= PRM_planner(map, startPoint, goalPoint)
    % Create a Probabilistic Road Map (PRM)
    planner = mobileRobotPRM(map);
    planner.NumNodes = 75;
    planner.ConnectionDistance = 5;
    
    % Check if the start and goal points are within the map boundaries
    if any(startPoint < 0) || any(startPoint > 10)
        error('Start point is outside the map boundaries.');
    end
    
    if any(goalPoint < 0) || any(goalPoint > 10)
        error('Goal point is outside the map boundaries.');
    end
    
    % Find a path from the start point to the goal point
    waypoints = findpath(planner, startPoint, goalPoint);
    % Shift the waypoints slightly to account for the bicycle model size
    waypoints = waypoints + repmat([0.25, 0.25], size(waypoints, 1), 1);

    %plot the planner
    planners = planner;
end