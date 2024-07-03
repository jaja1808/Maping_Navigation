function [waypoints] = RRT_planner(map, start, goal) 

    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    
    sv.Map = map;
    
    sv.ValidationDistance = 0.01;
    
    ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
    planner = plannerRRT(ss,sv,MaxConnectionDistance=0.3);
    
    rng(100,'twister'); % for repeatable result
    [pthObj,solnInfo] = plan(planner,start,goal);
    
    figure(1);
    show(map)
    
    hold on
    % Tree expansion
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
    % Draw path
    plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)
    
    % Extract states from the path object
    path_states = pthObj.States;
    
    % Initialize an empty list for waypoints
    waypoints = [];
    
    % Convert each state to a waypoint
    for i = 1:size(path_states, 1)
        x = path_states(i, 1);
        y = path_states(i, 2);
        theta = path_states(i, 3);  % Heading angle (orientation)
    
        % Append the waypoint to the list
        waypoints = [waypoints; x, y];
    end
end 