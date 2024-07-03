%% Initialize a zero matrix to represent the map
function [myBinaryMap] = Make_map(n_grid, radius)
    
    myMapSize = [n_grid, n_grid];  % Size of the map
    Map = zeros(myMapSize);
    
    % Define walls
    Map(2:4, :) = 1;               % Top wall
    Map(47:49, :) = 1;             % Bottom wall
    Map(:, 2:4) = 1;               % Left wall
    Map(:, 47:49) = 1;             % Right wall
    Map(20:30, 13) = 1;          % Middle left division
    Map(40, 20:40) = 1;          % Horizontal wall
    Map(10:15, 37:38) = 1;       % Middle right division
    Map(1:20, 25) = 1;           % Top-divider wall
    
    % Define the threshold distance for inflation
    inflationRadius = radius;
    
    % Use bwdist to calculate the distance transform
    distanceMap = bwdist(Map);
    
    % Set cells with distance less than or equal to the inflation radius to occupied
    Map = distanceMap <= inflationRadius;
    
    % Create a binary occupancy map from the logical matrix
    resolution = 5;  % Adjust the resolution as needed
    myBinaryMap = binaryOccupancyMap(Map, resolution);
    
end