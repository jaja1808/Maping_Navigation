function [obstacles, scanPoints] = Scan(map, veh)
    obstacles = [];
    scanPoints = [];

    x = veh.x(1);
    y = veh.x(2);
    theta = veh.x(3);  % Assuming this is in radians
    range = 10;  % Max range to scan
    cover = 20;  % Coverage angle in degrees
    resolution = map.Resolution;  % Assuming your map has a Resolution property

    % Map boundaries
    xmin = map.XWorldLimits(1);
    xmax = map.XWorldLimits(2);
    ymin = map.YWorldLimits(1);
    ymax = map.YWorldLimits(2);

    for angle_inc = linspace(-cover/2, cover/2, 5)
        rayAngle = theta + deg2rad(angle_inc);

        for dist = 0:1/resolution:range/resolution
            x_point = x + dist * cos(rayAngle);
            y_point = y + dist * sin(rayAngle);

            % Check if the scan point is within the map boundaries
            if x_point >= xmin && x_point <= xmax && y_point >= ymin && y_point <= ymax
                scanPoints = [scanPoints; [x_point, y_point]];

                if checkOccupancy(map, [x_point, y_point])
                    obstacles = [obstacles; [x_point, y_point]];  % Store relative angles
                    break;
                end
            end
        end
    end
end