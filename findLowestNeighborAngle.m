function angle = findLowestNeighborAngle(grid, currentPosition)
    % Define directions for the eight neighboring cells
    directions = [1, 0; 1, 1; 0, 1; -1, 1; -1, 0; -1, -1; 0, -1; 1, -1];

    % Initialize minimum value and angle
    minValue = Inf;
    angle = 0;

    % Get the current cell value
    currentValue = grid(currentPosition(1), currentPosition(2));

    % Iterate over the eight neighboring cells
    for i = 1:size(directions, 1)
        % Calculate the neighbor's position
        neighborPosition = currentPosition + directions(i, :);

        % Check if the neighbor position is within the grid boundaries
        if all(neighborPosition > 0) && ...
           neighborPosition(1) <= size(grid, 1) && ...
           neighborPosition(2) <= size(grid, 2)

            % Get the value of the neighbor cell
            neighborValue = grid(neighborPosition(1), neighborPosition(2));

            % Update minimum value and angle if the neighbor's value is lower than the current cell value
            if neighborValue < minValue && neighborValue < currentValue
                minValue = neighborValue;
                angle = atan2(directions(i, 2), directions(i, 1)); % Calculate angle towards the neighbor
            end
        end
    end
end