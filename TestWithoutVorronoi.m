% Map size
mapSize = [50, 50];

% Initialize map with unknowns
map = zeros(mapSize);

% Add some random obstacles
numObstacles = 100;
for i = 1:numObstacles
    x = randi(mapSize(1));
    y = randi(mapSize(2));
    map(x, y) = -1;  % Obstacle
end

numDrones = 5;
drones = struct('position', {}, 'path', {});

% Starting positions
startPositions = [ ...
    5, 5;
    5, 45;
    45, 5;
    45, 45;
    25, 25];

for i = 1:numDrones
    drones(i).position = startPositions(i, :);
    drones(i).path = drones(i).position;
end

function frontiers = detectFrontiers(map)
    [rows, cols] = size(map);
    frontiers = [];
    for x = 2:rows-1
        for y = 2:cols-1
            if map(x, y) == 1  % Free space
                neighbors = map(x-1:x+1, y-1:y+1);
                if any(neighbors(:) == 0)  % Adjacent to unknown
                    frontiers = [frontiers; x, y];
                end
            end
        end
    end
end

maxIterations = 500;
for t = 1:maxIterations
    % Update map based on drone sensors (simulate sensing)
    for i = 1:numDrones
        pos = drones(i).position;
        x = pos(1);
        y = pos(2);
        % Simulate sensing in a 3x3 neighborhood
        xRange = max(1, x-1):min(mapSize(1), x+1);
        yRange = max(1, y-1):min(mapSize(2), y+1);
        for xi = xRange
            for yi = yRange
                if map(xi, yi) ~= -1  % Not an obstacle
                    map(xi, yi) = 1;  % Mark as free space
                end
            end
        end
    end
    
    % Detect frontiers
    frontiers = detectFrontiers(map);
    if isempty(frontiers)
        disp('Exploration complete.');
        break;
    end
    
    % Assign frontiers to drones
    for i = 1:numDrones
        % Compute distances to all frontiers
        distances = sqrt(sum((frontiers - drones(i).position).^2, 2));
        % Select the nearest frontier
        [~, idx] = min(distances);
        target = frontiers(idx, :);
        
        % Move towards the target
        direction = target - drones(i).position;
        if norm(direction) > 0
            step = direction / norm(direction);  % Unit vector
            newPos = drones(i).position + step;
            newPos = round(newPos);
            
            % Collision avoidance with obstacles
            if map(newPos(1), newPos(2)) ~= -1
                % Check for collision with other drones
                collision = false;
                for j = 1:numDrones
                    if j ~= i && isequal(drones(j).position, newPos)
                        collision = true;
                        break;
                    end
                end
                if ~collision
                    drones(i).position = newPos;
                    drones(i).path = [drones(i).path; newPos];
                end
            end
        end
    end
    
    % Visualization
    imagesc(map);
    colormap(gray);
    hold on;
    for i = 1:numDrones
        plot(drones(i).path(:,2), drones(i).path(:,1), 'LineWidth', 2);
        plot(drones(i).position(2), drones(i).position(1), 'ro', 'MarkerSize', 10);
    end
    hold off;
    drawnow;
end

% Inside the movement loop, replace the collision check with:

% Predict next positions
nextPositions = zeros(numDrones, 2);
for i = 1:numDrones
    % Compute as before
    direction = target - drones(i).position;
    if norm(direction) > 0
        step = direction / norm(direction);
        nextPositions(i, :) = round(drones(i).position + step);
    else
        nextPositions(i, :) = drones(i).position;
    end
end

% Check for collisions among drones
for i = 1:numDrones
    newPos = nextPositions(i, :);
    collision = false;
    for j = 1:numDrones
        if j ~= i && isequal(newPos, nextPositions(j, :))
            collision = true;
            break;
        end
    end
    if ~collision && map(newPos(1), newPos(2)) ~= -1
        drones(i).position = newPos;
        drones(i).path = [drones(i).path; newPos];
    end
end

