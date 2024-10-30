function searchSpace = createSearchSpace(dimX, dimY, obstacleDensity, dronePositions)
% Creates the search space grid initialized as undiscovered (zeros)
% with obstacles (-1), ensuring drones' starting positions are not obstacles

    % Clamp obstacleDensity between 0 and 1
    obstacleDensity = max(0, min(1, obstacleDensity));
    
    % Initialize search space to zeros (undiscovered)
    searchSpace = zeros(dimX, dimY);
    
    % Calculate the number of obstacle cells
    numCells = dimX * dimY;
    numObstacles = round(numCells * obstacleDensity);
    
    % Total cells indices
    totalIndices = 1:numCells;
    
    % Exclude drone starting positions from obstacle placement
    droneLinearIndices = sub2ind([dimX, dimY], dronePositions(:,1), dronePositions(:,2));
    availableIndices = setdiff(totalIndices, droneLinearIndices);
    
    % Adjust number of obstacles if necessary
    maxObstacles = length(availableIndices);
    if numObstacles > maxObstacles
        warning('Obstacle density too high. Adjusting number of obstacles to maximum available.');
        numObstacles = maxObstacles;
    end
    
    % Randomly select cells to be obstacles
    obstacleIndices = availableIndices(randperm(length(availableIndices), numObstacles));
    
    % Convert linear indices to subscripts
    [obstacleX, obstacleY] = ind2sub([dimX, dimY], obstacleIndices);
    
    % Set obstacles in search space
    for i = 1:length(obstacleX)
        searchSpace(obstacleX(i), obstacleY(i)) = -1;
    end
end

function frontierCells = findFrontiers(searchSpace)
% Finds the frontier cells in the search space grid

    [dimX, dimY] = size(searchSpace);
    frontierCells = [];

    for x = 1:dimX
        for y = 1:dimY
            if searchSpace(x,y) > 0 % Discovered cell (not obstacle)
                % Check the 4-connected neighbors
                neighbors = [x+1,y; x-1,y; x,y+1; x,y-1];
                for k = 1:size(neighbors,1)
                    nx = neighbors(k,1);
                    ny = neighbors(k,2);
                    if nx >= 1 && nx <= dimX && ny >= 1 && ny <= dimY
                        if searchSpace(nx, ny) == 0 % Undiscovered neighbor (not obstacle)
                            % Add the undiscovered neighbor as a frontier cell
                            frontierCells = [frontierCells; nx, ny];
                        end
                    end
                end
            end
        end
    end
    % Remove duplicate frontier cells
    frontierCells = unique(frontierCells, 'rows');
end

function searchSpace = updateDiscoveredArea(searchSpace, dronePositions, droneIndices)
% Updates the search space grid to mark discovered areas by each drone

    for i = 1:size(dronePositions, 1)
        x = dronePositions(i,1);
        y = dronePositions(i,2);
        if searchSpace(x, y) >= 0 % Not an obstacle
            searchSpace(x, y) = droneIndices(i); % Mark the position with the drone index
        end
    end
end

function dronePositions = moveDrones(dronePositions, searchSpace)
% Moves drones towards the nearest frontier cells
% Drones can move over discovered cells (own path and others)
% Avoid obstacles and positions occupied by other drones

    numDrones = size(dronePositions, 1);
    [dimX, dimY] = size(searchSpace);

    % Find frontier cells
    frontierCells = findFrontiers(searchSpace);

    % Keep track of new positions to check for collisions
    newPositions = zeros(numDrones, 2);

    for i = 1:numDrones
        currentPos = dronePositions(i,:);

        % If no frontier cells are left, drone stays in place
        if isempty(frontierCells)
            newPositions(i,:) = currentPos;
            continue;
        end

        % Calculate Manhattan distances to all frontier cells
        distances = sum(abs(frontierCells - currentPos), 2);
        [~, idx] = min(distances);
        targetPos = frontierCells(idx,:);

        % Use a simple pathfinding approach to move towards the target
        % Allow moving over discovered cells
        % Move one step in the direction that minimizes the distance to the target

        possibleMoves = [
            currentPos + [1, 0];   % Right
            currentPos + [-1, 0];  % Left
            currentPos + [0, 1];   % Up
            currentPos + [0, -1];  % Down
        ];

        % Ensure moves are within bounds and not obstacles
        validMoves = [];
        for j = 1:size(possibleMoves,1)
            move = possibleMoves(j,:);
            if move(1) >= 1 && move(1) <= dimX && move(2) >= 1 && move(2) <= dimY
                if searchSpace(move(1), move(2)) ~= -1 % Not an obstacle
                    validMoves = [validMoves; move];
                end
            end
        end

        % Calculate distances from valid moves to the target
        if isempty(validMoves)
            newPos = currentPos; % No valid moves, stay in place
        else
            distancesToTarget = sum(abs(validMoves - targetPos), 2);
            [~, minIdx] = min(distancesToTarget);
            newPos = validMoves(minIdx,:);
        end

        % Collision avoidance
        occupiedPositions = [dronePositions(1:i-1,:); newPositions(1:i-1,:)];
        if ismember(newPos, occupiedPositions, 'rows')
            % Stay in place or pick another valid move if possible
            alternativeMoves = setdiff(validMoves, occupiedPositions, 'rows', 'stable');
            if ~isempty(alternativeMoves)
                newPos = alternativeMoves(1,:);
            else
                newPos = currentPos; % No valid moves, stay in place
            end
        end

        newPositions(i,:) = newPos;
    end

    dronePositions = newPositions;
end

function visualizeSearchSpace(searchSpace, dronePositions, numDrones)
% Visualizes the search space and drone positions

    imagesc(searchSpace');
    % Create a colormap where:
    % -1 (obstacle): gray
    %  0 (undiscovered): black
    %  1...numDrones: colors from the 'lines' colormap

    numColors = numDrones + 2; % +1 for undiscovered, +1 for obstacle
    cmap = zeros(numColors, 3);
    cmap(1,:) = [0.5 0.5 0.5]; % Obstacles (-1)
    cmap(2,:) = [0 0 0];       % Undiscovered (0)
    cmap(3:end,:) = lines(numDrones); % Drones (1...numDrones)

    colormap(cmap);
    caxis([-1 numDrones]);

    % Set up colorbar with custom tick labels
    tickValues = [-1, 0, 1:numDrones];
    tickLabels = ['Obstacle'; 'Undiscov'; arrayfun(@(x) ['Drone ' num2str(x)], 1:numDrones, 'UniformOutput', false)'];
    colorbar('Ticks', tickValues, 'TickLabels', tickLabels);

    axis equal tight;
    set(gca, 'YDir', 'normal'); % Correct the y-axis direction
    hold on;
    % Plot drone positions with unique colors
    for i = 1:size(dronePositions, 1)
        plot(dronePositions(i,1), dronePositions(i,2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, ...
            'MarkerFaceColor', cmap(i+2,:)); % i+2 because cmap starts from index 3 for drones
    end
    hold off;
    drawnow;
end

% Main Script

% Parameters
dimX = 50;            % Width of the search space
dimY = 50;            % Height of the search space
numDrones = 5;        % Number of drones
numTimeSteps = 600;   % Number of simulation steps

% Let the user define obstacle density (0 to 1)
obstacleDensity = input('Enter obstacle density (0 to 1): ');
while obstacleDensity < 0 || obstacleDensity > 1
    disp('Obstacle density must be between 0 and 1.');
    obstacleDensity = input('Enter obstacle density (0 to 1): ');
end

% Specify the initial positions of the drones
% Ensure that positions are within the bounds, unique, and not on obstacles
dronePositions = [
    1, 1;
    2, 1;
    3, 1;
    4, 1;
    50, 50
];

% Check that the number of drones matches the number of positions provided
if size(dronePositions, 1) ~= numDrones
    error('Number of drones does not match the number of specified positions.');
end

% Initialize the search space with obstacles, ensuring drones' positions are not obstacles
searchSpace = createSearchSpace(dimX, dimY, obstacleDensity, dronePositions);

% Drone indices
droneIndices = (1:numDrones)';

% Mark initial positions as discovered
searchSpace = updateDiscoveredArea(searchSpace, dronePositions, droneIndices);

% Simulation loop
for t = 1:numTimeSteps
    % Move drones towards frontier cells with collision avoidance
    dronePositions = moveDrones(dronePositions, searchSpace);

    % Update the discovered area based on drone positions
    searchSpace = updateDiscoveredArea(searchSpace, dronePositions, droneIndices);

    % Visualize the search space and drone positions
    visualizeSearchSpace(searchSpace, dronePositions, numDrones);
    pause(0.0); % Pause to control the speed of the simulation
end

% After the simulation, calculate the area covered by each drone
totalAreaCovered = zeros(numDrones,1);
% Total accessible cells (excluding obstacles)
totalCells = dimX * dimY - sum(searchSpace(:) == -1);
for i = 1:numDrones
    totalAreaCovered(i) = sum(searchSpace(:) == i);
    percentageCovered = (totalAreaCovered(i) / totalCells) * 100;
    fprintf('Drone %d covered %.2f%% of the accessible search space.\n', i, percentageCovered);
end
