% Complete MATLAB Code with Obstacles

% Parameters
dimX = 50;           % Width of the search space
dimY = 50;           % Height of the search space
numDrones = 5;       % Number of drones
numTimeSteps = 1000;  % Number of simulation steps

% Initialize the search space and drones
searchSpace = createSearchSpace(dimX, dimY);
[dronePositions, dronePaths] = initializeDrones(numDrones, dimX, dimY);

% Initialize drone discovery map to track which drone discovered each cell
droneDiscoveryMap = zeros(dimY, dimX);

% Add obstacles to the search space
[searchSpace, obstaclePositions] = addObstacles(searchSpace, dimX, dimY);

% Mark initial positions as discovered
[searchSpace, droneDiscoveryMap] = updateDiscoveredArea(searchSpace, dronePositions, droneDiscoveryMap);

% Simulation loop
for t = 1:numTimeSteps
    % Compute Voronoi partitioning
    voronoiMap = computeVoronoiMap(dronePositions, dimX, dimY);
    
    % Move drones towards frontier cells within their Voronoi cells
    [dronePositions, dronePaths] = moveDrones(dronePositions, searchSpace, voronoiMap, dronePaths);
    
    % Update the discovered area based on drone positions
    [searchSpace, droneDiscoveryMap] = updateDiscoveredArea(searchSpace, dronePositions, droneDiscoveryMap);
    
    % Calculate total coverage
    totalCoverage = sum(searchSpace(:) == 1) / numel(searchSpace(searchSpace ~= -1)) * 100;
    
    % Visualize the search space and drone positions
    visualizeSearchSpace(searchSpace, dronePositions, voronoiMap, dronePaths, totalCoverage);
    pause(0.1); % Pause to control the speed of the simulation
end

% Calculate coverage per drone
droneCoverage = zeros(numDrones,1);
for i = 1:numDrones
    droneCoverage(i) = sum(droneDiscoveryMap(:) == i);
end
droneCoveragePercent = (droneCoverage / numel(searchSpace(searchSpace ~= -1))) * 100;

% Final visualization with each drone's percentage coverage
visualizeFinalPaths(dronePositions, dronePaths, droneCoveragePercent);

%% Function Definitions

function searchSpace = createSearchSpace(dimX, dimY)
% Creates the search space grid initialized as undiscovered (zeros)

    searchSpace = zeros(dimY, dimX);
end

function [searchSpace, obstaclePositions] = addObstacles(searchSpace, dimX, dimY)
% Adds obstacles to the search space
% Obstacles are represented by -1 in the searchSpace matrix

    obstaclePositions = [];
    numObstacles = 100; % Adjust the number of obstacles as needed
    rng(0); % Seed for reproducibility

    for i = 1:numObstacles
        x = randi([1, dimX]);
        y = randi([1, dimY]);
        searchSpace(y, x) = -1; % Mark the position as an obstacle
        obstaclePositions = [obstaclePositions; x, y];
    end
end

function [dronePositions, dronePaths] = initializeDrones(numDrones, dimX, dimY)
% Initializes drone positions with 3 drones next to each other and 2 drones away
% Outputs drone positions and initializes drone paths

    positions = zeros(numDrones, 2);
    paths = cell(numDrones,1);

    % Define positions for drones
    % Place the first 3 drones next to each other
    positions(1, :) = [10, 1]; % Drone 1
    positions(2, :) = [11, 1]; % Drone 2
    positions(3, :) = [12, 1]; % Drone 3

    % Place the remaining drones away from the first group
    positions(4, :) = [40, 1]; % Drone 4
    positions(5, :) = [41, 1]; % Drone 5

    % Initialize paths for the drones
    for i = 1:numDrones
        paths{i} = positions(i,:);   % Initialize path
    end

    % Ensure positions are integers within bounds
    positions = round(positions);
    positions(:,1) = max(min(positions(:,1), dimX), 1);
    positions(:,2) = max(min(positions(:,2), dimY), 1);

    dronePositions = positions;
    dronePaths = paths;
end

function [searchSpace, droneDiscoveryMap] = updateDiscoveredArea(searchSpace, dronePositions, droneDiscoveryMap)
% Updates the search space grid to mark discovered areas
% Also updates the droneDiscoveryMap to track which drone discovered each cell

    for i = 1:size(dronePositions, 1)
        x = dronePositions(i,1);
        y = dronePositions(i,2);
        if searchSpace(y, x) == 0
            searchSpace(y, x) = 1; % Mark the position as discovered
            droneDiscoveryMap(y, x) = i; % Record the drone index
        end
    end
end

function voronoiMap = computeVoronoiMap(dronePositions, dimX, dimY)
% Computes the Voronoi partitioning of the search space

    [X, Y] = meshgrid(1:dimX, 1:dimY); % X: columns, Y: rows
    numDrones = size(dronePositions, 1);
    voronoiMap = zeros(dimY, dimX);

    % Calculate distances from each cell to each drone
    for i = 1:numDrones
        dx = X - dronePositions(i,1);
        dy = Y - dronePositions(i,2);
        distances = dx.^2 + dy.^2; % Use squared Euclidean distance
        if i == 1
            minDistances = distances;
            voronoiMap(:) = i;
        else
            mask = distances < minDistances;
            voronoiMap(mask) = i;
            minDistances(mask) = distances(mask);
        end
    end
end

function frontierCellsPerDrone = findFrontiersPerDrone(searchSpace, voronoiMap, numDrones)
% Finds frontier cells within each drone's Voronoi cell

    [dimY, dimX] = size(searchSpace);
    frontierCellsPerDrone = cell(numDrones, 1);

    for y = 1:dimY
        for x = 1:dimX
            if searchSpace(y,x) == 1 % Discovered cell
                droneIdx = voronoiMap(y,x);
                % Check the 4-connected neighbors
                neighbors = [x+1,y; x-1,y; x,y+1; x,y-1];
                for k = 1:size(neighbors,1)
                    nx = neighbors(k,1);
                    ny = neighbors(k,2);
                    if nx >= 1 && nx <= dimX && ny >= 1 && ny <= dimY
                        if searchSpace(ny, nx) == 0 && voronoiMap(ny, nx) == droneIdx
                            % Add the undiscovered neighbor as a frontier cell
                            frontierCellsPerDrone{droneIdx} = [frontierCellsPerDrone{droneIdx}; nx, ny];
                        end
                    end
                end
            end
        end
    end

    % Remove duplicate frontier cells for each drone
    for i = 1:numDrones
        if ~isempty(frontierCellsPerDrone{i})
            frontierCellsPerDrone{i} = unique(frontierCellsPerDrone{i}, 'rows');
        end
    end
end

function [dronePositions, dronePaths] = moveDrones(dronePositions, searchSpace, voronoiMap, dronePaths)
% Moves drones towards the nearest frontier cells within their Voronoi cells

    numDrones = size(dronePositions, 1);
    [dimY, dimX] = size(searchSpace);

    % Find frontier cells within each drone's Voronoi cell
    frontierCellsPerDrone = findFrontiersPerDrone(searchSpace, voronoiMap, numDrones);

    % Keep track of new positions to check for collisions
    newPositions = zeros(numDrones, 2);

    for i = 1:numDrones
        currentPos = dronePositions(i,:);

        frontierCells = frontierCellsPerDrone{i};

        % If no frontier cells are left in the Voronoi cell, drone stays in place
        if isempty(frontierCells)
            newPositions(i,:) = currentPos;
            continue;
        end

        % Calculate Manhattan distances to all frontier cells
        distances = sum(abs(frontierCells - currentPos), 2);
        [~, idx] = min(distances);
        targetPos = frontierCells(idx,:);

        % Determine movement direction
        direction = targetPos - currentPos;
        step = sign(direction);

        % Move one step towards the target
        newPos = currentPos + step;

        % Ensure drones stay within the bounds
        newPos(1) = max(min(newPos(1), dimX), 1);
        newPos(2) = max(min(newPos(2), dimY), 1);

        % Check if the new position is an obstacle
        if searchSpace(newPos(2), newPos(1)) == -1
            % Try alternative moves: stay in place or move orthogonally
            alternativeMoves = [
                currentPos; % Stay in place
                currentPos + [step(1), 0];
                currentPos + [0, step(2)];
                currentPos - [step(1), 0];
                currentPos - [0, step(2)];
            ];

            % Ensure moves are within bounds
            alternativeMoves(:,1) = max(min(alternativeMoves(:,1), dimX), 1);
            alternativeMoves(:,2) = max(min(alternativeMoves(:,2), dimY), 1);

            % Remove positions that are obstacles or outside Voronoi cell
            validMoves = [];
            for j = 1:size(alternativeMoves,1)
                pos = alternativeMoves(j,:);
                if searchSpace(pos(2), pos(1)) ~= -1 && voronoiMap(pos(2), pos(1)) == i
                    validMoves = [validMoves; pos];
                end
            end

            if ~isempty(validMoves)
                newPos = validMoves(1,:); % Choose the first valid move
            else
                newPos = currentPos; % No valid moves, stay in place
            end
        end

        % Collision avoidance
        % Check if the new position is already taken by another drone
        occupiedPositions = [dronePositions(1:i-1,:); newPositions(1:i-1,:)];
        if ismember(newPos, occupiedPositions, 'rows')
            % Try alternative moves: stay in place or move orthogonally
            alternativeMoves = [
                currentPos; % Stay in place
                currentPos + [step(1), 0];
                currentPos + [0, step(2)];
                currentPos - [step(1), 0];
                currentPos - [0, step(2)];
            ];

            % Ensure moves are within bounds
            alternativeMoves(:,1) = max(min(alternativeMoves(:,1), dimX), 1);
            alternativeMoves(:,2) = max(min(alternativeMoves(:,2), dimY), 1);

            % Remove positions that are obstacles, already occupied, or outside Voronoi cell
            validMoves = [];
            for j = 1:size(alternativeMoves,1)
                pos = alternativeMoves(j,:);
                if ~ismember(pos, occupiedPositions, 'rows') && ...
                   searchSpace(pos(2), pos(1)) ~= -1 && ...
                   voronoiMap(pos(2), pos(1)) == i
                    validMoves = [validMoves; pos];
                end
            end

            if ~isempty(validMoves)
                newPos = validMoves(1,:); % Choose the first valid move
            else
                newPos = currentPos; % No valid moves, stay in place
            end
        end

        newPositions(i,:) = newPos;
        % Update path
        dronePaths{i} = [dronePaths{i}; newPos];
    end

    dronePositions = newPositions;
end

function visualizeSearchSpace(searchSpace, dronePositions, voronoiMap, dronePaths, totalCoverage)
% Visualizes the search space, drone positions, obstacles, and Voronoi cells

    % Create a colored map for Voronoi cells
    voronoiColors = lines(max(voronoiMap(:))); % Generate distinguishable colors
    [dimY, dimX] = size(searchSpace);
    coloredMap = zeros(dimY, dimX, 3);
    for y = 1:dimY
        for x = 1:dimX
            if searchSpace(y,x) == 1
                colorIdx = voronoiMap(y,x);
                coloredMap(y,x,:) = voronoiColors(colorIdx, :);
            elseif searchSpace(y,x) == -1
                coloredMap(y,x,:) = [0, 0, 0]; % Obstacles are black
            else
                coloredMap(y,x,:) = 0.5; % Undiscovered areas are gray
            end
        end
    end

    image(coloredMap);
    axis equal tight;     % Equal scaling and tight axis limits
    set(gca, 'YDir', 'normal'); % Correct the y-axis direction
    hold on;

    % Plot drone positions and paths
    for i = 1:size(dronePositions,1)
        % Plot drone path
        path = dronePaths{i};
        plot(path(:,1), path(:,2), '-', 'Color', voronoiColors(i,:), 'LineWidth', 1.5);

        % Plot drone position
        plot(dronePositions(i,1), dronePositions(i,2), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'w');

        % Draw circle around drone
        radius = 5; % Adjust radius as needed
        pos = [dronePositions(i,1), dronePositions(i,2)];
        rectangle('Position',[pos(1)-radius,pos(2)-radius,2*radius,2*radius],'Curvature',[1,1],...
            'EdgeColor',voronoiColors(i,:),'LineWidth',1.5);
    end
    hold off;

    % Display total coverage
    title(sprintf('Total Coverage: %.2f%%', totalCoverage));
    drawnow;
end

function visualizeFinalPaths(dronePositions, dronePaths, droneCoveragePercent)
% Visualizes the drones' paths and displays each drone's percentage coverage

    figure;
    axis equal;
    axis([0, max(dronePositions(:,1))+10, 0, max(dronePositions(:,2))+10]);
    hold on;
    grid on;

    % Generate colors for drones
    numDrones = size(dronePositions,1);
    droneColors = lines(numDrones);

    % Plot drone positions and paths
    for i = 1:numDrones
        % Plot drone path
        path = dronePaths{i};
        plot(path(:,1), path(:,2), '-', 'Color', droneColors(i,:), 'LineWidth', 1.5);

        % Plot drone position
        plot(dronePositions(i,1), dronePositions(i,2), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'w');

        % Draw circle around drone
        radius = 5; % Adjust radius as needed
        pos = [dronePositions(i,1), dronePositions(i,2)];
        rectangle('Position',[pos(1)-radius,pos(2)-radius,2*radius,2*radius],'Curvature',[1,1],...
            'EdgeColor',droneColors(i,:),'LineWidth',1.5);

        % Display percentage coverage
        coverageText = sprintf('Drone %d: %.2f%%', i, droneCoveragePercent(i));
        text(pos(1), pos(2)+radius+2, coverageText, 'HorizontalAlignment', 'center', 'Color', droneColors(i,:), 'FontWeight', 'bold');
    end
    hold off;
    title('Drone Paths and Positions with Coverage Percentages');
end
