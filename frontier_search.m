function searchSpace = createSearchSpace(dimX, dimY)
% Creates the search space grid initialized as undiscovered (zeros)
% Adjusted dimensions to (dimY, dimX)

    searchSpace = zeros(dimY, dimX);
end


function dronePositions = initializeDrones(numDrones, dimX, xSpacing)
% Initializes drone positions along the x-axis with specified spacing
% Positions are in (x, y) format

    positions = zeros(numDrones, 2);
    xPositions = linspace(1, dimX, numDrones * xSpacing);
    xIndices = round(linspace(1, length(xPositions), numDrones));
    for i = 1:numDrones
        xPos = xPositions(xIndices(i));
        positions(i, :) = [xPos, 1]; % Place drones along y = 1
    end
    % Ensure positions are integers within bounds
    positions = round(positions);
    positions(:,1) = max(min(positions(:,1), dimX), 1);
    dronePositions = positions;
end


function voronoiMap = computeVoronoiMap(dronePositions, dimX, dimY)
% Computes the Voronoi partitioning of the search space
% Returns a map where each cell contains the index of the nearest drone
% Dimensions adjusted to (dimY, dimX)

    [X, Y] = meshgrid(1:dimX, 1:dimY); % X: columns, Y: rows
    numDrones = size(dronePositions, 1);
    voronoiMap = zeros(dimY, dimX); % Dimensions correspond to searchSpace
    
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



function dronePositions = moveDrones(dronePositions, searchSpace, voronoiMap)
% Moves drones towards the nearest frontier cells within their Voronoi cells
% Includes collision avoidance to prevent drones from occupying the same cell

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

            % Remove positions already occupied and outside Voronoi cell
            validMoves = [];
            for j = 1:size(alternativeMoves,1)
                pos = alternativeMoves(j,:);
                if ~ismember(pos, occupiedPositions, 'rows') && ...
                   voronoiMap(pos(2), pos(1)) == i % Accessed as (y, x)
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
    end

    dronePositions = newPositions;
end



function searchSpace = updateDiscoveredArea(searchSpace, dronePositions)
% Updates the search space grid to mark discovered areas
% Accessed as searchSpace(y, x)

    for i = 1:size(dronePositions, 1)
        x = dronePositions(i,1);
        y = dronePositions(i,2);
        searchSpace(y, x) = 1; % Mark the position as discovered
    end
end



function visualizeSearchSpace(searchSpace, dronePositions, voronoiMap)
% Visualizes the search space, drone positions, and Voronoi cells
% Adjusted indexing to (y, x)

    % Create a colored map for Voronoi cells
    voronoiColors = lines(max(voronoiMap(:))); % Generate distinguishable colors
    [dimY, dimX] = size(searchSpace);
    coloredMap = zeros(dimY, dimX, 3);
    for y = 1:dimY
        for x = 1:dimX
            if searchSpace(y,x) == 1
                colorIdx = voronoiMap(y,x);
                coloredMap(y,x,:) = voronoiColors(colorIdx, :);
            else
                coloredMap(y,x,:) = 0; % Undiscovered areas are black
            end
        end
    end

    image(coloredMap);
    axis equal tight;     % Equal scaling and tight axis limits
    set(gca, 'YDir', 'normal'); % Correct the y-axis direction
    hold on;
    % Plot drone positions
    plot(dronePositions(:,1), dronePositions(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'w');
    hold off;
    drawnow;
end



function frontierCellsPerDrone = findFrontiersPerDrone(searchSpace, voronoiMap, numDrones)
% Finds frontier cells within each drone's Voronoi cell
% Adjusted indexing to (y, x)

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
                            % Add the undiscovered neighbor as a frontier cell for this drone
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



%{
function frontierCells = findFrontiers(searchSpace)
% Finds the frontier cells in the search space grid

    [dimX, dimY] = size(searchSpace);
    frontierCells = [];

    for x = 1:dimX
        for y = 1:dimY
            if searchSpace(x,y) == 1 % Discovered cell
                % Check the 4-connected neighbors
                neighbors = [x+1,y; x-1,y; x,y+1; x,y-1];
                for k = 1:size(neighbors,1)
                    nx = neighbors(k,1);
                    ny = neighbors(k,2);
                    if nx >= 1 && nx <= dimX && ny >= 1 && ny <= dimY
                        if searchSpace(nx, ny) == 0 % Undiscovered neighbor
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
%}


% Main Script

% Parameters
dimX = 50;           % Width of the search space
dimY = 50;           % Height of the search space
numDrones = 5;       % Number of drones
numTimeSteps = 500;  % Number of simulation steps
xSpacing = 10;       % Spacing between drones along the x-axis

% Initialize the search space and drones
searchSpace = createSearchSpace(dimX, dimY);
dronePositions = initializeDrones(numDrones, dimX, xSpacing);

% Mark initial positions as discovered
searchSpace = updateDiscoveredArea(searchSpace, dronePositions);

% Simulation loop
for t = 1:numTimeSteps
    % Compute Voronoi partitioning
    voronoiMap = computeVoronoiMap(dronePositions, dimX, dimY);
    
    % Move drones towards frontier cells within their Voronoi cells
    dronePositions = moveDrones(dronePositions, searchSpace, voronoiMap);
    
    % Update the discovered area based on drone positions
    searchSpace = updateDiscoveredArea(searchSpace, dronePositions);
    
    % Visualize the search space and drone positions
    visualizeSearchSpace(searchSpace, dronePositions, voronoiMap);
    pause(0.1); % Pause to control the speed of the simulation
end




