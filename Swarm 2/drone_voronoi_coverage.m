% Parameters
numDrones = 15;             % Number of drones
envWidth = 100;             % Width of the 2D environment
envHeight = 100;            % Height of the 2D environment
speed = 1;                  % Speed of drones in random directions
repelRadius = 10;           % Radius to detect nearby drones for repelling
gridSize = 5;               % Size of grid cells for coverage tracking

% Initialize drone positions and directions
dronePositions = [envWidth * rand(numDrones, 1), 0.1 * rand(numDrones, 1)];
angles = 2 * pi * rand(numDrones, 1); % Initial random angles for each drone

% Create grid for coverage tracking
xGrid = 0:gridSize:envWidth;
yGrid = 0:gridSize:envHeight;
coverageGrid = zeros(length(yGrid), length(xGrid)); % Grid to track visited cells

% Visualization setup
figure;
axis([0 envWidth 0 envHeight]);
axis equal;
title('Enhanced Real-Time Voronoi Tessellation for Drone Coverage');
xlabel('X Position');
ylabel('Y Position');
hold on;

% Color map for Voronoi cells
colors = lines(numDrones); % Random colors for each drone's region

% Real-time simulation loop
while true
    % Update drone positions and repel from high-density areas
    for i = 1:numDrones
        % Calculate nearby drones within repelRadius
        distances = vecnorm(dronePositions - dronePositions(i,:), 2, 2);
        neighbors = (distances < repelRadius) & (distances > 0); % Exclude self
        
        % Adjust angle away from neighbors to avoid clustering
        if any(neighbors)
            directionAway = mean(dronePositions(neighbors,:) - dronePositions(i,:), 1);
            angles(i) = atan2(-directionAway(2), -directionAway(1)); % Move away
        end
        
        % Update position in the direction of the angle
        dronePositions(i,1) = dronePositions(i,1) + speed * cos(angles(i));
        dronePositions(i,2) = dronePositions(i,2) + speed * sin(angles(i));
    end
    
    % Keep drones within bounds by reflecting at boundaries
    for i = 1:numDrones
        if dronePositions(i,1) <= 0 || dronePositions(i,1) >= envWidth
            angles(i) = pi - angles(i); % Reflect horizontally
        end
        if dronePositions(i,2) <= 0 || dronePositions(i,2) >= envHeight
            angles(i) = -angles(i); % Reflect vertically
        end
    end
    
    % Track coverage on the grid
    for i = 1:numDrones
        xIdx = max(1, min(length(xGrid)-1, floor(dronePositions(i,1) / gridSize) + 1));
        yIdx = max(1, min(length(yGrid)-1, floor(dronePositions(i,2) / gridSize) + 1));
        coverageGrid(yIdx, xIdx) = 1; % Mark grid cell as covered
    end
    
    % Calculate coverage percentage
    coveragePercent = 100 * sum(coverageGrid(:)) / numel(coverageGrid);
    
    % Clear previous Voronoi diagram and grid display
    clf;
    axis([0 envWidth 0 envHeight]);
    title(['Enhanced Real-Time Voronoi Coverage - Coverage: ', num2str(coveragePercent, '%.2f'), '%']);
    xlabel('X Position');
    ylabel('Y Position');
    hold on;
    
    % Plot Voronoi tessellation for current positions
    [V, C] = voronoin(dronePositions);
    
    % Draw each Voronoi cell with a color and the drone position
    for i = 1:numDrones
        if all(C{i} ~= 1)  % Exclude unbounded cells
            % Get vertices of the Voronoi cell
            cellVertices = V(C{i}, :);
            
            % Fill the Voronoi cell with color
            patch(cellVertices(:,1), cellVertices(:,2), colors(i,:), ...
                'FaceAlpha', 0.5, 'EdgeColor', 'none');
            
            % Plot drone position as a black dot
            plot(dronePositions(i,1), dronePositions(i,2), 'ko', ...
                'MarkerFaceColor', 'k', 'MarkerSize', 5);
        end
    end
    
    % Overlay grid cells to show coverage
    for x = 1:length(xGrid)-1
        for y = 1:length(yGrid)-1
            if coverageGrid(y, x) == 1
                rectangle('Position', [xGrid(x), yGrid(y), gridSize, gridSize], ...
                          'FaceColor', [0, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
            end
        end
    end
    
    % Update display
    drawnow;
    
    % Break condition if coverage reaches a threshold (e.g., 95%)
    if coveragePercent >= 95
        disp('Coverage threshold reached!');
        break;
    end
    
    % Small pause to control the animation speed
    pause(0.1);
end

hold off;
