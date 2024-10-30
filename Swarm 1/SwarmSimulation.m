% Number of drones
numDrones = 5;
dt = 0.1; % Time step for simulation

% Initialize drone states and control inputs
drones = struct('state', {}, 'control', {});
for i = 1:numDrones
    drones(i).state = [rand()*10; rand()*10; rand()*2*pi]; % Random position and orientation
    drones(i).control = [1; 0.1]; % Constant speed and rotation
end

% Run the simulation for 100 steps
for t = 1:100
    % Get all drone positions
    positions = cell2mat(arrayfun(@(d) d.state(1:2)', drones, 'UniformOutput', false)');
    
    % Generate Voronoi diagram for coverage
    [V, C] = voronoin(positions); % Calculate Voronoi regions
    
    % Update each drone
    for i = 1:numDrones
        region = V(C{i}, :); % Get region points for drone i
        centroid = mean(region, 1, 'omitnan'); % Calculate region centroid
        
        % Move drone towards its centroid
        drones(i).control = [0.5; atan2(centroid(2)-drones(i).state(2), centroid(1)-drones(i).state(1))];
        drones(i).state = updateDroneState(drones(i).state, drones(i).control, dt);
    end
    
    % Plot the swarm and their regions
    clf; hold on;
    for i = 1:numDrones
        region = V(C{i}, :);
        fill(region(:,1), region(:,2), rand(1,3), 'FaceAlpha', 0.3); % Color region
        plot(drones(i).state(1), drones(i).state(2), 'ro'); % Mark drone position
    end
    pause(0.1); % Wait a moment to update the plot
end

