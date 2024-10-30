function newState = updateDroneState(state, controlInput, dt)
    % Define how each drone's state changes based on its control inputs.
    newState = state; % Copy the current state
    newState(1) = state(1) + controlInput(1) * cos(state(3)) * dt; % Update x position
    newState(2) = state(2) + controlInput(1) * sin(state(3)) * dt; % Update y position
    newState(3) = state(3) + controlInput(2) * dt; % Update orientation
end