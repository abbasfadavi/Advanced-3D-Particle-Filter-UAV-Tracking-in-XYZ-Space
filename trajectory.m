function [z,x_true] = trajectory(state_dim,T,Q_pos,Q_vel,dt,R)
x_true = zeros(state_dim, T);
x_true(:,1) = [0; 0; 10; 1; 0; 0];  % Start at (0,0,10) with velocity in X direction
%% Generate True Trajectory (UAV Maneuver)
for k = 2:T
    % Variable accelerations for maneuver simulation
    if k < 25
        acceleration = [0.1; 0.05; 0.02];   % Gentle acceleration
    elseif k < 50
        acceleration = [0.05; 0.1; -0.1];   % Direction change
    elseif k < 75
        acceleration = [-0.1; 0.05; 0.05];  % Turning
    else
        acceleration = [0.02; -0.1; 0.02];  % Final maneuver
    end
    % State update
    x_true(1:3,k) = x_true(1:3,k-1) + dt * x_true(4:6,k-1) + 0.5 * dt^2 * acceleration;
    x_true(4:6,k) = x_true(4:6,k-1) + dt * acceleration;
    x_true(:,k) = x_true(:,k) + [sqrt(Q_pos)*randn(3,1); sqrt(Q_vel)*randn(3,1)];
end
z = x_true(1:3,:) + sqrtm(R) * randn(3, T);

z = single(z);
x_true = single(x_true);