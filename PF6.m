%% Advanced 3D Particle Filter - UAV Tracking in XYZ Space
clc, clear, close all;
fs = 20e6;
ts = 1/fs;
rng(5);
state_dim = 6;
N = 1000;         % Number of particles
T = 100;        % Number of time steps
Q_pos = single(0.01);
Q_vel = single(0.05);
dt = single(0.1);
R = single(diag([0.5, 0.5, 1.0]));
[z,x_true]  = trajectory(state_dim,T,Q_pos,Q_vel,dt,R);
save_file(z','z_matrix.bin')
%% 3D Particle Filter Implementation
weights = single(ones(1, N) / N);
x_est = zeros(state_dim, T,'single');
Q = single(blkdiag(Q_pos * eye(3), Q_vel * eye(3)));QQ = chol(Q,'lower');
%
r = make_rand(N);
save_file(r','r_matrix.bin')
%
initial_pos = z(:,1) + single(sqrt(2)) * r(1:3,:);
initial_vel = single(0.5 * r(1:3,:));
p = [initial_pos; initial_vel];
A = single([eye(3), dt * eye(3); zeros(3), eye(3)]);
%% Main Filter Loop
for t = 2:T
    % Step 1: Prediction
    process_noise = single(QQ * r);
    p = A * p;
    p = p + process_noise;
    % Step 2: Weight Update
    innovation = z(:,t) - p(1:3,:);
    innovation = innovation.^2;
    mahalanobis = -0.5 *sum(innovation./diag(R));
    weights = single(exp( mahalanobis));
    weight_sum = sum(weights);
    weights = weights / weight_sum;
    % Step 3: Systematic Resampling
    indices = systematic_resampling_3d(weights,r(1,1));
    p = p(:,indices);
    % Step 4: State Estimation
    x = single(single(sum(p, 2))/N);
    x_est(:,t) = x;
end
err = sumabs(x_true - x_est)
save_file(x_est','x_matrix.bin');x_est'
%% Systematic Resampling Function
function indices = systematic_resampling_3d(weights,r)
N = single(length(weights));
indices = zeros(1, N,'single');
CDF = cumsum(weights);
u0 = r / N;
i = 1;
for j = 1:N
    u = u0 + (j-1)/N;
    while u > CDF(i) && i<N
        i = i + 1;
    end
    indices(j) = i;
end
end
%%
function x = make_rand(N)
state = single(0.1234567);
for i=1:6
    for j = 1:N
        tmp = state * 3.999877;
        state = fix(tmp) - tmp;
        x(i,j) = state;
    end
end
end
%%
function save_file(x,name)
fid = fopen(name, 'wb');
fwrite(fid,x, 'float32');
fclose(fid);
end
