clear all; clc;
delete(instrfind);
%% Setup
% upper leg length and lower leg length
leg.u = 0.44;
leg.l = 0.43;
% upper leg sensor and lower leg sensor
leg.us = 0.13;
leg.ls = 0.3;

% Simulation
dt = 0.3;
t_sim = 0:dt:500;

% Noise matrices
% - R is measurement noise covariance matrix, assumed zero-mean and white (v)
% - Q is process noise and assumed to be zero-mean and gaussian (w)
% Rgyro = eye(3)*freq*0.01;
% Raccel = eye(3)*freq*freq*0.01;
% R = blkdiag(Rgyro, Raccel, Rgyro, Raccel);
% [RE, Re] = eig(R);

load('measure_covariance.mat');
% load('meas_cov_10hz.mat');
[RE, Re] = eig(R);

% Q_i = [0.01       0            0;
%        0          1e-3         0;
%        0          0            1e-3];
Q_i = eye(3)*1e-5;
W_i = [0 0 0;
       0 1 0;
       0 0 1];
QW_i = W_i*Q_i*W_i';
% QW_i = Q_i;
Q = blkdiag(QW_i, QW_i, QW_i, QW_i, QW_i);

% Process matrix
A_i = [1, dt, dt*dt/2;
       0, 1, dt;
       0, 0, 1];
A = blkdiag(A_i, A_i, A_i, A_i, A_i);
At = A';

% State variables
ds = 1e-8;
s_mu = zeros(15, 1);
s_mu(3*(0:4) + 1) = [pi/2 pi/2 -pi/2 pi/2 0]';
s_mu_data = zeros(15, length(t_sim));
s_mu_data(:, 1) = s_mu;
P_pred = eye(15);
P_mu = eye(15);

%% Robotics toolbox
mdl_leg;

%% Load data
s = serial('COM4','BaudRate',115200);
%Open Port 
fopen(s);
%Set Terminator
set(s,'Terminator','CR/LF');fscanf(s);
deg2rad = pi/180;
% - Lower leg imu (frame 3)
%   - Map imu 'z' to -'y', imu 'x' to 'z', imu 'y' to -'x'
% - Upper leg imu (frame 5)
%   - Map imu 'z' to -'x', imu 'x' to 'y', imu 'y' to -'z'
%
% - acc5 -> columns 2:4
% - gyr5 -> columns 5:7
% - acc3 -> columns 11:13
% - gyr3 -> columns 14:16
z = zeros(12, 1);
data = zeros(13, length(t_sim));
%% Simulation
for i =1:length(t_sim)
    % Update current time
    % t = (i - 1)*dt;
    z(:,1) = fscanf(s, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,');
    
    % Prediction update
    s_mu_prev = s_mu;
    s_mu = A*s_mu;
    P_pred = A*P_mu*At + Q;

    C = compute_C(s_mu, ds, leg);    % use s_mu before or after process update?
    Ct = C';
    P_y = C*P_pred*Ct + R;
    K = P_pred*Ct/P_y;
    
    % Measurement update
    % - 'z' is measurement
    % - 'h' is predicted measurement
    
    % z = compute_z(s_true, leg) + RE*sqrt(Re)*randn(12, 1);    % 3 axes * 4 sensors (2 gyros, 2 accels) = 12
    
    h = compute_z(s_mu, leg);
    z_i = z(:,1);
    s_mu = s_mu + K*(z_i - h);
    P_mu = (eye(15) - K*C)*P_pred;
    
     % Store data
     %s_mu(3*(0:4) + 1) = wrapToPi(s_mu(3*(0:4) + 1));
     %s_mu_data(:, i) = s_mu;

    % Visualize
    legm.plot( (s_mu(3*(0:4) + 1))', 'view', [85 -60], 'notiles', 'noshadow', 'lightpos', [0 0 -20], 'zoom', 1.0);
     %legm.animate((s_mu(3*(0:4) + 1))');
     %legm.plot( (s_mu(3*(0:4) + 1))' , 'notiles', 'noshadow', 'lightpos', [0 0 -20]);
end