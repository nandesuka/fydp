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
% n = 1 -> 20 Hz -> dt = 0.05
% n = 2 -> 10 Hz -> dt = 0.1
n = 1;
dt = 0.05*n;
t_sim = 0:dt:100;

% Joint sinusoids
freq = 0.1;
qfunc = @(t) sin(freq*t);
qdfunc = @(t) freq*cos(freq*t);
qddfunc = @(t) -freq*freq*sin(freq*t);

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
%% Simulation
for i = 1:length(t_sim)
    % Update current time
    % t = (i - 1)*dt;
    data(1,:)=fscanf(s, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,');
    %data(1,:) = ones(1,19)*0.01;
    z(1:3, :) = data(1, 14:16)';
    z(4:6, :) = data(1, 11:13)';
    z(7:9, :) = data(1, 5:7)';
    z(10:12, :) = data(1, 2:4)';
    % imu frame 3
    temp = z(1, :);
    z(1, :) = -z(2, :)*deg2rad;
    z(2, :) = -z(3, :)*deg2rad;
    z(3, :) = temp*deg2rad;
    temp = z(4, :);
    z(4, :) = -z(5, :)*9.81;
    z(5, :) = -z(6, :)*9.81;
    z(6, :) = temp*9.81;
    temp = z(7, :);
    z(7, :) = -z(9, :)*deg2rad;
    z(9, :) = -z(8, :)*deg2rad;
    z(8, :) = temp*deg2rad;
    temp = z(10, :);
    z(10, :) = -z(12, :)*9.81;
    z(12, :) = -z(11, :)*9.81;
    z(11, :) = temp*9.81;
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
     s_mu(3*(0:4) + 1) = wrapToPi(s_mu(3*(0:4) + 1));
     s_mu_data(:, i) = s_mu;
%toc

    % Visualize
    if (mod(i, 10) == 0)
        tic
        legm.plot( (s_mu(3*(0:4) + 1))', 'view', [85 -60], 'notiles', 'noshadow', 'lightpos', [0 0 -20]);
        toc
        %legm.animate( (s_mu(3*(0:4) + 1))');
        %legm.plot( (s_mu(3*(0:4) + 1))' , 'notiles', 'noshadow', 'lightpos', [0 0 -20]);
    end

end


