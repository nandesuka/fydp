clear all;
clc;
%% Load data
dt = 0.1;
t_sim = 0:dt:26;
% - Lower leg imu (frame 3)
%   - Map imu 'z' to -'y', imu 'x' to 'z', imu 'y' to -'x'
% - Upper leg imu (frame 5)
%   - Map imu 'z' to -'x', imu 'x' to 'y', imu 'y' to -'z'
%
% - acc5 -> columns 2:4
% - gyr5 -> columns 5:7
% - acc3 -> columns 11:13
% - gyr3 -> columns 14:16
data = csvread('putty_table.log');
n = 1:2:length(t_sim);
z = zeros(12, length(n));
z(1:3, :) = data(n, 14:16)';
z(4:6, :) = data(n, 11:13)';
z(7:9, :) = data(n, 5:7)';
z(10:12, :) = data(n, 2:4)';

% z(1, :) = -data(length(t_sim), 15)';
% z(2, :) = data(length(t_sim), 16)';
% z(3, :) = data(length(t_sim), 14)';

% imu frame 3
temp = z(1, :);
z(1, :) = -z(2, :);
z(2, :) = -z(3, :);
z(3, :) = temp;

z(1, :) = z(1, :) - mean(z(1, :));
z(2, :) = z(2, :) - mean(z(2, :));
z(3, :) = z(3, :) - mean(z(3, :));

temp = z(4, :);
z(4, :) = -z(5, :)*9.81;
z(5, :) = -z(6, :)*9.81;
z(6, :) = temp*9.81;

z(4, :) = z(4, :) - mean(z(4, :));
z(5, :) = z(5, :) - mean(z(5, :));
z(6, :) = z(6, :) - mean(z(6, :));

% imu frame 5
temp = z(7, :);
z(7, :) = -z(9, :);
z(9, :) = -z(8, :);
z(8, :) = temp;

z(7, :) = z(7, :) - mean(z(7, :));
z(8, :) = z(8, :) - mean(z(8, :));
z(9, :) = z(9, :) - mean(z(9, :));

temp = z(10, :);
z(10, :) = -z(12, :)*9.81;
z(12, :) = -z(11, :)*9.81;
z(11, :) = temp*9.81;

z(10, :) = z(10, :) - mean(z(10, :));
z(11, :) = z(11, :) - mean(z(11, :));
z(12, :) = z(12, :) - mean(z(12, :));

% gyro 3
Sg = cov([z(1, :)', z(2, :)', z(3, :)']);
[SEg,Seg] = eig(Sg);

Sg_diag = zeros(3, 3);
Sg_diag(1, 1) = Sg(1, 1);
Sg_diag(2, 2) = Sg(2, 2);
Sg_diag(3, 3) = Sg(3, 3);
[SEg_diag,Seg_diag] = eig(Sg_diag);

z_sim = zeros(size(z, 1), size(z, 2));
z_sim_diag = zeros(size(z, 1), size(z, 2));
z_sim(1:3, :) = SEg*sqrt(Seg)*randn(3,length(n));
z_sim_diag(1:3, :) = SEg_diag*sqrt(Seg_diag)*randn(3,length(n));

% gyro 5
Sg2 = cov([z(7, :)', z(8, :)', z(9, :)']);
[SEg2,Seg2] = eig(Sg2);
Sg_diag2 = zeros(3, 3);
Sg_diag2(1, 1) = Sg2(1, 1);
Sg_diag2(2, 2) = Sg2(2, 2);
Sg_diag2(3, 3) = Sg2(3, 3);
[SEg_diag2,Seg_diag2] = eig(Sg_diag2);

z_sim(7:9, :) = SEg2*sqrt(Seg2)*randn(3,length(n));
z_sim_diag(7:9, :) = SEg_diag2*sqrt(Seg_diag2)*randn(3,length(n));

% accel 3
Sa = cov([z(4, :)', z(5, :)', z(6, :)']);
[SEa,Sea] = eig(Sa);

Sa_diag = zeros(3, 3);
Sa_diag(1, 1) = Sa(1, 1);
Sa_diag(2, 2) = Sa(2, 2);
Sa_diag(3, 3) = Sa(3, 3);
[SEa_diag,Sea_diag] = eig(Sa_diag);

z_sim(4:6, :) = SEa*sqrt(Sea)*randn(3,length(n));
z_sim_diag(4:6, :) = SEa_diag*sqrt(Sea_diag)*randn(3,length(n));

% accel 3
Sa2 = cov([z(10, :)', z(11, :)', z(12, :)']);
[SEa2,Sea2] = eig(Sa2);

Sa_diag2 = zeros(3, 3);
Sa_diag2(1, 1) = Sa2(1, 1);
Sa_diag2(2, 2) = Sa2(2, 2);
Sa_diag2(3, 3) = Sa2(3, 3);
[SEa_diag2,Sea_diag2] = eig(Sa_diag2);

z_sim(10:12, :) = SEa2*sqrt(Sea2)*randn(3,length(n));
z_sim_diag(10:12, :) = SEa_diag2*sqrt(Sea_diag2)*randn(3,length(n));

% Construct R
R = blkdiag(Sg_diag, Sa_diag, Sg_diag2, Sa_diag2);

figure(1)
t_sim = 1:length(z(1, :));
subplot(3, 2, 1);
plot(t_sim, z(1, :), 'b', t_sim, z_sim_diag(1, :), 'm');

subplot(3, 2, 2);
plot(t_sim, z(2, :), 'b', t_sim, z_sim_diag(2, :), 'm');

subplot(3, 2, 3);
plot(t_sim, z(3, :), 'b', t_sim, z_sim_diag(3, :), 'm');

subplot(3, 2, 4);
plot(t_sim, z_sim_diag(4, :), 'r', t_sim, z(4, :), 'g');

subplot(3, 2, 5);
plot(t_sim, z_sim_diag(5, :), 'r', t_sim, z(5, :), 'g');

subplot(3, 2, 6);
plot(t_sim, z_sim_diag(6, :), 'r', t_sim, z(6, :), 'g');

figure(2)
subplot(3, 2, 1);
plot(t_sim, z(7, :), 'b', t_sim, z_sim_diag(7, :), 'm');

subplot(3, 2, 2);
plot(t_sim, z(8, :), 'b', t_sim, z_sim_diag(8, :), 'm');

subplot(3, 2, 3);
plot(t_sim, z(9, :), 'b', t_sim, z_sim_diag(9, :), 'm');

subplot(3, 2, 4);
plot(t_sim, z_sim_diag(10, :), 'r', t_sim, z(10, :), 'g');

subplot(3, 2, 5);
plot(t_sim, z_sim_diag(11, :), 'r', t_sim, z(11, :), 'g');

subplot(3, 2, 6);
plot(t_sim, z_sim_diag(12, :), 'r', t_sim, z(12, :), 'g');