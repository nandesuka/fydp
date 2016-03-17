function [ z ] = compute_z( s, leg )
z = zeros(12, 1);

R01 = compute_R(1, s, leg);
R12 = compute_R(2, s, leg);
R23 = compute_R(3, s, leg);
R34 = compute_R(4, s, leg);
% R45 = compute_R(5, s, leg);

R03 = R01*R12*R23;
R05 = R03*R34;

z(1:3) = compute_w(3, s, leg);
% z(4:6) = compute_xdd_sensor(3, s, leg) + R03*[-9.81 0 0]';
rl = [leg.ls; 0; 0];
w3 = compute_w(3, s, leg);
z(4:6) = cross(compute_wd(3, s, leg), rl) + cross(w3, cross(w3, rl)) + R03'*[-9.81 0 0]';

z(7:9) = compute_w(5, s, leg);
% z(10:12) = compute_xdd_sensor(5, s, leg) + R05*[-9.81 0 0]';
ru = [0; 0; leg.us];
w5 = compute_w(5, s, leg);
z(10:12) = compute_xdd(4, s, leg) + cross(compute_wd(5, s, leg), ru) ...
           + cross(w5, cross(w5, ru)) + R05'*[-9.81 0 0]';

end

