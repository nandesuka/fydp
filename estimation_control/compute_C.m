function [ C ] = compute_C( s, ds, leg )
C = zeros(4, 15);

z2 = compute_z(s, leg);

% for j = 1:15
%     s1 = s;
%     s1(j) = s1(j) + ds;
%     C(1:12, j) = compute_z(s1, leg) - z2;
% end

s1 = s;
j = 1;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 2;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 3;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 4;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 5;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 6;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 7;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 8;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 9;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 10;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 11;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 12;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 13;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 14;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;

s1 = s;
j = 15;
s1(j) = s1(j) + ds;
C(1:12, j) = compute_z(s1, leg) - z2;


C = C/ds;

end

