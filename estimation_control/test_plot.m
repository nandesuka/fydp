clear all;
clc;
mdl_leg;
% legm.plot([0 0 0 0 0]);
% legm.plot([pi/2 pi/2 -pi/2 pi/2 0]);
% legm.plot([pi/2 pi/2 -pi/2 pi/4 0], 'view', [-150 40], 'notiles', 'noshadow', 'noshading');

% tic
% legm.plot([2.0252    1.0705   -1.8659    0.2702   -3.0847], 'view', [85 -60], 'notiles', 'noshadow', 'lightpos', [0 0 -20]);
% toc
% tic
% legm.plot([2.0252    1.0705   -1.8659    0.2702   -3.0847], 'view', [85 -60], 'notiles', 'noshadow', 'lightpos', [0 0 -20]);
% toc

tic
legm.plot([2.0252    1.0705   -1.8659    0.2702   -3.0847], 'view', [85 -60], ...
           'notiles', 'noshadow', 'noshading');
toc

tic
legm.plot([2.0252    1.0705   -1.8659    0.2702   -3.0847], 'view', [85 -60], ...
           'notiles', 'noshadow', 'noshading');
toc
% for i = 0:0.1:2*pi
%     tic
%     legm.plot([i 0 0 0 0]);
%     toc
% end