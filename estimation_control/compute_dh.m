function [ dh ] = compute_dh( s, leg )
dh.alp = [pi/2; -pi/2; -pi/2; pi/2; 0];
dh.a = [0; 0; leg.l; 0; 0];
dh.d = [0; 0; 0; 0; leg.u];

dh.th = [s(3*(1 - 1) + 1); s(3*(2 - 1) + 1); s(3*(3 - 1) + 1); s(3*(4 - 1) + 1); s(3*(5 - 1) + 1)];
end

