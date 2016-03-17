function [ xdd ] = compute_xdd( i, s, leg )
% gravity not implemented
dh = compute_dh(s, leg);
if i == 0 || i == 1 || i == 2
    xdd = [0; 0; 0];
else
    R = compute_R(i, s, leg);
    r = [0; 0; 0] + (i == 5)*[0; 0; -dh.d(5)] + (i == 3)*[-dh.a(3); 0; 0];
    w = compute_w(i, s, leg);
    xdd = R'*compute_xdd(i - 1, s, leg) + cross(compute_wd(i, s, leg), r) + ...
          cross(w, cross(w, r));
end

end

