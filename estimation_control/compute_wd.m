function [ wd ] = compute_wd( i, s, leg)
    if i == 0
        wd = [0; 0; 0];
    else
        R = compute_R(i, s, leg);
        w = compute_w(i, s, leg);
        wd = R'*compute_wd(i - 1, s, leg) + R'*[0; 0; s(3*(i - 1) + 3)] ...
             + cross(w, R'*[0; 0; 3*(i - 1) + 2]);
    end
end

