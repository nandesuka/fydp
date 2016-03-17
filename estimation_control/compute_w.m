function w = compute_w(i, s, leg)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if i == 0
    w = [0; 0; 0];
else
    R = compute_R(i, s, leg);
    w = R'*compute_w(i - 1, s, leg) + R'*[0, 0, s(3*(i - 1) + 2)]';
end

end

