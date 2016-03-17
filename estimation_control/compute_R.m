function [ R ] = compute_R( i, s, leg )
dh = compute_dh( s, leg );

R = [cos(dh.th(i)), -sin(dh.th(i))*cos(dh.alp(i)),  sin(dh.th(i))*sin(dh.alp(i));
     sin(dh.th(i)),  cos(dh.th(i))*cos(dh.alp(i)), -cos(dh.th(i))*sin(dh.alp(i));
     0            ,  sin(dh.alp(i))              ,  cos(dh.alp(i))               ];

end

