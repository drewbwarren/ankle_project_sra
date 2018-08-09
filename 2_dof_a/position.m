function [P1,P2] = position(u)

theta_list = u.theta;
phi_list = u.phi;
P = u.P;
L1 = u.L1;
L2 = u.L2;
l1 = u.l1;
l2 = u.l2;
h = u.h;



c = [0 0 h]';

a1 = [-l1 0 0]';
a2 = [0 -l2 0]';

b1 = [-L1 0 0]';
b2 = [0 -L2 0]';

P1 = zeros(1,length(theta_list));
P2 = zeros(1,length(theta_list));

for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    
    R = [cos(theta) 0 sin(theta);
        sin(theta)*sin(phi) cos(phi) -sin(phi)*cos(theta);
        -sin(theta)*cos(phi) sin(phi) cos(theta)*cos(phi)];

    p1 = c + R*a1 - b1;
    p2 = c + R*a2 - b2;

    P1(i) = norm(p1) - P;
    P2(i) = norm(p2) - P;
end


end

