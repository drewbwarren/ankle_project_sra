function [P1,P2,P3] = actuator_positions(u)


L1 = u.L1;
L2 = u.L2;
l1 = u.l1;
l2 = u.l2;
P = u.P;
f = u.f;

theta_list = u.theta;
phi_list = u.phi;
H_list = u.H;



% front, left, right
P1 = zeros(1,length(theta_list));
P2 = zeros(1,length(theta_list));
P3 = zeros(1,length(theta_list));

for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    H = H_list(i);
    
    Rot = [cos(theta) 0 sin(theta);
        sin(theta)*sin(phi) cos(phi) -sin(phi)*cos(theta);
        -sin(theta)*cos(phi) sin(phi) cos(theta)*cos(phi)];
    Rz = [-1 0 0;
        0 -1 0;
        0 0 1];
    Rot = Rot*Rz;
    
    
    c = [0 0 H]';
    
    
    % a - from output frame to top of linear actuator
    a1 = [-l1 0 0]';
    a2 = [0 -l2 0]';
    a3 = [0 l2 0]';

    % b - from base frame to bottom actuator joint
    b1 = [L1 0 .05]';
    b2 = [0 L2 .05]';
    b3 = [0 -L2 .05]';

    p1 = c + Rot*a1 - b1;
    p2 = c + Rot*a2 - b2;
    p3 = c + Rot*a3 - b3;

    P1(i) = norm(p1) - P;
    P2(i) = norm(p2) - P;
    P3(i) = norm(p3) - P;
end

