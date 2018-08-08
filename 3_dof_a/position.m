function [P1,P2,P3] = position(u)

theta_list = u.theta;
phi_list = u.phi;
z_list = u.z;
P = u.P;
h = u.h;
r = u.r;




% 1, left, 2, right, 3, back

% distance from top of side actuator to ankle - taken from sim
% stick_length = 13.5/cos(pi/6);
% the_rest = [-tan(pi/6)*6.5 -6.5 00000];



P1 = zeros(1,length(theta_list));
P2 = zeros(1,length(theta_list));
P3 = zeros(1,length(theta_list));

for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    z = z_list(i);
    
    R = [cos(theta) 0 sin(theta);
        sin(theta)*sin(phi) cos(phi) -sin(phi)*cos(theta);
        -sin(theta)*cos(phi) sin(phi) cos(theta)*cos(phi)];
    Rz = [-1 0 0;
        0 -1 0;
        0 0 1];
    R = R*Rz;
    
    c = [0 0 h+z]';
    
    % a - from output frame to top of linear actuator
    a1 = [tan(pi/6)*6.5+7.7942 6.5+13.5 0]'/100;
    a2 = [tan(pi/6)*6.5+7.7942 -6.5-13.5 0]'/100;
    a3 = [-17.36-5 0 0]'/100;

    % b - from base frame to bottom actuator joint
    b1 = [-r*sin(pi/6) -r*cos(pi/6) 5]'/100;
    b2 = [-r*sin(pi/6) r*cos(pi/6) 5]'/100;
    b3 = [r 0 5]'/100;

    p1 = c + R*a1 - b1;
    p2 = c + R*a2 - b2;
    p3 = c + R*a3 - b3;

    P1(i) = norm(p1) - P;
    P2(i) = norm(p2) - P;
    P3(i) = norm(p3) - P;
end