function [P1,P2,P3,P4] = position(u)

theta_list = u.theta;
phi_list = u.phi;
P = u.P;
L1 = u.L1;
L2 = u.L2;
l1 = u.l1;
l2 = u.l2;
L3 = u.L3;
l3 = u.l3;
h = u.h;

% Pback0 = sqrt(L1^2 + l1^2 + h^2 - 2*l1*(L1*cos(0) - h*sin(0))) - P;%cos(theta);
% Pside0 = sqrt(L2^2 + l2^2 + h^2 - 2*l2*(L2*cos(0) + h*sin(0))) - P;
% 
% Pback = sqrt(L1^2 + l1^2 + h^2 - 2*l1*(L1*cos(theta) - h*sin(theta)));
% Pside = sqrt(L2^2 + l2^2 + h^2 - 2*l2*(L2*cos(phi) + h*sin(phi)));
% 
% 
% Pback = Pback - P; % - Pback0;
% Pside = Pside - P; % - Pside0 ;

x = 0.00;
z = u.z;

c = [-x 0 h+z]';

a1 = [-(l1-x) 0 -z]';
a2 = [0 -l2 -z]';
a3 = [(l3+x) 0 -z]';
a4 = [0 l2 -z]';

b1 = [-L1 0 0]';
b2 = [0 -L2 0]';
b3 = [L3 0 0]';
b4 = [0 L2 0]';

P1 = zeros(1,length(theta_list));
P2 = zeros(1,length(theta_list));
P3 = zeros(1,length(theta_list));
P4 = zeros(1,length(theta_list));

for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    
    R = [cos(theta) 0 sin(theta);
        sin(theta)*sin(phi) cos(phi) -sin(phi)*cos(theta);
        -sin(theta)*cos(phi) sin(phi) cos(theta)*cos(phi)];

    p1 = c + R*a1 - b1;
    p2 = c + R*a2 - b2;
    p3 = c + R*a3 - b3;
    p4 = c + R*a4 - b4;

    P1(i) = norm(p1) - P;
    P2(i) = norm(p2) - P;
    P3(i) = norm(p3) - P;
    P4(i) = norm(p4) - P;
end


end

