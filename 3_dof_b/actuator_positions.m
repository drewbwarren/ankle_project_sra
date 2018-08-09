function [P1,P2,P3] = actuator_positions(u)

r = u.r;
R = u.R;
L1 = u.L1;
L2 = u.L2;
P = u.P;

theta_list = u.theta;
phi_list = u.phi;
H_list = u.H;



% Pfront = sqrt(r.^2 - 2.*r.*L1.*cos(theta) + L1.^2.*cos(theta).^2 + L1.^2.*sin(theta).^2.*cos(phi).^2 + ...
%     2.*H.*L1.*sin(theta).*cos(phi) + H.^2);
% Pleft = sqrt(R.^2 + 2.*R.*L1.*sin(theta).*sin(phi) - 2.*R.*L2.*cos(phi) + L1.^2.*sin(theta).^2.*sin(phi).^2 - ...
%     2.*L1.*L2.*sin(theta).*sin(phi).*cos(phi) + L2.^2 + H.^2 - 2.*H.*L2.*sin(phi));
% Pright = sqrt(R.^2 - 2.*R.*L1.*sin(theta).*sin(phi) - 2.*R.*L2.*cos(phi) + L1.^2.*sin(theta).^2.*sin(phi).^2 + ...
%     2.*L1.*L2.*sin(theta).*sin(phi).*cos(phi) + L2.^2 + H.^2 + 2.*H.*L2.*sin(phi));

% Kinematic Equations but remove a and b
% Pfront = sqrt(r.^2 - 2.*r.*L1.*cos(theta) + L1.^2.*cos(theta).^2 + L1.^2.*sin(theta).^2 + ...
%     2.*H.*L1.*sin(theta) + H.^2);
% Pleft = sqrt(R.^2 - 2.*R.*L2.*cos(phi) ...
%      + L2.^2 + H.^2 - 2.*H.*L2.*sin(phi));
% Pright = sqrt(R.^2 - 2.*R.*L2.*cos(phi) ...
%      + L2.^2 + H.^2 + 2.*H.*L2.*sin(phi));


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
    a1 = [-L1 0 0]';
    a2 = [0 -L2 0]';
    a3 = [0 L2 0]';

    % b - from base frame to bottom actuator joint
    b1 = [r 0 .05]';
    b2 = [0 R .05]';
    b3 = [0 -R .05]';

    p1 = c + Rot*a1 - b1;
    p2 = c + Rot*a2 - b2;
    p3 = c + Rot*a3 - b3;

    P1(i) = norm(p1) - P;
    P2(i) = norm(p2) - P;
    P3(i) = norm(p3) - P;
end

