function [Pfront,Pleft,Pright] = actuator_positions(u)

r = u.r;
R = u.R;
L1 = u.L1;
L2 = u.L2;

theta = u.theta;
phi = u.phi;
H = u.H;



Pfront = sqrt(r.^2 - 2.*r.*L1.*cos(theta) + L1.^2.*cos(theta).^2 + L1.^2.*sin(theta).^2.*cos(phi).^2 + ...
    2.*H.*L1.*sin(theta).*cos(phi) + H.^2);
Pleft = sqrt(R.^2 + 2.*R.*L1.*sin(theta).*sin(phi) - 2.*R.*L2.*cos(phi) + L1.^2.*sin(theta).^2.*sin(phi).^2 - ...
    2.*L1.*L2.*sin(theta).*sin(phi).*cos(phi) + L2.^2 + H.^2 - 2.*H.*L2.*sin(phi));
Pright = sqrt(R.^2 - 2.*R.*L1.*sin(theta).*sin(phi) - 2.*R.*L2.*cos(phi) + L1.^2.*sin(theta).^2.*sin(phi).^2 + ...
    2.*L1.*L2.*sin(theta).*sin(phi).*cos(phi) + L2.^2 + H.^2 + 2.*H.*L2.*sin(phi));

% Kinematic Equations but remove a and b
% Pfront = sqrt(r.^2 - 2.*r.*L1.*cos(theta) + L1.^2.*cos(theta).^2 + L1.^2.*sin(theta).^2 + ...
%     2.*H.*L1.*sin(theta) + H.^2);
% Pleft = sqrt(R.^2 - 2.*R.*L2.*cos(phi) ...
%      + L2.^2 + H.^2 - 2.*H.*L2.*sin(phi));
% Pright = sqrt(R.^2 - 2.*R.*L2.*cos(phi) ...
%      + L2.^2 + H.^2 + 2.*H.*L2.*sin(phi));


end

