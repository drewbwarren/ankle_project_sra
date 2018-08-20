clc
clear
close all

% Kinematic Parameters
L1 = .08;
L2 = .1;
l1 = .08;
l2 = .1;
h = .125;
r = sqrt(2000)/2+20; r = r/100;
a11 = tan(pi/6)*6.5+7.7942; a11 = a11/100;
a12 = 6.5+13.5; a12 = a12/100;
a31 = 17.36+5; a31 = a31/100;
b11 = r*sin(pi/6);
b12 = r*cos(pi/6);
b31 = r;
f = .05;

P = h-f;





theta = 0;
phi = 0;
thetadot = 3.49066;
phidot = 2;
zdot = 2.5;
theta_tau = 150;
phi_tau = 25;
z_f = 1150;

theta_list = [0 20 20 -20 -20 20 20 -20 -20]*pi/180;
phi_list = [0 15 -15 -15 15 15 -15 -15 15]*pi/180;
z_list = [0 .1 -.1 .1 -.1 .1 -.1 .1 -.1];
thetadot_list = [thetadot -thetadot -thetadot thetadot thetadot -thetadot -thetadot thetadot thetadot];
phidot_list = [phidot -phidot phidot phidot -phidot -phidot phidot phidot -phidot];
zdot_list = [zdot -zdot zdot -zdot zdot -zdot zdot -zdot zdot];
theta_tau_list = [theta_tau -theta_tau -theta_tau theta_tau theta_tau -theta_tau -theta_tau theta_tau theta_tau];
phi_tau_list = [phi_tau -phi_tau phi_tau phi_tau -phi_tau -phi_tau phi_tau phi_tau -phi_tau];
z_f_list = [z_f z_f z_f z_f z_f z_f z_f z_f z_f];


Pdot = zeros(3,length(theta_list));
Pforce = zeros(3,length(theta_list));


for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    z = z_list(i);
    thetadot = thetadot_list(i);
    phidot = phidot_list(i);
    zdot = z_list(i);
    theta_tau = theta_tau_list(i);
    phi_tau = phi_tau_list(i);
    
    
    vars = table(r,theta,phi,h,P,z);
    [P1,P2,P3] = position(vars);
    P1 = P1 + P;
    P2 = P2 + P;
    P3 = P3 + P;

    
    
    J1 = 1/P1*[a11*sin(theta)*(-cos(theta)*a11 + b11) + (-sin(theta)*sin(phi)*a11 - cos(phi)*a12 + b12)*(-cos(theta)*sin(phi)*a11) + (sin(theta)*cos(phi)*a11 - sin(phi)*a12 - f + h + z)*(cos(theta)*cos(phi)*a11);
        (-sin(theta)*sin(phi)*a11 - cos(phi)*a12 + b12)*(-sin(theta)*cos(phi)*a11 + sin(phi)*a12) + (sin(theta)*cos(phi)*a11 - sin(phi)*a12 - f + h + z)*(-sin(theta)*sin(phi)*a11 - cos(phi)*a12);
        (sin(theta)*cos(phi)*a11 - sin(phi)*a12 - f + h + z)]';
    J2 = 1/P2*[a11*sin(theta)*(-cos(theta)*a11 + b11) + (-sin(theta)*sin(phi)*a11 + cos(phi)*a12 - b12)*(-cos(theta)*sin(phi)*a11) + (sin(theta)*cos(phi)*a11 + sin(phi)*a12 - f + h + z)*(cos(theta)*cos(phi)*a11);
        (-sin(theta)*sin(phi)*a11 + cos(phi)*a12 - b12)*(-sin(theta)*cos(phi)*a11 - sin(phi)*a12) + (sin(theta)*cos(phi)*a11 + sin(phi)*a12 - f + h + z)*(-sin(theta)*sin(phi)*a11 + cos(phi)*a12);
        (sin(theta)*cos(phi)*a11 + sin(phi)*a12 - f + h + z)]';
    J3 = 1/P3*[-a31*sin(theta)*(cos(theta)*a31 - r) + sin(theta)*cos(phi)^2*cos(theta)*a31^2 + (-sin(theta)*cos(phi)*a31 - f + h + z)*(-cos(theta)*cos(phi)*a31);
        -sin(theta)^2*sin(phi)*cos(phi)*a31^2 + (-sin(theta)*cos(phi)*a31 - f + h + z)*(sin(theta)*sin(phi)*a31);
        (-sin(theta)*cos(phi)*a31 - f + h + z)]';
        
        
    J = inv([J1; J2; J3]);
    inv(J)

    Pdot(:,i) = inv(J)*[thetadot phidot zdot]';
    Pforce(:,i) = J'*[theta_tau phi_tau z_f]';
    
end

% Actuator specs
max_vel = max(max(abs(Pdot),[],2))
max_f = max(max(abs(Pforce),[],2))

% Screw dimensions
d = [.5 .5 .625 .625 .75 .75 .75 .875 1 1 1 1 1]*.0254; % m
l = [.125 .1 1/6 .125 .2 1/6 .125 1/6 .25 .2 1/6 .125 .1]*.0254; % m
mu = .25; % friction

% Motor specs
T = max_f*(d./2).*(l + pi*mu*d)./(pi*d - mu*l)
w = max_vel./l.*60

