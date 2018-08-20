clc
clear
close all

% Kinematic Parameters
L1 = .4;
L2 = L1*sind(60);
l1 = .2;
l2 = .2;
h = .4268;
f = .05;


P = h;


r = sqrt(2000)/2+20;


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
    
    H = z+h;
    vars = table(L1,L2,l1,l2,theta,phi,h,P,z,f,H);
    [P1,P2,P3] = actuator_positions(vars);
    P1 = P1 + P;
    P2 = P2 + P;
    P3 = P3 + P;

   
    
    
    J1 = 1/P1*[-l1*sin(theta)*(l1*cos(theta) - L1) + l1^2*cos(theta)*sin(phi) - (h + z - f - l1*sin(theta)*cos(phi))*l1*cos(theta)*cos(phi);
        l1^2*sin(theta)*cos(phi) + (h + z - f - l1*sin(theta)*cos(phi))*l1*sin(theta)*sin(phi);
        (h + z - f - l1*sin(theta)*cos(phi))]';
    J2 = 1/P2*[0;
        -l2*sin(phi)*(l2*cos(phi) + L2) + (h + z - f + l2*sin(phi))*l2*cos(phi);
        (h + z - f + l2*sin(phi))]';
    J3 = 1/P3*[0;
        (L2 - l2*cos(phi))*l2*sin(phi) - (h + z - f - l2*sin(phi))*l2*cos(phi);
        (h + z - f - l2*sin(phi))]';
        
        
    J = inv([J1; J2; J3]);

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

