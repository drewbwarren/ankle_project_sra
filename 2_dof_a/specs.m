clc
clear
close all

% Kinematic Parameters
L1 = .3; %.175;
L2 = .15;
l1 = .05; %.15;
l2 = .02; %.075;
h = .125;
f = 0.05;

x = .07;
y = .11;
L1 = .08;
L2 = .1;
l1 = .08;
l2 = .1;
h = .125;

P = h-f;

theta = 0;
phi = 0;
thetadot = 3.49066;
phidot = 2;
theta_tau = 150;
phi_tau = 25;

theta_list = [0 20 20 -20 -20 20 20 -20 -20]*pi/180;
phi_list = [0 15 -15 -15 15 15 -15 -15 15]*pi/180;
thetadot_list = [thetadot -thetadot -thetadot thetadot thetadot -thetadot -thetadot thetadot thetadot];
phidot_list = [phidot -phidot phidot phidot -phidot -phidot phidot phidot -phidot];
theta_tau_list = [theta_tau -theta_tau -theta_tau theta_tau theta_tau -theta_tau -theta_tau theta_tau theta_tau];
phi_tau_list = [phi_tau -phi_tau phi_tau phi_tau -phi_tau -phi_tau phi_tau phi_tau -phi_tau];


Pdot = zeros(2,length(theta_list));
Pforce = zeros(2,length(theta_list));


for i = 1:length(theta_list)
    
    theta = theta_list(i);
    phi = phi_list(i);
    thetadot = thetadot_list(i);
    phidot = phidot_list(i);
    theta_tau = theta_tau_list(i);
    phi_tau = phi_tau_list(i);
    
    
    vars = table(L1,L2,l1,l2,theta,phi,h,P);
    [P1,P2] = position(vars);
    P1 = P1 + P;
    P2 = P2 + P;


    J = inv([(l1/P1)*((-l1*sin(theta) + L1)*sin(theta) + l1*cos(theta)*sin(phi) + (h - f + l1*sin(theta)*cos(phi))*cos(theta)*cos(phi)) (l1/P1)*(l1*sin(theta)*cos(phi) - (h - f + l1*sin(theta)*cos(phi))*sin(theta)*sin(phi));
        0 (l2/P2)*(L2*sin(phi) - h*cos(phi) + f*cos(phi))]);

    Pdot(:,i) = inv(J)*[thetadot phidot]';
    Pforce(:,i) = J'*[theta_tau phi_tau]';
    
end

% Actuator specs
Pdot%*39.37
Pforce%*.224809
max(abs(Pdot),[],2)%*39.37
max(abs(Pforce),[],2)%*.224809


% Motor specs
d = .0254; % m, 1 inch
l = 1/6*.0254; % m, 1/6 inch pitch
mu = .25; % friction
T = max(max(abs(Pforce),[],2))*(d/2)*(l + pi*mu*d)/(pi*d - mu*l)
w = max(max(abs(Pdot),[],2))/l*60

