clc
clear
close all



tf = 10;

base_length = .4;
base_width = .4;
base_height = .01;

foot_length = .03;

leg1_base = .1;

leg1_top_length = .05;
leg1_top_width = .01;
leg1_top_height = .01;
leg1_top_thickness = .005;

foot_h = .05;
foot_l = .15;
foot_w = .08;
foot_p = 500/(foot_h*foot_l*foot_w)/2;


% Kinematic parameters
L1 = .2% + .1;
L2 = .2 %+ .1;
l1 = .2;
l2 = .2;
L3 = L1;
l3 = l1;
f = foot_h;
h = .125+f;
P = h-f;


arm_length = .05;
arm_height = .06; % this is the height from the base of the foot that determines where the ankle is
z = arm_height;
plat_w = (l2-arm_length)*2;
plat_l = (l1+l3-2*arm_length);



% Kinematics
t = linspace(0,tf,10000);
theta = 2*sin(2*t)*pi/180 - 0*pi/180;
phi = 2*sin(4*t)*pi/180 + 0*pi/180;
% phi = 0*t + 0*pi/180;
% phi = t*100*pi/180;
vars = table(L1,L2,L3,l1,l2,l3,theta,phi,h,P,z,f);
[P1,P2,P3,P4] = position(vars);

% P1 = t*0; P2 = P1;P3 = P1;P4=P1;

P1 = cat(2,t',P1');
P2 = cat(2,t',P2');
P3 = cat(2,t',P3');
P4 = cat(2,t',P4');



sim('rp_2dof_4motors.slx')


