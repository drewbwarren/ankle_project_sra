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
foot_p = 150/(foot_h*foot_l*foot_w)/2;


% Kinematic parameters
L1 = .08;
L2 = .1;
l1 = .08;
l2 = .1;
f = foot_h;
h = .125+f;
P = h-f;


plat_w = l2*2;
plat_l = l1*2;
middle_leg = h-f;



% Kinematics
t = linspace(0,tf,10000);
theta = 20*sin(200/20*t)*pi/180 ;
phi = 15*sin(115/15*t)*pi/180;

vars = table(L1,L2,l1,l2,theta,phi,h,P,f);
[P1,P2,P3,P4] = position(vars);


P1 = cat(2,t',P1');
P2 = cat(2,t',P2');
P3 = cat(2,t',P3');
P4 = cat(2,t',P4');


sim('rp_2dof_4motors.slx')

