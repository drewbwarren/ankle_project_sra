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
L1 = 0.2;
L2 = 0.1;
l1 = .33/2;
l2 = .13/2;
h = .125;



L1 = .08;
L2 = .1;
l1 = .08;
l2 = .1;
h = .125;

plat_w = l2*2;
plat_l = l1*2;
middle_leg = h;
P = h;




% Kinematics
t = linspace(0,tf,10000);
theta = 20*sin(200/20*t)*pi/180 ;
phi = 15*sin(115/15*t)*pi/180;

vars = table(L1,L2,l1,l2,theta,phi,h,P);
[P1,P2] = position(vars);

P1 = cat(2,t',P1');
P2 = cat(2,t',P2');


sim('rp_2dof.slx')


