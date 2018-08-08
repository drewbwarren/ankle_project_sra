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
% L1 = 0.2;
% L2 = 0.1;
% l1 = .33/2;
% l2 = .13/2;
% h = .125;


L1 = .1% + .1;
L2 = .125 %+ .1;
l1 = .1;
l2 = .1;
L3 = .2;
l3 = .2;
h = .125;


middle_leg = h;
P = h;
arm_length = .05;
arm_height = .06; % this is the height from the base of the foot that determines where the ankle is
z = arm_height;
plat_w = (l2-arm_length)*2;
plat_l = (l1+l3-2*arm_length);



% Kinematics
t = linspace(0,tf,10000);
theta = 0*sin(2*t)*pi/180 - 0*pi/180;
phi = 0*sin(4*t)*pi/180 + 0*pi/180;
% phi = 0*t + 0*pi/180;
% phi = t*100*pi/180;
vars = table(L1,L2,L3,l1,l2,l3,theta,phi,h,P,z);
[P1,P2,P3,P4] = position(vars);

% P4 = -P2;
P1 = cat(2,t',P1');
P2 = cat(2,t',P2');
P3 = cat(2,t',P3');
P4 = cat(2,t',P4');

figure
plot(P2(:,1),P2(:,2))
hold on
plot(P4(:,1),P4(:,2))



sim('rp_2dof_4motors_2016version2.slx')


% This is for checking the output orientation against the desired
% orientation
% theta_test = pose.data(:,2);
% theta_test = theta_test(1:10000);
% theta_test = theta_test';
% phi_test = pose.data(:,1);
% phi_test = phi_test(1:10000);
% phi_test = phi_test';
% diff = theta*180/pi - theta_test;
% diff2 = phi*180/pi - phi_test;
% figure
% plot(t,diff)
% hold on
% plot(t,diff2)


max(P1_force)
max(P2_force)

