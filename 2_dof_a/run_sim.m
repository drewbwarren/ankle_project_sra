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
[P1,P2,P3,P4] = position(vars);

% P1 = 0*t - .01;
% P3 = 0*t + .01;
% P2 = 0*t + .01;

% P3 = -P1;
% P4 = -P2;

P1 = cat(2,t',P1');
P2 = cat(2,t',P2');
P3 = cat(2,t',P3');
P4 = cat(2,t',P4');
max(P1(:,2))-min(P1(:,2))
max(P2(:,2))-min(P2(:,2))
max(P3(:,2))-min(P3(:,2))
max(P4(:,2))-min(P4(:,2))


sim('rp_2dof_2016a.slx')


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
% max(P3_force)
% max(P4_force)
max(center_force)

figure
hold on
plot(P1_force)
plot(P2_force)
% plot(P3_force)
% plot(P4_force)
plot(center_force)

%%
figure
hold on
plot(P1_sphere)
plot(P2_sphere)

max(P1_sphere.data(:,1))
max(P1_sphere.data(:,2))
max(P1_sphere.data(:,3))
max(P2_sphere.data(:,1))
max(P2_sphere.data(:,2))
max(P2_sphere.data(:,3))
