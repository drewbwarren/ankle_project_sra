
clc
clear
close all


L1 = .4;
L2 = L1*sind(60);
l1 = .2;
l2 = .2;
h = .4268;
f = .05;

actuator_length = 0.25;
P = actuator_length*2;

% Desired motion
m = 10000;
tf = 10;
t = linspace(0,tf,m);
theta = 20*sin(115/15*t)*pi/180; 
phi = 15*sin(10*t)*pi/180;
z = .1*sin(.5*t);
H = h+z;


vars = table(L1,L2,l1,l2,theta,phi,H,P,f);
[Pfront,Pleft,Pright] = actuator_positions(vars);

% Pfront = 0*t;
% Pleft = 0*t;
% Pright = 0*t;

Pfront = cat(2,t',Pfront');
Pleft = cat(2,t',Pleft');
Pright = cat(2,t',Pright');

% Changes to the actuator commands because of the way the prismatic joints
% are set up

% Pfront(:,2) = Pfront(:,2) - actuator_length*2;
% Pleft(:,2) = Pleft(:,2) - actuator_length*2;
% Pright(:,2) = Pright(:,2) - actuator_length*2;
% 
% min_length = min(cat(1,Pfront(:,2),cat(1, Pleft(:,2), Pright(:,2))));
% Pfront(:,2) = Pfront(:,2) + abs(min_length);
% Pleft(:,2) = Pleft(:,2) + abs(min_length);
% Pright(:,2) = Pright(:,2) + abs(min_length);


run('run_device_sim.m')