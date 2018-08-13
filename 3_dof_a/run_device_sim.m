%% Script for automating design iterations on the platform

%% Settings for the sim
% Sim length
tf = 10;

% Positional input

% Frontal rotation
frontal_amp = 0*0.01; %0*0.06;
frontal_frq = 10;

% Sagittal rotation
sagittal_amp = 0*0.05; %0*.115;
sagittal_frq = 1;
sagittal_offset = -.1; %-.15; %-.2;

% Step
step_times = [0 4.8 5];
step_magn = [-.1 .1 -.1]; 


% Materials
aluminum_rho = 2700;


% Device dimensions

% Legs
foot_w = 5; % cm
foot_l = foot_w;
foot_h = foot_w;

actuator_base_l = 30; % cm
actuator_base_w = 5; % cm
actuator_base_h = actuator_base_w;

actuator_extender_l = 30; % cm
actuator_extender_w = 3;
actuator_extender_h = actuator_extender_w;

% Leg positioning
r = sqrt(2000)/2+20; % legs feet are placed a distance r away from the center of the base
theta_r = 120;
theta_l = -120;


%% Kinematics

h = .6168; % This was just measured from the zero position in the sim
% h = .511;
P = actuator_base_l + actuator_extender_l; P = P/100; % P = actuator_base_l/100;

% Desired motion
t = linspace(0,tf,10000);
theta = 0*sin(2*t)*pi/180 - 0*pi/180;
phi = 0*sin(4*t)*pi/180 + 0*pi/180;
z = .2*sin(t);

vars = table(r,theta,phi,h,P,z);
[P1,P2,P3] = position(vars);

% P1 = 0*t; P2 = P1; P3 = P1;
P1 = cat(2,t',P1');
P2 = cat(2,t',P2');
P3 = cat(2,t',P3');

%% Run the sim

sim('RPS3Dof.slx')

