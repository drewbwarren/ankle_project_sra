%% Script for automating design iterations on the platform

%% Settings for the sim
% Sim length
tf = 10;

% Positional input

% Frontal rotation
frontal_amp = 0.01; %0*0.06;
frontal_frq = 10;

% Sagittal rotation
sagittal_amp = 0.05; %0*.115;
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

actuator_base_l = 40; % cm
actuator_base_w = 5; % cm
actuator_base_h = actuator_base_w;

actuator_extender_l = 40; % cm
actuator_extender_w = 3;
actuator_extender_h = actuator_extender_w;

% Leg positioning
r = sqrt(2000)/2+20; % legs feet are placed a distance r away from the center of the base
theta_r = 120;
theta_l = -120;


%% Run the sim

sim('RPS3Dof.slx')



%% Extract data about the ankle

% t = ankle_p.time;
% 
% % Orientation - deg
% ankle_roll = ankle_p.data(:,1,:);
% ankle_pitch = ankle_p.data(:,2,:);
% ankle_yaw = ankle_p.data(:,3,:);
% clear ankle_p
% max_roll = max(ankle_roll)
% max_pitch = max(ankle_pitch)
% 
% % Velocty - deg/s
% ankle_wx = ankle_w.data(:,1,:);
% ankle_wy = ankle_w.data(:,2,:);
% ankle_wz = ankle_w.data(:,3,:);
% clear ankle_w
% max_wx = max(ankle_wx)
% max_wy = max(ankle_wy)
% 
% % Actuation force - units should be N, but I should double check
% actuator_force_left = actuation_force.data(:,1,:);
% actuator_force_back = actuation_force.data(:,2,:);
% actuator_force_right = actuation_force.data(:,3,:);
% clear actuation_force
% max_f_left = max(actuator_force_left)
% max_f_back = max(actuator_force_back)
% max_f_right= max(actuator_force_right)