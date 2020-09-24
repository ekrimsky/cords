
%
%
%
%		Some explanation here 
%
%
%
%
%
%
close all; clear all; clc; 

%
%
%        Walking Trajectory 
%
%

% Robot Trajectory for 1 Gait 
% define time vector 
g = 9.81;   % m/s^2 
dt = 1e-2; 
v_avg = 0.5; % m/s 
d_gait = 0.25;  % m 
T_gait = d_gait/v_avg; 
t_gait = 0:dt:T_gait;   % one gait 
t_gait = t_gait(:); 
N_gait = length(t_gait);
 
l_body = 0.5; % body length 
h_body = 0.3; % avg body com height 
foot_z_max = h_body/4; 

l1 = (h_body/2)/cos(pi/4);
l2 = l1; 

% walking trot 
T_swing = 0.45 * T_gait;
T_stance = T_gait - T_swing; 
N_swing = T_swing/dt; 

x_com_gait = v_avg * t_gait(:); % constant vel
z_com_gait = h_body * ones(N_gait, 1);
theta_com_gait = zeros(N_gait, 1);
theta_ddot_com_gait = zeros(N_gait, 1); % No angular accel 


swing_idxs = round(T_stance/dt) + 1:N_gait;  % for LH and RF
swing_idxs_a = round(T_stance/dt) + (1:ceil(N_swing/2 - eps));
swing_idxs_b = swing_idxs_a(end) + 1:N_gait; 

RF_x_gait = (l_body/2 + d_gait/2) + zeros(N_gait, 1); 
RF_x_gait(swing_idxs_a) = (l_body/2 + d_gait/2) + (2*d_gait/T_swing^2)*(t_gait(swing_idxs_a) - T_stance).^2;
RF_x_gait(swing_idxs_b) = (l_body/2 + d_gait/2) + d_gait - (2*d_gait/T_swing^2)*(t_gait(swing_idxs_b) - T_gait).^2;

points = [0; 0; foot_z_max; 0];
slopes = [0; 0; 0; 0];
t_node = [0; T_stance; T_stance + T_swing/2; T_gait];
RF_z_gait = pm_spline(points, slopes, t_node, t_gait); % TODO name this something better 

LH_x_gait = RF_x_gait - l_body; 
LH_z_gait = RF_z_gait; 



% LF -- circshift and then offset 
N_shift = round(N_gait/2); % shift 50% of gait cycle 
LF_x_tmp = [LH_x_gait(1:end - 1); LH_x_gait(end) + LH_x_gait - min(LH_x_gait)];
LF_x_gait = LF_x_tmp(N_shift + 1:N_shift + N_gait);
LF_x_gait = LF_x_gait - max(LF_x_gait - x_com_gait) + (l_body/2 + d_gait/2);
LF_z_gait = circshift(LH_z_gait, N_shift);

RH_x_gait = LF_x_gait - l_body; 
RH_z_gait = LF_z_gait; 

%{
% TODO - replace this with the robot animatino without grfs 
figure, hold on 
axis equal
for i = 1:N_gait
	plot(x_com(i) + 0.5*l_body*cos(theta_com(i)),...
					z_com(i) + 0.5*l_body*sin(theta_com(i)), 'kx')
	plot(x_com(i) - 0.5*l_body*cos(theta_com(i)),...
					z_com(i) - 0.5*l_body*sin(theta_com(i)), 'cs')

 	plot(RF_x(i), RF_z(i), 'ro');
 	plot(RH_x(i), RH_z(i), 'ys');

 	plot(LF_x(i), LF_z(i), 'bo');
 	plot(LH_x(i), LH_z(i), 'gs');


 	xlim(x_com(i) + [-1, 1]);
 	ylim([-0.1, 0.4]);

 	pause(5*dt)
end 
%} 

% From here we want.....

% gamma -- new var 
% DELTA (not using anymore because leave out MLD Approach)



% Get all the joint angles 
theta_rfs = zeros(N_gait, 1);
theta_rfk = zeros(N_gait, 1);
theta_rhs = zeros(N_gait, 1);
theta_rhk = zeros(N_gait, 1);

theta_lfs = zeros(N_gait, 1);
theta_lfk = zeros(N_gait, 1);
theta_lhs = zeros(N_gait, 1);
theta_lhk = zeros(N_gait, 1);

[theta_rfs_gait, theta_rfk_gait, theta_rhs_gait, theta_rhk_gait,...
   theta_lfs_gait, theta_lfk_gait, theta_lhs_gait, theta_lhk_gait] = ...
         compute_joint_angles(l_body, l1, l2, theta_com_gait, x_com_gait,...
                  z_com_gait, RF_x_gait, RF_z_gait, RH_x_gait, RH_z_gait,...
                              LF_x_gait, LF_z_gait, LH_x_gait, LH_z_gait); 



% Compute Joint Velocities and Accelerations 

% Robot Body Velocities + Accelerations 
theta_com_dot_gait = central_diff(theta_com_gait, t_gait); 
theta_com_ddot_gait = central_diff(theta_com_dot_gait, t_gait);

x_com_dot_gait = central_diff(x_com_gait, t_gait);
x_com_ddot_gait = central_diff(x_com_dot_gait, t_gait);

z_com_dot_gait = central_diff(z_com_gait, t_gait);
z_com_ddot_gait = central_diff(z_com_dot_gait, t_gait); 

% Joint Velocities 
omega_rfs_gait = central_diff(theta_rfs_gait, t_gait);
omega_rfk_gait = central_diff(theta_rfk_gait, t_gait);
omega_rhs_gait = central_diff(theta_rhs_gait, t_gait);
omega_rhk_gait = central_diff(theta_rhk_gait, t_gait);

omega_lfs_gait = central_diff(theta_lfs_gait, t_gait);
omega_lfk_gait = central_diff(theta_lfk_gait, t_gait);
omega_lhs_gait = central_diff(theta_lhs_gait, t_gait);
omega_lhk_gait = central_diff(theta_lhk_gait, t_gait);

% Joint Accelerations 
omega_dot_rfs_gait = central_diff(omega_rfs_gait, t_gait);
omega_dot_rfk_gait = central_diff(omega_rfk_gait, t_gait);
omega_dot_rhs_gait = central_diff(omega_rhs_gait, t_gait);
omega_dot_rhk_gait = central_diff(omega_rhk_gait, t_gait);

omega_dot_lfs_gait = central_diff(omega_lfs_gait, t_gait);
omega_dot_lfk_gait = central_diff(omega_lfk_gait, t_gait);
omega_dot_lhs_gait = central_diff(omega_lhs_gait, t_gait);
omega_dot_lhk_gait = central_diff(omega_lhk_gait, t_gait);



% 
%        Jump trajectory 
%
%        Accelerate upward at 3g for 0.03 seconds from crouched position 
%        Assume start with com at 0.15 m 

t_jump = (0:0.01:0.03)';   % just a few time points 

N_jump = length(t_jump); 
z_com_jump_init = 0.15; 
z_com_ddot_jump = 3*g*ones(N_jump, 1); 
z_com_dot_jump = cumtrapz(t_jump, z_com_ddot_jump);
z_com_jump = z_com_jump_init + cumtrapz(t_jump, z_com_dot_jump);

x_com_jump = zeros(N_jump, 1);
x_com_dot_jump = zeros(N_jump, 1);
x_com_ddot_jump = zeros(N_jump, 1);

theta_com_jump = zeros(N_jump, 1);
theta_com_dot_jump = zeros(N_jump, 1);
theta_com_ddot_jump = zeros(N_jump, 1);

% Feet directly under shoulders 
RF_x_jump = 0.5 * l_body * ones(N_jump, 1);
RF_z_jump = zeros(N_jump, 1);
RH_x_jump = -0.5 * l_body * ones(N_jump, 1);
RH_z_jump = zeros(N_jump, 1);

LF_x_jump = 0.5 * l_body * ones(N_jump, 1);
LF_z_jump = zeros(N_jump, 1);
LH_x_jump = -0.5 * l_body * ones(N_jump, 1);
LH_z_jump = zeros(N_jump, 1);


[theta_rfs_jump, theta_rfk_jump, theta_rhs_jump, theta_rhk_jump,...
   theta_lfs_jump, theta_lfk_jump, theta_lhs_jump, theta_lhk_jump] = ...
         compute_joint_angles(l_body, l1, l2, theta_com_jump, x_com_jump,...
                  z_com_jump, RF_x_jump, RF_z_jump, RH_x_jump, RH_z_jump,...
                              LF_x_jump, LF_z_jump, LH_x_jump, LH_z_jump); 

% Jump Joint Velocities 
omega_rfs_jump = central_diff(theta_rfs_jump, t_jump);
omega_rfk_jump = central_diff(theta_rfk_jump, t_jump);
omega_rhs_jump = central_diff(theta_rhs_jump, t_jump);
omega_rhk_jump = central_diff(theta_rhk_jump, t_jump);

omega_lfs_jump = central_diff(theta_lfs_jump, t_jump);
omega_lfk_jump = central_diff(theta_lfk_jump, t_jump);
omega_lhs_jump = central_diff(theta_lhs_jump, t_jump);
omega_lhk_jump = central_diff(theta_lhk_jump, t_jump);

% Jump Joint Accelerations 
omega_dot_rfs_jump = central_diff(omega_rfs_jump, t_jump);
omega_dot_rfk_jump = central_diff(omega_rfk_jump, t_jump);
omega_dot_rhs_jump = central_diff(omega_rhs_jump, t_jump);
omega_dot_rhk_jump = central_diff(omega_rhk_jump, t_jump);

omega_dot_lfs_jump = central_diff(omega_lfs_jump, t_jump);
omega_dot_lfk_jump = central_diff(omega_lfk_jump, t_jump);
omega_dot_lhs_jump = central_diff(omega_lhs_jump, t_jump);
omega_dot_lhk_jump = central_diff(omega_lhk_jump, t_jump);



%
%
%        Add the jump trajectory onto the end of gait trajectory 
%
%
%


theta_com = [theta_com_gait; theta_com_jump];
theta_com_dot = [theta_com_dot_gait; theta_com_dot_jump];
theta_com_ddot = [theta_com_ddot_gait; theta_com_ddot_jump];

x_com = [x_com_gait; x_com_jump];
x_com_dot = [x_com_dot_gait; x_com_dot_jump];
x_com_ddot = [x_com_ddot_gait; x_com_ddot_jump];

z_com = [z_com_gait; z_com_jump];
z_com_dot = [z_com_dot_gait; z_com_dot_jump];
z_com_ddot = [z_com_ddot_gait; z_com_ddot_jump];

theta_rfs = [theta_rfs_gait; theta_rfs_jump];
theta_rfk = [theta_rfk_gait; theta_rfk_jump];
theta_rhs = [theta_rhs_gait; theta_rhs_jump];
theta_rhk = [theta_rhk_gait; theta_rhk_jump];
theta_lfs = [theta_lfs_gait; theta_lfs_jump];
theta_lfk = [theta_lfk_gait; theta_lfk_jump];
theta_lhs = [theta_lhs_gait; theta_lhs_jump];
theta_lhk = [theta_lhk_gait; theta_lhk_jump];

omega_rfs = [omega_rfs_gait; omega_rfs_jump];
omega_rfk = [omega_rfk_gait; omega_rfk_jump];
omega_rhs = [omega_rhs_gait; omega_rhs_jump];
omega_rhk = [omega_rhk_gait; omega_rhk_jump];
omega_lfs = [omega_lfs_gait; omega_lfs_jump];
omega_lfk = [omega_lfk_gait; omega_lfk_jump];
omega_lhs = [omega_lhs_gait; omega_lhs_jump];
omega_lhk = [omega_lhk_gait; omega_lhk_jump];

omega_dot_rfs = [omega_dot_rfs_gait; omega_dot_rfs_jump];
omega_dot_rfk = [omega_dot_rfk_gait; omega_dot_rfk_jump];
omega_dot_rhs = [omega_dot_rhs_gait; omega_dot_rhs_jump];
omega_dot_rhk = [omega_dot_rhk_gait; omega_dot_rhk_jump];
omega_dot_lfs = [omega_dot_lfs_gait; omega_dot_lfs_jump];
omega_dot_lfk = [omega_dot_lfk_gait; omega_dot_lfk_jump];
omega_dot_lhs = [omega_dot_lhs_gait; omega_dot_lhs_jump];
omega_dot_lhk = [omega_dot_lhk_gait; omega_dot_lhk_jump];


RF_x = [RF_x_gait; RF_x_jump];
RF_z = [RF_z_gait; RF_z_jump];
RH_x = [RH_x_gait; RH_x_jump];
RH_z = [RH_z_gait; RH_z_jump];

LF_x = [LF_x_gait; LF_x_jump];
LF_z = [LF_z_gait; LF_z_jump];
LH_x = [LH_x_gait; LH_x_jump];
LH_z = [LH_z_gait; LH_z_jump];


del_RF_x = RF_x - (x_com + 0.5*l_body*cos(theta_com));
del_RH_x = RH_x - (x_com - 0.5*l_body*cos(theta_com));
del_LF_x = LF_x - (x_com + 0.5*l_body*cos(theta_com));
del_LH_x = LH_x - (x_com - 0.5*l_body*cos(theta_com));


%% Save out to mat file 
save('example2_input_data.mat',...
         't_gait', ...
         't_jump',...
         'l_body',... 	% robot length (front to back shouler)
         'l1',... 		% shoulder to knee link length 
         'l2',... 		% knee to foot link length 
         'theta_com',... % robot orientation trajectory 
         'theta_com_dot',... % robot angular vel
         'theta_com_ddot',... % robot angular accel
         'x_com',...        % x com trajectory 
         'x_com_dot',...    % robot x vel
         'x_com_ddot',...   % robot x accel
         'z_com',....       % z com trajetory 
         'z_com_dot',...    % robot z vel 
         'z_com_ddot',...   % robot z accel 
         'theta_rfs',... % right front shoulder 
         'theta_rfk',... % right front knee
         'theta_rhs',... % right hind shoulder 
         'theta_rhk',... % right hind knee 
         'theta_lfs',... % left front shoulder 
         'theta_lfk',... % left front knee
         'theta_lhs',... % left hind shoulder 
         'theta_lhk',... % left hind knee 
    ...  % Joint Velocities  
         'omega_rfs',... % right front shoulder 
         'omega_rfk',... % right front knee
         'omega_rhs',... % right hind shoulder 
         'omega_rhk',... % right hind knee 
         'omega_lfs',... % left front shoulder 
         'omega_lfk',... % left front knee
         'omega_lhs',... % left hind shoulder 
         'omega_lhk',... % left hind knee 
    ...  % Joint Accels 
         'omega_dot_rfs',... % right front shoulder 
         'omega_dot_rfk',... % right front knee
         'omega_dot_rhs',... % right hind shoulder 
         'omega_dot_rhk',... % right hind knee 
         'omega_dot_lfs',... % left front shoulder 
         'omega_dot_lfk',... % left front knee
         'omega_dot_lhs',... % left hind shoulder 
         'omega_dot_lhk',... % left hind knee 
    ...  % Foot placements 
         'RF_x',...
         'RF_z',...
         'RH_x',...
         'RH_z',...
         'LF_x',...
         'LF_z',...
         'LH_x',...
         'LH_z',...
    ...  % Foot relative to shoulder for computing moments 
         'del_RF_x',...
         'del_RH_x',...
         'del_LF_x',...
         'del_LH_x'); 



%{
figure, plot(t_gait, theta_rhs, 'b');
figure, plot(t_gait, omega_rhs, 'b');
figure, plot(t_gait, omega_dot_rhs, 'b');

figure, plot(t_gait, theta_rhk, 'g');
figure, plot(t_gait, omega_rhk, 'g');
figure, plot(t_gait, omega_dot_rhk, 'g');
%} 




function [theta_rfs, theta_rfk, theta_rhs, theta_rhk, ...
          theta_lfs, theta_lfk, theta_lhs, theta_lhk] = ...
         compute_joint_angles(l_body, l1, l2, theta_com, ...
                              x_com, z_com,...
                                RF_x, RF_z, RH_x, RH_z,...
                                LF_x, LF_z, LH_x, LH_z)

   syms th1 th2;
   var_lims = [pi, 2*pi;...    % prevent wrong knee orientation 
         3*pi/2, 2*pi]; 

   N = length(x_com);
   theta_rfs = zeros(N, 1);
   theta_rfk = zeros(N, 1);
   theta_rhs = zeros(N, 1);
   theta_rhk = zeros(N, 1);
   theta_lfs = zeros(N, 1);
   theta_lfk = zeros(N, 1);
   theta_lhs = zeros(N, 1);
   theta_lhk = zeros(N, 1);

   for i = 1:N
      eq1 = x_com(i) + (l_body/2)*cos(theta_com(i)) + ...
                     l1*cos(th1) + l2*cos(th2) == RF_x(i);
      eq2 = z_com(i) + (l_body/2)*sin(theta_com(i)) + ...
                     l1*sin(th1) + l2*sin(th2) == RF_z(i);
      S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
      theta_rfs(i) = double(S.th1);
      theta_rfk(i) = double(S.th2);

      % Right Hind 
      eq1 = x_com(i) - (l_body/2)*cos(theta_com(i)) + ...
                     l1*cos(th1) + l2*cos(th2) == RH_x(i);
      eq2 = z_com(i) - (l_body/2)*sin(theta_com(i)) + ...
                     l1*sin(th1) + l2*sin(th2) == RH_z(i);
      S  = vpasolve([eq1, eq2], [th1, th2], var_lims);   
      theta_rhs(i) = double(S.th1);
      theta_rhk(i) = double(S.th2);


      % Left Front
      eq1 = x_com(i) + (l_body/2)*cos(theta_com(i)) + ...
                     l1*cos(th1) + l2*cos(th2) == LF_x(i);
      eq2 = z_com(i) + (l_body/2)*sin(theta_com(i)) + ...
                     l1*sin(th1) + l2*sin(th2) == LF_z(i);

      S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
      theta_lfs(i) = double(S.th1);
      theta_lfk(i) = double(S.th2);

      % Left Hind 
      eq1 = x_com(i) - (l_body/2)*cos(theta_com(i)) + ...
                     l1*cos(th1) + l2*cos(th2) == LH_x(i);
      eq2 = z_com(i) - (l_body/2)*sin(theta_com(i)) + ...
                     l1*sin(th1) + l2*sin(th2) == LH_z(i);
      S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
      theta_lhs(i) = double(S.th1);
      theta_lhk(i) = double(S.th2);
   end 
end 