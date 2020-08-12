
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


% Robot Trajectory for 1 Gait 
% define time vector 
dt = 5e-3; 
v_avg = 0.5; % m/s 
d_gait = 0.25;  % m 
t_gait = d_gait/v_avg; 
t = 0:dt:t_gait;   % one gait 
t = t(:); 
N_gait = length(t);
 
l_body = 0.5; % body length 
h_body = 0.3; % avg body com height 
foot_z_max = h_body/4; 

l1 = (h_body/2)/cos(pi/4);
l2 = l1; 

% walking trot 
t_swing = 0.45 * t_gait;
t_stance = t_gait - t_swing; 
N_swing = t_swing/dt; 

x_com = v_avg * t(:); % constant vel
z_com = h_body * ones(N_gait, 1);
theta_com = zeros(N_gait, 1);
theta_ddot_com = zeros(N_gait, 1); % No angular accel 


swing_idxs = round(t_stance/dt) + 1:N_gait;  % for LH and RF
swing_idxs_a = round(t_stance/dt) + (1:ceil(N_swing/2 - eps));
swing_idxs_b = swing_idxs_a(end) + 1:N_gait; 

RF_x = (l_body/2 + d_gait/2) + zeros(N_gait, 1); 
RF_x(swing_idxs_a) = (l_body/2 + d_gait/2) + (2*d_gait/t_swing^2)*(t(swing_idxs_a) - t_stance).^2;
RF_x(swing_idxs_b) = (l_body/2 + d_gait/2) + d_gait - (2*d_gait/t_swing^2)*(t(swing_idxs_b) - t_gait).^2;

%RF_z = zeros(1, N_gait);
%RF_z(swing_idxs) =  -(4*foot_z_max/t_swing^2)*(t(swing_idxs) - t_stance).*...
%								(t(swing_idxs) - t_gait);% parabola 

points = [0; 0; foot_z_max; 0];
slopes = [0; 0; 0; 0];
t_node = [0; t_stance; t_stance + t_swing/2; t_gait];
RF_z = pm_spline(points, slopes, t_node, t); % TODO name this something better 

LH_x = RF_x - l_body; 
LH_z = RF_z; 



% LF -- circshift and then offset 
N_shift = round(N_gait/2); % shift 50% of gait cycle 
LF_x_tmp = [LH_x(1:end - 1); LH_x(end) + LH_x - min(LH_x)];
LF_x = LF_x_tmp(N_shift + 1:N_shift + N_gait);
LF_x = LF_x - max(LF_x - x_com) + (l_body/2 + d_gait/2);
LF_z = circshift(LH_z, N_shift);

RH_x = LF_x - l_body; 
RH_z = LF_z; 


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


% From here we want.....

% gamma -- new var 
% DELTA (not using anymore because leave out MLD Approach)
del_RF_x = RF_x - (x_com + 0.5*l_body*cos(theta_com));
del_RH_x = RH_x - (x_com - 0.5*l_body*cos(theta_com));
del_LF_x = LF_x - (x_com + 0.5*l_body*cos(theta_com));
del_LH_x = LH_x - (x_com - 0.5*l_body*cos(theta_com));


% Get all the joint angles 
theta_rfs = zeros(N_gait, 1);
theta_rfk = zeros(N_gait, 1);
theta_rhs = zeros(N_gait, 1);
theta_rhk = zeros(N_gait, 1);

theta_lfs = zeros(N_gait, 1);
theta_lfk = zeros(N_gait, 1);
theta_lhs = zeros(N_gait, 1);
theta_lhk = zeros(N_gait, 1);


syms th1 th2;
var_lims = [pi, 2*pi;...    % prevent wrong knee orientation 
			3*pi/2, 2*pi]; 
for i = 1:N_gait 
	% Right Front
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


% Compute Joint Velocities and Accelerations 
time = t(:); 

% Robot Body Velocities + Accelerations 
theta_com_dot = central_diff(theta_com, time); 
theta_com_ddot = central_diff(theta_com_dot, time);

x_com_dot = central_diff(x_com, time);
x_com_ddot = central_diff(x_com_dot, time);

z_com_dot = central_diff(z_com, time);
z_com_ddot = central_diff(z_com_dot, time); 

% Joint Velocities 
omega_rfs = central_diff(theta_rfs, time);
omega_rfk = central_diff(theta_rfk, time);
omega_rhs = central_diff(theta_rhs, time);
omega_rhk = central_diff(theta_rhk, time);

omega_lfs = central_diff(theta_lfs, time);
omega_lfk = central_diff(theta_lfk, time);
omega_lhs = central_diff(theta_lhs, time);
omega_lhk = central_diff(theta_lhk, time);

% Joint Accelerations 
omega_dot_rfs = central_diff(omega_rfs, time);
omega_dot_rfk = central_diff(omega_rfk, time);
omega_dot_rhs = central_diff(omega_rhs, time);
omega_dot_rhk = central_diff(omega_rhk, time);

omega_dot_lfs = central_diff(omega_lfs, time);
omega_dot_lfk = central_diff(omega_lfk, time);
omega_dot_lhs = central_diff(omega_lhs, time);
omega_dot_lhk = central_diff(omega_lhk, time);



%% Save out to mat file 
save('example2_input_data.mat',...
         'time', ...
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




figure, plot(time, theta_rhs, 'b');
figure, plot(time, omega_rhs, 'b');
figure, plot(time, omega_dot_rhs, 'b');

figure, plot(time, theta_rhk, 'g');
figure, plot(time, omega_rhk, 'g');
figure, plot(time, omega_dot_rhk, 'g');