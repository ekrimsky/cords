

close all; clear all; clc; 


% Robot Trajectory for 1 Gait 
% define time vector 
dt = 2.5e-3; 
v_avg = 0.5; % m/s 
d_gait = 0.25;  % m 
t_gait = d_gait/v_avg; 
t = 0:dt:t_gait;   % one gait 
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

x_com = v_avg * t; % constant vel
z_com = h_body * ones(1, N_gait);
theta_com = zeros(1, N_gait);
theta_ddot_com = zeros(1, N_gait); % No angular accel 



swing_idxs = round(t_stance/dt) + 1:N_gait;  % for LH and RF
swing_idxs_a = round(t_stance/dt) + (1:ceil(N_swing/2 - eps));
swing_idxs_b = swing_idxs_a(end) + 1:N_gait; 

RF_x = (l_body/2 + d_gait/2) + zeros(1, N_gait); 
RF_x(swing_idxs_a) = (l_body/2 + d_gait/2) + (2*d_gait/t_swing^2)*(t(swing_idxs_a) - t_stance).^2;
RF_x(swing_idxs_b) = (l_body/2 + d_gait/2) + d_gait - (2*d_gait/t_swing^2)*(t(swing_idxs_b) - t_gait).^2;

RF_z = zeros(1, N_gait);
RF_z(swing_idxs) =  -(4*foot_z_max/t_swing^2)*(t(swing_idxs) - t_stance).*...
								(t(swing_idxs) - t_gait);% parabola 

LH_x = RF_x - l_body; 
LH_z = RF_z; 



% LF -- circshift and then offset 
N_shift = round(N_gait/2); % TODO    % 50% of gait cycle 

%LF_x = circshift(LH_x, N_shift);
LF_x_tmp = [LH_x(1:end - 1), LH_x(end) + LH_x - min(LH_x)];
LF_x = LF_x_tmp(N_shift + 1:N_shift + N_gait);
LF_x = LF_x - max(LF_x - x_com) + (l_body/2 + d_gait/2);
LF_z = circshift(LH_z, N_shift);

RH_x = LF_x - l_body; 
RH_z = LF_z; 

%{
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

del_LH_x = LH_x - (x_com - 0.5*l_body*cos(theta_com));
del_LF_x = LF_x - (x_com + 0.5*l_body*cos(theta_com));
del_RH_x = RH_x - (x_com - 0.5*l_body*cos(theta_com));
del_RF_x = RF_x - (x_com + 0.5*l_body*cos(theta_com));


% Get all the joint angles 
theta1_LF = zeros(N_gait, 1);
theta2_LF = zeros(N_gait, 1);
theta1_LH = zeros(N_gait, 1);
theta2_LH = zeros(N_gait, 1);

theta1_RF = zeros(N_gait, 1);
theta2_RF = zeros(N_gait, 1);
theta1_RH = zeros(N_gait, 1);
theta2_RH = zeros(N_gait, 1);

syms th1 th2;
var_lims = [pi, 2*pi;...
			3*pi/2, 2*pi]; 
for i = 1:N_gait 

	disp(i)

	% Left Front
	eq1 = x_com(i) + (l_body/2)*cos(theta_com(i)) + ...
						l1*cos(th1) + l2*cos(th2) == LF_x(i);
	eq2 = z_com(i) + (l_body/2)*sin(theta_com(i)) + ...
						l1*sin(th1) + l2*sin(th2) == LF_z(i);

	S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
	theta1_LF(i) = double(S.th1);
	theta2_LF(i) = double(S.th2);

	% Left Hind 
	eq1 = x_com(i) - (l_body/2)*cos(theta_com(i)) + ...
						l1*cos(th1) + l2*cos(th2) == LH_x(i);
	eq2 = z_com(i) - (l_body/2)*sin(theta_com(i)) + ...
						l1*sin(th1) + l2*sin(th2) == LH_z(i);
	S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
	theta1_LH(i) = double(S.th1);
	theta2_LH(i) = double(S.th2);

	% Right Front
	eq1 = x_com(i) + (l_body/2)*cos(theta_com(i)) + ...
						l1*cos(th1) + l2*cos(th2) == RF_x(i);
	eq2 = z_com(i) + (l_body/2)*sin(theta_com(i)) + ...
						l1*sin(th1) + l2*sin(th2) == RF_z(i);
	S  = vpasolve([eq1, eq2], [th1, th2], var_lims);
	theta1_RF(i) = double(S.th1);
	theta2_RF(i) = double(S.th2);

	% Right Hind 
	eq1 = x_com(i) - (l_body/2)*cos(theta_com(i)) + ...
						l1*cos(th1) + l2*cos(th2) == RH_x(i);
	eq2 = z_com(i) - (l_body/2)*sin(theta_com(i)) + ...
						l1*sin(th1) + l2*sin(th2) == RH_z(i);
	S  = vpasolve([eq1, eq2], [th1, th2], var_lims);	
	theta1_RH(i) = double(S.th1);
	theta2_RH(i) = double(S.th2);
end 


% Compute Joint Velocities and Accelerations 






% Add in other robot data so can actually solve example 1 



% TODO -- add in nice plot features for the robot somewhere 


% Ssave 
%save('example1_input_data.mat', 'time', 'theta', 'omega',...
%             'omega_dot', 'tau_des'); 
