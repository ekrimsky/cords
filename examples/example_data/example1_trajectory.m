clear vars; 


dt = 0.01;   % TODO -- make finer later  
theta_min = deg2rad(-45); 
theta_max = deg2rad(45); 

% Sweeping a robot arm +/- 45 degrees with gravity 
g = 9.81; 
l = 0.3; 
m = 1; % unit mass, will scale torques 
J_arm = 0.5 * m * l^2; 

time = 0:dt:1; 
n = length(time); 
theta = 0.5*((theta_min + theta_max) - cos(2*pi*time)*(theta_max - theta_min)); 

omega = central_diff(theta, time); 
omega_dot = central_diff(omega, time); 

tau_des = omega_dot * J_arm + g*l*cos(theta); 

tau_des = 10 * tau_des/max(abs(tau_des)); % make max torque 10 Nm 


% may want to do 2 trajectories with different frequencies 
save('example1_input_data.mat', 'time', 'theta', 'omega',...
             'omega_dot', 'tau_des'); 


