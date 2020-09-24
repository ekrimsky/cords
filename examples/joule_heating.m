% TODO - comment 

%
addpath('examples/example_data');
load('parallel_elastic_input_data.mat', 'time',  'theta', 'omega',...
                                'omega_dot', 'tau_des'); 
data = struct();        % empty struct with no fields that we will fill in 
data.omega = omega;
data.omega_dot = omega_dot; 
data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
data.c0 = [];
data.M0 = [];
data.r0 = [];
data.beta0 = []; 
data.I_max = 80;     % set 80 Amp current limit
data.V_max = 24;     % set 24 Volt voltage limit 
data.tau_c = tau_des;
data.T = [];         % simple case, no x variable 

prob = cords();                  % create a new cords object with default settings  
prob.update_problem(data);       % attach the data to the problem
solutions = prob.optimize(10);   % get the 10 best motor/gearbox combinations 


% Now try with parallel elastic component
data.T = [-theta(:)];      % include coupling of motor torque and spring torque
prob.update_problem(data);       % attach the data to the problem
solutions_pe = prob.optimize(10);   % get the 10 best motor/gearbox combinations 
