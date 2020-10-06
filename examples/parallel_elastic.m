
%
%
%       Big block comment AND problem statement repeated word for word 
%
%
%
%
%
% See also example1_simple
clearvars; clc; close all; 
% Another idea for the website 
% ... have pictures and jupiter notebook type 
% deal where we walk through each of the examples 

% Get Trajectory Data 
addpath('examples/example_data');


%....fill in  

% defining a desired trajectory under gravity 

% Fill in with rando stuff -- TODO - load in from file 

% TODO -- package up so can refenerence correctly 
load('parallel_elastic_input_data.mat', 'time',  'theta', 'omega',...
                                'omega_dot', 'tau_des'); 
theta = theta(:); % col vec
omega = omega(:);
omega_dot = omega_dot(:);
time = time(:); 
tau_des = tau_des(:); 
n = length(time); 

% Fill in all the problem data parts 

% Define aux variable x and the indices into it 
% x = [kp, tau_c, t]




w = 2 + n; % dimensionality of x vector
kp_idx = 1; 
tau_offset_idx = 2; 
pwr_idxs = 3:(3 + n - 1);  % indices corresponding to power consumed at each time step

% Block comment at the begining of each of these would be good 




%
%                       Fill in Objective (i.e. Costs)
%
M0 = []; 
p0 = []; 
c0 = []; 
f0 = zeros(w, 1);
f0(pwr_idxs(1):pwr_idxs(end - 1)) = 0.5*ones(n - 1, 1).*diff(time);
f0(pwr_idxs(2):pwr_idxs(end)) = f0(pwr_idxs(2):pwr_idxs(end)) + 0.5*ones(n - 1, 1).*diff(time);
beta0 = 0; 


% Define efficiencies psi, 


%
%   TODO -- make this cleaner 
%       
%   Power consumption is given by P = IV where I = current, V = Voltage
%
%   From KVL for the motor loop we can relate the motor speed to the
%   current and voltage by 
%
%   V = IR + k_t*omega_motor     (where k_t, the torque constant is the same as the 
%   the back-emf constant in SI Units )
%
%   omega_motor = gearbox.direction*gearbox.ratio*omega_joint, omega_joint
%   is the velocity we are telling to the solver 
%
%   So, idally we could write our power consumption as 
%       I(IR + k_t*omega_motor)= I^2 R + I * k_t*omega_motor
%
%   When we are drawing positive electrical power from the battery, we have 
%   efficiecy losses in the driver boards where 0 < Phi <= 1 is the driver
%   board efficiency, so we would state the actual power draw as:
%       (1/Phi) * I^2 R + I*k_t*omega_motor
%
%   When we are using the motor as a generator and sending current back
%   to the battery we have additional losses associated with charging 
%   the battery (this depends both on the cicuitry and battery type)
%
%   In this case the (negative) power draw is:
%         Psi* I^2 R + I*k_t*omega_motor where Psi the regeneration 
%   efficiency (0 <= Psi < 1). We could set Psi to zero for resistive braking
%
%   We can combine these equations to say that the power draw, P 
%   is given by:
%       P = max{(1/Phi) (I^2 R + I*k_t*omega_motor), Psi*(I^2 R + k_t*omega_motor)}
%   This can be encoded in differentiable form by introducing a new varialbe, P,
%   for the power draw at each time step and adding the inwequalities 
%
%       P >= (1/Phi) (I^2 R + I*k_t*omega_motor)
%               AND
%       P >=  Psi*(I^2 R + k_t*omega_motor)
%
%   Now to get the total power consumed over the trajectory we perform
%   trapezoidal integration the P variables 
%   
%



Phi = 0.99; % driver board efficienc 
Phi_inv = 1/Phi; 
Psi = 0.8; % regen efficincy psi 



% Comment here should then explain what each row of this inequality looks like 
tmp_mat = sparse(eye(n)); 
P = @(motor, gearbox) motor.R * [Phi_inv * tmp_mat; Psi * tmp_mat]; 
omega_diag = sparse(diag(omega));
C = @(motor, gearbox) motor.k_t*gearbox.direction*gearbox.ratio * ...
                                        [Phi_inv*omega_diag; Psi*omega_diag]; 

% NOTE on how it can be faster to declare like this 
F_tmp = sparse(1:2*n, [pwr_idxs, pwr_idxs], -ones(1, 2*n), 2*n, w);
F = F_tmp; 
bet = zeros(2*n, 1); % no constant offsets 

%% Add bounds on the spring properties (will make solving easier )
x_lb = -inf(w, 1);
x_ub = inf(w, 1); 
x_lb([kp_idx, tau_offset_idx]) = [-20; -20];
x_ub([kp_idx, tau_offset_idx]) = [20, 20]; 



% Fill in Torque Constraints 
T = @(~, ~) [theta,  ones(n,1), zeros(n, w - 2)]; 
tau_c = tau_des;   % inertial compensation done in optimizer 

V_max = 48; % volts  
I_max = 100;  % Amps 




%% Fill in trajectory and desired torque data 
problem_data.T = T;
problem_data.tau_c = tau_c; 
problem_data.omega = omega;
problem_data.omega_dot = omega_dot; 

problem_data.I_max = I_max; 
problem_data.V_max = V_max; 

% Fill in Objective
problem_data.p0 = p0;
problem_data.c0 = c0;
problem_data.f0 = f0;
problem_data.beta0 = beta0;
problem_data.M0 = M0; 

% TODO add M0 

%% Fill in other constraints 
problem_data.P = P;
problem_data.C = C; 
problem_data.F = F; 
problem_data.beta = bet; 

problem_data.x_lb = x_lb;
problem_data.x_ub = x_ub; 



% Solve the problem 
%settings.solver = 'ecos'; 
settings.solver = 'gurobi';
prob = cords('settings', settings, 'reuse_db', true); % instantiate new motor selection problem 
prob.update_problem(problem_data);   % update with the data 

% Lets set a mass limit of 8 kg 
filters.total_mass = 8;
% only look at motors/gearboxes from maxon + faulhaber
filters.manufacturer = {'Maxon', 'Faulhaber'}; 
prob.update_filters(filters); 


% Need to think about what return types should be 
% What inputs would make sense????? 
num_solutions = 100;
sol_struct = prob.optimize(num_solutions);




%save(fullfile('examples', 'parallel_elastic_results.mat')); 

%energy = 

% TODO - would be good to return current AND aux var separately 
% also want to return cost -- how to return motors is harder question 
% NOTE: could be good to have a seperate return of motor torque 
% even though it could be calculated externally 


% Plot results - animate would be nice rtoo 

% show break up of torque 



% define one-hot in src 
save('examples/parallel_elastic_results.mat'); 


opt_struct = sol_struct(1); 


% Extract Results 
kp = opt_struct.sol.x(kp_idx)
tau_offset = opt_struct.sol.x(tau_offset_idx);
theta_0 = tau_offset/kp; 


opt_motor = opt_struct.motor; 
opt_gearbox = opt_struct.gearbox; 

% break down parameters of solutions 
opt_sol = opt_struct.sol;
x = opt_sol.x; 
I = opt_sol.I;

pwr = x(pwr_idxs); 

spring_torque = -kp*theta - tau_offset; 





