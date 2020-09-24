
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

omega(omega == 0) = -1e-12;  % to help debug something 

%disp('fix velocities ')
%omega = abs(omega) + 0.1; 


% NOTE: could do 2 trajectories 
% Fill in all the problem data parts 

% Define aux variable x and the indices into it 
% x = [kp, tau_c, t]

w = 2 + n; % dimensionality of x vector
kp_idx = 1; 
tau_offset_idx = 2; 
pwr_idxs = 3:(3 + n - 1);  % indices corresponding to power consumed at each time step

% Block comment at the begining of each of these would be good 


% Initialize Cell Arrays 



% Fill in Objective --- slpit out ibjective 
Q0 = []; 
c0 = []; 
M0 = [];

r0 = zeros(w, 1);
r0(pwr_idxs(1):pwr_idxs(end - 1)) = 0.5*ones(n - 1, 1).*diff(time);
r0(pwr_idxs(2):pwr_idxs(end)) = r0(pwr_idxs(2):pwr_idxs(end)) + 0.5*ones(n - 1, 1).*diff(time);
beta0 = 0; 


% Define efficiencies psi, 

Q = {}; 
c = {};  
M = {};
r = {}; 
bet = {};




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
% Fill in general inequality constraints 
for j = 1:n  % Loop through the time indices 

    % If positive power conumption 
    e_j = zeros(n, 1); 
    e_j(j) = 1; % one hot vector / unit basis vector 
    e_js = sparse(e_j);
    % NOTE  -- would it be more efficient to divide through by R so no Q dependednce on inputs? Still r and c dependence though 

    % Q can either take in a sparse diagonal matrix OR 
    % a non-sparse vector specifying the diagoan

    r_tmp = [0; 0; -e_j];  % This picks out the specific power consumption value at this time step

    Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * e_j; % vector type input 
    c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_t*gearbox.direction*gearbox.ratio*omega(j)*e_j; % TODO -- should need to account for direction 
    
    M{end + 1, 1} = []; 
    r{end + 1, 1} = r_tmp;  % accounting for the other vars 
    bet{end + 1, 1} = 0; 

    % If negative power consumption


    %Q{end + 1, 1} = @(motor, ~) Psi*motor.R * spdg_j; % sparse matrix type input
    Q{end + 1, 1} = @(motor, ~) Psi*motor.R * e_j; % sparse matrix type input 
    c{end + 1, 1} = @(motor, gearbox) Psi*motor.k_t*gearbox.direction*gearbox.ratio*omega(j)*e_j;     
    M{end + 1, 1} = []; 
    r{end + 1, 1} = r_tmp;  % accounting for the other vars 
    bet{end + 1, 1} = 0; 
end 


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
problem_data.Q0 = Q0;
problem_data.M0 = M0;
problem_data.c0 = c0;
problem_data.r0 = r0;
problem_data.beta0 = beta0;

%% Fill in other constraints 
problem_data.Q = Q;
problem_data.c = c; 
problem_data.M = M; 
problem_data.r = r; 
problem_data.beta = bet; 
problem_data.x_lb = x_lb;
problem_data.x_ub = x_ub; 



% Solve the problem 

%settings.solver = 'ecos';   % if we wanted to use ecos 
settings.solver = 'gurobi';
prob = cords('settings', settings, 'reuse_db', false); % instantiate new motor selection problem 
prob.update_problem(problem_data);   % update with the data 

%{
prob.filter_motors 
prob.filter_gears 

% consider whats the best way to do this 
prob.hints = % HOW BEST TO DO THIS? -- TODO 
%} 

% Need to think about what return types should be 
% What inputs would make sense????? 
num_solutions = 100;
sol_struct = prob.optimize(num_solutions);

sol = sol_struct(1).sol; 


% Extract Results 
kp = sol.x(kp_idx)
tau_offset = sol.x(tau_offset_idx);
theta_0 = tau_offset/kp; 


%save(fullfile('examples', 'parallel_elastic_results.mat')); 

%energy = 

% TODO - would be good to return current AND aux var separately 
% also want to return cost -- how to return motors is harder question 
% NOTE: could be good to have a seperate return of motor torque 
% even though it could be calculated externally 


% Plot results - animate would be nice rtoo 

% show break up of torque 



% define one-hot in src 



