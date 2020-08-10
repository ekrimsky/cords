
%
%
%       Big block comment AND problem statement repeated word for word 
%
%
%
%
%
%
clearvars; clc;
% Another idea for the website 
% ... have pictures and jupiter notebook type 
% deal where we walk through each of the examples 

% Get Trajectory Data 


%....fill in  

% defining a desired trajectory under gravity 

% Fill in with rando stuff -- TODO - load in from file 

% TODO -- package up so can refenerence correctly 
load('example1_input_data.mat', 'time',  'theta', 'omega',...
                                'omega_dot', 'tau_des'); 
theta = theta(:); % col vec
omega = omega(:);
omega_dot = omega_dot(:);
time = time(:); 
tau_des = tau_des(:); 
n = length(time); 



% NOTE: could do 2 trajectories 
% Fill in all the problem data parts 

% Define aux variable x and the indices into it 
% x = [kp, tau_c, t]

w = 2 + n; % dimensionality of x vector
kp_idx = 1; 
tau_c_idx = 2; 
t_idxs = 3:(3 + n - 1); 

% Block comment at the begining of each of these would be good 


% Initialize Cell Arrays 
Q = {}; 
c = {};   % ...etc 
M = {};
r = {}; 
bet = {};


% Fill in Objective 
Q{1, 1} = []; 
c{1, 1} = []; 
M{1, 1} = [];

r0 = zeros(w, 1);
r0(t_idxs(1):t_idxs(end - 1)) = 0.5*ones(n - 1, 1).*diff(time);
r0(t_idxs(2):t_idxs(end)) = r0(t_idxs(2):t_idxs(end)) + 0.5*ones(n - 1, 1).*diff(time);
r{1, 1} = r0; 
bet{1, 1} = 0;


% Define efficiencies psi, 

Phi = 0.95; % driver board efficienc 
Phi_inv = 1/Phi; 
Psi = 0.8; % regen efficincy psi 
% Fill in general inequality constraints 
for j = 1:n

    % If positive power conumption 
    e_j = zeros(n, 1); 
    e_j(j) = 1; % one hot vector / unit basis vector 

    % NOTE  -- would it be more efficient to divide through by R so no Q dependednce on inputs? Still r and c dependence though 
    spdg_j =  sparse(diag(e_j)); 
    Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * spdg_j; 
    c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_t*gearbox.alpha*omega(j)*e_j;  % TODO -- k_e 
    M{end + 1, 1} = []; 
    r{end + 1, 1} = [0; 0; -e_j];  % accounting for the other vars 
    bet{end + 1, 1} = 0; 

    % If negative power consumption
    Q{end + 1, 1} = @(motor, ~) Psi*motor.R * spdg_j; 
    c{end + 1, 1} = @(motor, gearbox) Psi*motor.k_t*gearbox.alpha*omega(j)*e_j;     % TODO-- k_e 
    M{end + 1, 1} = []; 
    r{end + 1, 1} = [0; 0; -e_j];  % accounting for the other vars 
    bet{end + 1, 1} = 0; 
end 

% Fill in Equality Constraints 
% THERE ARE NOT ANY FOR THIS PROBLEM 
H = []; 
b = [];

% Fill in Torque Constraints 
% inertial compensation 
% accounting for par el
T = @(~, ~) [theta,  ones(n,1), zeros(n, w - 2)]; 
d = @(~, ~) tau_des;   % inertial compensation done in optimizer 

V_max = 48; % volts  
I_max = 100;  % Amps 


% Fill in Bound Constraints 
% account for voltage and current limits 
I_u = @(motor, gearbox) min(I_max,...
         (1/motor.R)*min(abs(V_max + motor.k_e*gearbox.alpha.*omega),...
               abs(V_max + motor.k_e*gearbox.alpha.*omega) ) ); 

% Assign all to the same struct 
problem_data.Q = Q;
problem_data.c = c; 
problem_data.M = M; 
problem_data.r = r; 
problem_data.beta = bet; 
problem_data.H = H;    % maybe A instead 
problem_data.b = b; 
problem_data.T = T;
problem_data.d = d;
problem_data.omega = omega; 
problem_data.omega_dot = omega_dot; 
problem_data.I_u = I_u; 



% Solve the problem 


prob = MotorSelection(); % instantiate new motor selection problem 

prob.update_problem(problem_data);   % update with the data 

%{
prob.filter_motors 
prob.filter_gears 

% consider whats the best way to do this 
prob.hints = % HOW BEST TO DO THIS? 
%} 

% Need to think about what return types should be 
% What inputs would make sense????? 

sol_struct = prob.optimize(10);

sol = sol_struct(1).sol; 


% Extract Results 
kp = sol.x(kp_idx)
tau_c = sol.x(tau_c_idx);
theta_0 = tau_c/kp; 


%energy = 

% TODO - would be good to return current AND aux var separately 
% also want to return cost -- how to return motors is harder question 
% NOTE: could be good to have a seperate return of motor torque 
% even though it could be calculated externally 


% Plot results 

% show break up of torque 



% define one-hot in src 



