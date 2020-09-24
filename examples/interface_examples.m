
%
%
%       TODO 
%
%
%
%
%   See also parallel_elastic
%
clearvars; clc; close all; 


addpath('examples/example_data');
load('parallel_elastic_input_data.mat', 'time',  'theta', 'omega',...
                                'omega_dot', 'tau_des'); 
theta = theta(:); % col vec
omega = omega(:);
omega_dot = omega_dot(:);
time = time(:); 
tau_des = tau_des(:); 
n = length(time); 

omega(omega == 0) = -1e-12;  % to help debug something 


eta_drive = 0.95;       % efficiency of drive electronics when driving motor
eta_regen = 0.80;       % efficiency of drive electronics in regeneration

% NOTE: could do 2 trajectories 
% Fill in all the problem data parts 

% Define aux variable x and the indices into it 
% x = [kp, tau_offset]
w = 2; % dimensionality of x vector
kp_idx = 1; 
tau_offset_idx = 2; 


% Block comment at the begining of each of these would be good 


x_lb = [-20; -20];   % providing reasonable bounds on inputs can help :)
x_ub = [20; 20];


% Fill in Torque Constraints 
% inertial compensation 
% accounting for par el
T = @(~, ~) [-theta,  -ones(n,1)]; 
tau_c = @(~, ~) tau_des;  

V_max = 48; % volts  
I_max = 100;  % Amps 

% Fill in Bound Constraints 
% account for voltage and current limits 


% Assign all to the same struct 

%% Fill in trajectory and desired torque data 
problem_data.T = T;
problem_data.tau_c = tau_c;  
problem_data.omega = omega;
problem_data.omega_dot = omega_dot; 

problem_data.x_lb = x_lb;
problem_data.x_ub = x_ub;

problem_data.I_max = I_max; 
problem_data.V_max = V_max; 

% For a list of valid filters see the MGDB documentation
selection_filters.total_mass = 4; % only accept combinations lighter than 4 kg 

% Solve the problem 
settings.solver = 'gurobi';     % set the solver to gurobi (the default)
% instantiate new motor selection problem and build a new database 
prob = cords('settings', settings, 'reuse_db', false); 
prob.update_filters(selection_filters); 
num_solutions  = 100; 




%}
%%
%
%        Solve Prolem of Minimum Mass
%
%


mass_sols = min_mass(prob, problem_data, num_solutions);

mass_fig = plot_result(mass_sols(1), kp_idx, tau_offset_idx,...
                                                 time, theta, tau_des);
%%
%
%        Solve Prolem of Minimum Effective Inertia 
%
%
inertia_sols = min_effective_inertia(prob, problem_data, num_solutions);


%%
%
%        Solve Prolem of Minimum Power Consumption
%
%

power_sols = min_power_consumption(prob, problem_data, num_solutions, w,...
                                              time, eta_drive, eta_regen);



%%
%
%     Solve Prolem of Minimum Power Consumption using Resistive Braking
%
%   If using a brake resistor instead of regenerative breaking, we set the 
%   regen efficiency to zero 
eta_regen_br = 0;  % do not regenerate  
power_sols_br = min_power_consumption(prob, problem_data, num_solutions, w,...
                                    time, eta_drive, eta_regen_br);


save('examples/interface_examples_result.mat')




function fig = plot_result(solution, kp_idx, tau_offset_idx,...
                                                 time, theta, tau_des)

    sol = solution.sol;
    kp = sol.x(kp_idx);
    tau_offset = sol.x(tau_offset_idx); 
    tau_spring = tau_offset + kp * theta; 



    fig = figure; hold on  
    plot(time, tau_des, 'k--', 'linewidth', 2, 'DisplayName', 'Desired Torque');
    plot(time, sol.tau, 'r', 'linewidth', 1.6, 'DisplayName', 'Output Torque');
    plot(time, tau_spring, 'b', 'linewidth', 1.6, 'DisplayName', 'Spring Torque');
    plot(time, sol.tau(:) + tau_spring(:), 'm', 'linewidth', 1.4,...
                                                'DisplayName', 'Total_torque');

    ylabel('Torque (Nm)')
    xlabel('Time (s)')
    leg = legend;
    hold off 

    % ADD SOME PRINT TEXT 

end 