function [sol_struct] = min_mass(prob, input_data, num_solutions)
%
%
%   TODO comment me 
%
%
%
% See also CORDS, MGDB, MIN_EFFECTIVE_INERTIA, MIN_POWER_CONSUMPTION, MIN_PEAK_TORQUE

    input_data.Q0 = [];
    input_data.r0 = [];
    input_data.M0 = [];
    input_data.c0 = [];
    input_data.beta0 = @(motor, gearbox) motor.mass + gearbox.mass; 

    prob.update_problem(input_data);
    sol_struct = prob.optimize(num_solutions);
end 