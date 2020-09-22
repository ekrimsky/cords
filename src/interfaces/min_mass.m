function [sol_struct] = min_mass(prob, input_data, num_solutions)
%
%
%
%
%
%
% See also CORDS, MGDB, MIN_EFFECTIVE_INERTIA, MIN_POWER_CONSUMPTION, MIN_PEAK_TORQUE

    sol_struct = struct([]); % we might not get a solution below 
    input_data.Q0 = 0;
    input_data.r0 = 0;
    input_data.M0 = 0;
    input_data.c0 = 0;
    input_data.beta0 = @(motor, gearbox) motor.mass + gearbox.mass; 

    prob.update_problem(input_data);
    sol_struct = prob.optimize(num_solutions);
end 