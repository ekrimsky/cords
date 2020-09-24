function [sol_struct] = min_effective_inertia(prob, input_data, num_solutions)
%
%
%   TODO -- comment me 
%
%
%
% See also CORDS, MGDB, MIN_MASS, MIN_POWER_CONSUMPTION, MIN_PEAK_TORQUE

    sol_struct = struct([]); % we might not get a solution below 
    input_data.Q0 = [];
    input_data.r0 = [];
    input_data.M0 = [];
    input_data.c0 = [];
    input_data.beta0 = @(motor, gearbox) (gearbox.ratio^2)*...
                                            (motor.inertia + gearbox.inertia);

    prob.update_problem(input_data);
    sol_struct = prob.optimize(num_solutions);
end 