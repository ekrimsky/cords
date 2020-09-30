function [sol_struct] = min_effective_inertia(prob, input_data, num_solutions)
%
%
%   TODO -- comment me 
%
%
%
% See also CORDS, MGDB, MIN_MASS, MIN_POWER_CONSUMPTION, MIN_PEAK_TORQUE

    input_data.p0 = [];
    input_data.c0 = [];
    input_data.f0 = [];
    input_data.M0 = [];
    input_data.beta0 = @(motor, gearbox) (gearbox.ratio^2)*...
                                            (motor.inertia + gearbox.inertia);

    prob.update_problem(input_data);
    sol_struct = prob.optimize(num_solutions);
end 