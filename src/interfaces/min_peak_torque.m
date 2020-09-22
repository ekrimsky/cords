function [sol_struct] = min_peak_torque(prob, input_data, num_solutions, w_init)
                                                    
%
%   Note this is joint output torque 
%
%
%
% See also CORDS, MGDB, MIN_POWER_CONSUMPTION, MIN_MASS, MIN_EFFECTIVE_INERTIA 

    sol_struct = struct([]); % we might not get a solution below 
    omega = input_data.omega; 
    n = length(omega);
    w_add = 1; % adding 1 element corresponding to 
    w = w_init + w_add; 
    % point corresponding the power consumption at that time point 

    T_init = input_data.T;  % making a copy of this because need it below 
    tau_c = input_data.tau_c;
    if ~isa(T_init, 'function_handle')
        T_init = @(~, ~) T_init; 
    end 
    if ~isa(input_data.tau_c, 'function_handle')
        input_data.tau_c = @(~, ~) tau_c;
    end 
    input_data = resize_input_data(input_data, w_init, w_add);

    % Add 2*n constraints that tau_peak >= tau and tau_peak >= -tau
    % at every time step 
    %
    %   Augment G_ineq, h_ineq
    %
    %   Tx + tau_c = tau  
    %   
    %    New constraints:
    %       tau_peak >=  T*x + tau_c 
    %       tau_peak >= -T*x - tau_c 
    %   
    %
    if ~isfield(input_data, 'G_ineq')
        G_ineq_tmp = (~, ~) [];     % function handle to emtpy 
        h_ineq_tmp = (~, ~) [];     % function handle to empty
    else 
        if ~isa(input_data.G_ineq, 'function_handle')
            G_ineq_tmp = @(~, ~) input_data.G_ineq;
        else 
            G_ineq_tmp = input_data.G_ineq;
        end  
        if ~isa(input_data.h_ineq, 'function_handle')    
            h_ineq_tmp = @(~, ~) input_data.h_ineq
        else 
            h_ineq_tmp = input_data.h_ineq;
        end 
    end 

    G_ineq = @(motor, gearbox) [G_ineq_tmp(motor, gearbox);...
                                T_init(motor, gearbox), -ones(n, 1);
                                -T_init(motor, gearbox),-ones(n, 1)];
    h_ineq = @(motor, gearbox) [h_ineq_tmp(motor, gearbox);...
                                tau_c(motor, gearbox);...
                               -tau_c(motor, gearbox)];
        
    % Fill in Objective -- only has r0, everything else empty 
    Q0 = []; 
    c0 = []; 
    M0 = [];

    peak_torque_index = w; % its at the end of the x opt vector 
    r0 = zeros(w, 1);
    r0(peak_torque_index) = 1; 
    beta0 = 0; 

    input_data.Q0 = Q0;
    input_data.c0 = c0;
    input_data.M0 = M0; 
    input_data.r0 = r0;
    input_data.beta0 = 0; 

    input_data.G_ineq = G_ineq; 
    input_data.h_ineq = h_ineq; 

    prob.update_problem(input_data);
    sol_struct = prob.optimize(num_solutions);


    % Loop through outputs and add power information 

    for jj = 1:numel(sol_struct)
        sol = sol_struct(jj).sol;
        x = sol.x(1:w_init);
        pwr = sol.x(w_init + 1:end);
        sol.x = x; 
        sol.power = pwr;
        sol_struct(jj).sol = sol;
    end 

end 