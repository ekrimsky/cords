function [sol_struct] = min_power_consumption(prob, input_data, num_solutions, w_init,...
                                                     time, eta_drive, eta_regen)
%
%
%
%
%
%
% See also CORDS, MGDB, MIN_MASS, MIN_EFFECTIVE_INERTIA, MIN_PEAK_TORQUE


    omega = input_data.omega; 
    w_add = length(omega); % adding 1 element to opt vector for every time 
    n = length(omega); % -- same as w_add in this context 
    w = w_init + w_add; 
    % point corresponding the power consumption at that time point 
    
    input_data = resize_input_data(input_data, w_init, w_add);

    P = input_data.P;
    C = input_data.C;
    F = input_data.F;
    bet = input_data.beta; 
    quadcon = input_data.quadcon;


    % Fill in Objective -- only has r0, everything else empty 
    p0 = []; 
    c0 = []; 
    M0 = [];

    f0 = zeros(w, 1);
    pwr_idxs = w_init + (1:n); 
    f0(pwr_idxs(1):pwr_idxs(end - 1)) = 0.5*ones(w_add - 1, 1).*diff(time);
    f0(pwr_idxs(2):pwr_idxs(end)) = f0(pwr_idxs(2):pwr_idxs(end)) + ...
                                                0.5*ones(w_add - 1, 1).*diff(time);
    beta0 = 0; 
    input_data.p0 = p0;
    input_data.c0 = c0;
    input_data.M0 = M0; 
    input_data.f0 = f0;
    input_data.beta0 = 0; 

    % Comment here should then explain what each row of this inequality looks like 
    tmp_mat = sparse(eye(n)); 
    P_add = @(motor, gearbox) motor.R * [(1/eta_drive) * tmp_mat; eta_regen * tmp_mat]; 
    omega_diag = sparse(diag(omega));
    C_add = @(motor, gearbox) motor.k_t*gearbox.direction*gearbox.ratio * [(1/eta_drive)*omega_diag; eta_regen*omega_diag]; 
    F_tmp = sparse(1:2*n, [pwr_idxs, pwr_idxs], -ones(1, 2*n), 2*n, w);
    F_add = F_tmp; 
    bet_add = zeros(2*n, 1); % no constant offsets 

    % If there is an initial P, C, F, beta -- add to it 
    % if there isnt just use the new ones 

    if isa(P, 'function_handle') || ~isempty(P)
        P = @(motor, gearbox) [P(motor, gearbox); P_add(motor, gearbox)];
    else 
        P = P_add; 
    end 

    if isa(C, 'function_handle') || ~isempty(C)
        C = @(motor, gearbox) [C(motor, gearbox); C_add(motor, gearbox)];
    else 
        C = C_add;
    end  

    if isa(F, 'function_handle') || ~isempty(F)
        F = @(motor, gearbox) [F(motor, gearbox); F_add(motor, gearbox)];
    else
        F = F_add;
    end 
        

    if isa(bet, 'function_handle') || ~isempty(bet)
        bet = @(motor, gearbox) [bet(motor, gearbox); bet_add(motor, gearbox)];
    else 
        bet = bet_add;
    end 

    %% Solve the problem with the write number of outputs? 
    
    
    input_data.P = P; 
    input_data.C = C;
    input_data.F = F; 
    input_data.beta = bet;  % scalar, does not need to be resized 

    
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