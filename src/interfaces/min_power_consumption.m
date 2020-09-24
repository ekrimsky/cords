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

    Q = input_data.Q;
    c = input_data.c;
    M = input_data.M;
    r = input_data.r; 
    bet = input_data.beta; 

    % Fill in Objective -- only has r0, everything else empty 
    Q0 = []; 
    c0 = []; 
    M0 = [];

    r0 = zeros(w, 1);
    pwr_idxs = w_init + (1:n); 
    r0(pwr_idxs(1):pwr_idxs(end - 1)) = 0.5*ones(w_add - 1, 1).*diff(time);
    r0(pwr_idxs(2):pwr_idxs(end)) = r0(pwr_idxs(2):pwr_idxs(end)) + ...
                                                0.5*ones(w_add - 1, 1).*diff(time);
    beta0 = 0; 
    input_data.Q0 = Q0;
    input_data.c0 = c0;
    input_data.M0 = M0; 
    input_data.r0 = r0;
    input_data.beta0 = 0; 

    % Define efficiencies psi, 
    Phi_inv = 1/eta_drive; 
    % Fill in general inequality constraints 
    for j = 1:n
        % If positive power conumption 
        e_j = zeros(n, 1); 
        e_j(j) = 1; % one hot vector / unit basis vector 

        r_tmp = [0; 0; -e_j];

        Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * e_j; % vector type input 
        c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_t ...                      % TODO -- dounle check directions with math 
                                 *gearbox.direction*gearbox.ratio*omega(j)*e_j; 
        M{end + 1, 1} = []; 
        r{end + 1, 1} = r_tmp;  % accounting for the other vars 
        bet{end + 1, 1} = 0; 

        % If negative power consumption
        Q{end + 1, 1} = @(motor, ~) eta_regen*motor.R * e_j; % sparse matrix type input 
        c{end + 1, 1} = @(motor, gearbox) eta_regen*motor.k_t ...
                                  *gearbox.direction*gearbox.ratio*omega(j)*e_j;     
        M{end + 1, 1} = []; 
        r{end + 1, 1} = r_tmp;  % accounting for the other vars 
        bet{end + 1, 1} = 0; 
    end 


    %% Solve the problem with the write number of outputs? 
    input_data.Q = Q; 
    input_data.c = c;
    input_data.M = M; 
    input_data.r = r; 
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