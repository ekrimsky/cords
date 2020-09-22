function input_data = resize_input_data(input_data, w_init, w_add)
%
%
%   TODO 
%
%
%
%

    n = length(input_data.omega);
    w = w_init + w_add;
    dim_y = w + n; % new problem size 


    % TODO -- confirming valid input sizes and all that 

    % For all the contraints that we already have, adjust their size accordingly 
    if isa(input_data.T, 'function_handle')
        T_init = input_data.T; 
        input_data.T = @(motor, gearbox) [T_init(motor, gearbox),...
                                                 sparse(w_add, w_add)];
    else 
        input_data.T = [T_init, sparse(w_add, w_add)];
    end 

    if isfield(input_data, 'G')
        G_tmp = input_data.G;
        G_pad = sparse(size(input_data.G, 1), w_add); 
        if isa(G_ineq_tmp, 'function_handle')
            input_data.G = @(motor, gearbox) [G_tmp(motor,gearbox), G_pad]; 
        else 
            input_data.G = [G_tmp, G_pad]; 
        end
    end 

    if isfield(input_data, 'G_ineq')
        G_ineq_tmp = input_data.G_ineq;
        G_ineq_pad = sparse(size(input_data.G_ineq, 1), w_add); 
        if isa(G_ineq_tmp, 'function_handle')
            input_data.G_ineq = @(motor, gearbox) [G_ineq_tmp(motor,gearbox),...
                                                                  G_ineq_pad]; 
        else 
            input_data.G_ineq = [G_ineq_tmp, G_ineq_pad]; 
        end 
    end  

    
    xlb_pad = -inf(w_add, 1);
    if isfield(input_data, 'x_lb')
        x_lb_tmp = input_data.x_lb;
        if isa(x_lb_tmp, 'function_handle')
            input_data.x_lb = @(motor, gearbox) [x_lb_tmp(motor,gearbox);...
                                                                  xb_pad]; 
        else 
            input_data.x_lb = [x_lb_tmp(:); xlb_pad]; 
        end 
    end  
    xub_pad = inf(w_add, 1);
    if isfield(input_data, 'x_ub')
        x_ub_tmp = input_data.x_ub;
        if isa(x_ub_tmp, 'function_handle')
            input_data.x_ub = @(motor, gearbox) [x_ub_tmp(motor,gearbox);...
                                                                  xb_pad]; 
        else 
            input_data.x_ub = [x_ub_tmp(:); xub_pad]; 
        end 
    end  




    if isfield(input_data, 'Q') % needs to have all anyway 
        Q = input_data.Q; 
        c = input_data.c;
        M = input_data.M; 
        r = input_data.r; 
        bet = input_data.beta;
    else 
        Q = {}; 
        c = {};
        M = {};
        r = {};
        bet = {};
    end 

    %% Fill in constraints 
    
    num_qc = numel(Q); % Number of quadratic constraints 

    % Appropriately size empty sparse matrices for padding M
    M_rpad = sparse(w_add, w_add);      % pad right 
    M_bpad = sparse(w_add, w);  % pad bottom

    % Q, c only act on currents, dont need to be resized 
    for jj = 1:num_qc   % loop through the quadratic constraints and resize appropriatly 
        if ~isempty(M{jj}) 
            if isa(M{jj}, 'function_handle')
                M{jj} = @(motor, gearbox) [M{jj}(motor, gearbox), M_pad;  M_bpad] % reindexing into sparse array 
            else 
                M{jj} = @(~, ~) [M{jj}, M_pad;  M_bpad] % reindexing into sparse array 
            end 
        end 

        if ~isempty(r{jj})
            if isa(r{jj}, 'function_handle')
                r{jj} = @(motor, gearbox) [r{jj}(motor, gearbox); zeros(w_add, 1)];
            else 
                r{jj} = [r{jj}; zeros(w_add, 1)];
            end
        end 
    end   % end resizing quadratic constraints 


    input_data.Q = Q;
    input_data.c = c;
    input_data.M = M;
    input_data.r = r; 
    input_data.beta = bet; 
end 