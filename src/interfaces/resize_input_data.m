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


    if isfield(input_data, 'P') % needs to have all anyway 
        P = input_data.P; 
        C = input_data.C;
        F = input_data.F; 
        bet = input_data.beta;
    else 
        P = [];
        C = [];
        F = []; 
        bet = [];
    end 

    %% Fill in constraints 
    

    if isfield(input_data, 'quadcon')
        quadcon = @(motor, gearbox) resize_quadcon(motor, gearbox,...
                                 input_data.quadcon, w_add);
    else 
        input_data.quadcon = struct([]); % for consistency really 
    end 

    input_data.P = P;
    input_data.C = C;
    input_data.F = F;
    input_data.beta = bet; 

end 



function new_quadcon = resize_quadcon(motor, gearbox, init_quadcon, w_add)

    q_pad = zeros(w_add, 1); 

    if isa(init_quadcon, 'function_handle')
        init_quadcon = init_quadcon(motor, gearbox);
    end 

    num_qc = numel(init_quadcon); 

    for i = 1:num_qc
        if isfield(init_quadcon(i), 'Qc')
            [Qrow, Qcol, Qval] = find(init_quadcon(i).Qc); % assumed sparse 
        else 
            Qrow = init_quadcon(i).Qrow;
            Qcol = init_quadcon(i).Qcol;
            Qval = init_quadcon(i).Qval;
        end 
        new_quadcon(i).Qrow = Qrow;
        new_quadcon(i).Qcol = Qcol;
        new_quadcon(i).Qval = Qval; 
        new_quadcon(i).q = [init_quadcon(i).q(:); q_pad]
        new_quadcon(i).rhs = init_quadcon(i).rhs; 
    end 
end 