function [result, model] = gurobi_solve(Q, obj, objcon, A_eq, b_eq, A_ineq, b_ineq,... 
                                                            quadcon, lb, ub)
%
%
%
%
%
%

    %% Gurobi Model 

    sense = [repmat('=', length(b_eq), 1); repmat('<', length(b_ineq), 1)]; 
    A_all = sparse([A_eq; A_ineq]); 
    b_all = [b_eq; b_ineq]; 


    dim_y = length(lb); 


    model.A = A_all;
    model.Q = Q_cost; 
    model.rhs = b_all; 
    model.sense = sense;   % equality/inequality sense 
    model.obj = c_cost;  % linear objective term
    model.objcon = beta0; % constant objective term 
    model.lb = lb; 
    model.ub = ub; 

    model.quadcon = quadcon;
    model.vtype = repmat('C', dim_y, 1); % all continuous variables 
    model.modelsense = 'min'; % minimize or maximize 


    % TODO -- make a separete functoin 
    % we call regardless of solver???? YAS 

    % TODO -- logging? 
    % TODO -- a way for users to change solver params 
    % like tolerances (--- 'Advanced Users only ----')
    params.DualReductions = 0; % To help debug inf_or_unbd
    params.BarHomogeneous = 1; % NOTE: maybe ONLY do this for binary aug problems if costs use speed 
    params.cutoff = cutoff; 
    params.outputflag = 0;      % TODO OPTIONAL FOR LOGGING TO FILE TOO -- IF IN DEBUG MODE 

    params.FeasibilityTol = obj.solve_settings.feastol;
    params.OptimalityTol = obj.solve_settings.abstol; 
    %params.BarConvTol = 1e-9; % defaulat 1e-8 
    %params.BarQCPConvTol = 1e-9; % default 1e-6


    if strcmpi(obj.settings.solver, 'gurobi') 
        if mixed_integer
            %params.presolve = 0;
            %params.FeasibilityTol = 1e-2; 
            params.IntFeasTol = 1e-4; % default 1e-5, max 1e-1, min 1e-9 
            model.vtype(matrices.binary) = 'B'; 
            params.outputflag = 1; % for debug 
        end 
    else 
        error('Havent written external B&B module yet');
    end 

    result = gurobi(model, params); 

    if mixed_integer
        %display(result)
        disp('lll--------------------------------- MIXED INT SOLVED ========')
    end 
    % TODO -- clean up this switch so not as much code reused 
    % GO THROUGH ALL STATUS CODES             
    % TODO https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html

    switch result.status 

        case 'OPTIMAL'
            y_sol = result.x; 
            combo_cost = result.objval; % NOTE: not subtracting off augmented cost 
        case 'SUBOPTIMAL'


            y_sol = result.x; 
            % subtract off cost augmentation 
            combo_cost = result.objval; 

            %if combo_cost <= cutoff

            % TODO -- should this be the condition? what about subotimal solves on fractional problems
            if (result.objval <= cutoff) && strcmp(obj.problem_type, 'standard')

                warning(['Solver returned suboptimal solution',...
                     ' may need to loosen tolerances']);

                display(result)
                display(combo_cost)
                %keyboard
            
            else 
                y_sol = nan(dim_y, 1);
                combo_cost = inf; % returned "suboptimal" but really cutoff 
            end 
        case 'NUMERIC'
            disp('yoooooooooooooooooooooooooooooooooooooooooooooooo')
            keyboard 
        case 'UNBOUNDED'

            error(['Unbounded problem encountered',...
                        ' Check that problem is well posed']);
        case 'INF_OR_UNBD'
            y_sol = nan(dim_y, 1);
            combo_cost = inf;
            %warning('Problem may be unbounded\n'); % WHY NOT PRINTIG? 
        otherwise  % CUTOFF / INF_OR_UNBD 
            y_sol = nan(dim_y, 1); 
            combo_cost = inf; 
    end 




end 