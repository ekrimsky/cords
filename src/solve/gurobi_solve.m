function [result, model] = gurobi_solve(Q, obj, objcon, A_eq, b_eq, A_ineq,...
                                    b_ineq, quadcon, lb, ub, cutoff, settings)
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

    model.Q = Q; 
    model.A = A_all;
    model.rhs = b_all; 
    model.sense = sense;   % equality/inequality sense 
    model.obj = obj;  % linear objective term
    model.objcon = objcon; % constant objective term 
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

    params.FeasibilityTol = settings.feastol;
    params.OptimalityTol = settings.abstol; 


    %params.BarConvTol = 1e-9; % defaulat 1e-8 
    %params.BarQCPConvTol = 1e-9; % default 1e-6

    result = gurobi(model, params); 

    % In rare cases the gurobi presolve proceduce can lead to numerical issues
    % leading to suboptimal termination. In this we will try to solve again
    % with presolve disabled
    if strcmp(result.status, 'SUBOPTIMAL')
        %arams.ScaleFlag = 2;
        %params.CrossoverBasis = 1; 
        params.cutoff = inf; 
        params.Presolve = 0;
        params.Aggregate = 0; 
        params.AggFill = 0; 
        params.BarConvTol = 1e-7; 
        %params.NumericFocus = 1;
        params.BarQCPConvTol = 1e-5;   % defualt 1e-6 
        result = gurobi(model, params); 
    end 



    % TODO https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html

    switch result.status 
        case 'OPTIMAL'
        case 'SUBOPTIMAL'
            % For linear fractional problems where we are really only concerend 
            % with feasiblity but still add the rho cost, 'subpoptimal'
            % solutions are fine 

            if (result.objval <= cutoff) 
                warning(['Gurobi solver returned suboptimal solution',...
                     ' may need to loosen tolerances']);
                keyboard

            end 
        case 'NUMERIC'
            warning(['Gurobi solver encountered numerical issues']);
        case 'UNBOUNDED'
            warning(['Gurobi solver encountered unbounded problem. ',...
                        'Check problem formulation']);
        otherwise  % CUTOFF / INF_OR_UNBD 
            result.objval = inf; % add this onmto the results 
    end  % end switch 




end 