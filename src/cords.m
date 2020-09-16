classdef cords < matlab.mixin.Copyable 
% CORDS (Convex Optimal Robot Drive Selection)    
%      Summary goes here  
%
%    This needs to work well with 'help'
%
%
%   CORDS Methods:
%       update_problem - 
%       optimize - 
%
%   CORDS Properties: 
%       settings - 
%       rankings - 3 x N array. Cols - [expected_cost, motor_idx, gear_idx]
%     
%
%
%
%   Blah. blah. blah 
%
%   A reference to the github
%
%   A reference to the eventual paper 
%
%   see also -- how do you link these????
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 7/27/20
%       Stanford University, Biomechatronics Lab 
%
properties (GetAccess = public, SetAccess = private)
    settings        %  for solver, verbose, print frequency
    solve_settings  % numerical tolerances and the like  
    problem_type    % char array, 'standard' (SOCP) or 'fractional'
    problem_data    % data for the problem we want to solve 
    test_motor      % for validating input properties 
    test_gearbox     % for validating input properitse 
    mg_database     % motor gearbox database object 
    % TODO Friction and other model settings? 
end 


methods (Access = public)

        % https://www.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html


    function obj = cords(varargin) % we will parse the inputs directly 
    %**********************************************************************
    % Constructor for the class. 
    %
 	% 	Optional Inputs (in order)
    %  
    %
    % 
    %
    %
    % Returns:
    %    MotorSelection object
    %********************************************************************    
        ip = inputParser; 
        addParameter(ip, 'settings', []);
        addParameter(ip, 'solve_settings', []);
        parse(ip, varargin{:}); 

        obj.settings = validate_settings(ip.Results.settings); 
        obj.solve_settings = validate_solve_settings(ip.Results.solve_settings); 


        [test_motor, test_gearbox] = test_motor_gearbox();
        obj.test_motor = test_motor;
        obj.test_gearbox = test_gearbox; 

        % NOTE: might want to look for saved file that has a database object 
        % stored away and ONLY if we cant find one, instantiate a new one
        % the reason to do this is that reading in all the csvs can be slow 
        obj.mg_database = mgdb(); 
    end %end constructor 

    


    %**********************************************************************
    % Method: update_problem
    % 
    % Inputs: 
    %       problem_data, a struct with the following
    %       
    %       Required Fields: 
    %
    %       Optional Fields:
    %
    %   Added:   
    %**********************************************************************
    function update_problem(obj, problem_data)
    %
    %   
    %
    %
    %
    %
    %

        [problem_data, problem_type] = obj.validate_problem_data(problem_data);
        obj.problem_data = problem_data; 
        obj.problem_type = problem_type; 
    end 

        



    %**********************************************************************
    % Method: optimize
    % 
    % Inputs: 
    %       % DOcumentation for methods???? 
    %      
    % Returns:
    %   
    %
    %   Added:   
    %**********************************************************************
    function [sol_structs] = optimize(obj, varargin)
    %   optimize Do a thing dude 
    %
    %   Optional Input:  
    %       num_return - default = 1. The number of motor/gearbox combinations
    %               and solutions to return. Returned in ascending order.
    %               use inf to return all solutions however this will take
    %               much longer 
    %
    %   Output: sol_structs - struct array of solution with fields TODO 
    %
    %
    %   The problem type is assumed from the setup (standard or fractional)
    %

        if isempty(varargin) || isempty(varargin{1})
            num_return = 1;
        else 
            num_return = varargin{1};
        end 

        %
        %       Get Valud Motor/Gearbox Combinations 
        %
        %
        %   TODO filter me 
        %   TODO filter me 
        %   TODO filter me 
        %
        obj.mg_database.update_filter('omega_max', max(abs(obj.problem_data.omega))); 

        [motor_keys, gearbox_keys] = obj.mg_database.get_combinations();
        % Convert from database map keys to structs 
        for i = 1:length(motor_keys)  % slightly faster than cell fun
            motors(i) = obj.mg_database.motors(motor_keys{i});
            gearboxes(i) = obj.mg_database.gearboxes(gearbox_keys{i});
        end 
    
        %
        %       
        %          Call the Optimizer for the Given Problem Type  
        %
        %
        if strcmp(obj.problem_type, 'standard')  
            [sol_structs, cost_list, mincost, exitflag] = ...
                        obj.optimize_standard(motors, gearboxes, num_return);
        elseif strcmp(obj.problem_type, 'fractional')
            [sol_structs, cost_list, mincost, exitflag] = ...
                     obj.optimize_fractional(motors, gearboxes, num_return); 
        else 
            error('Invalid problem type. Must be "standard" or "fractional" '); 
        end 

        if isinf(mincost)       
            sol_structs = struct(); % empty 
            warning('No feasible solutions found');
        end 

        % Update the rankings using the cost list to speed up the next solve 

        obj.mg_database.update_rankings(motor_keys, gearbox_keys, cost_list);

    end % end optimize 


end % end public methods 

%
%
%
%
%
%
%
%
%
%


    % NOTE: some plotting methods could be good 
    % Selection Filtering?
    % Private methods? 

    % Function to CLEAR FILTERS on motor/gb -- go back to original 
    % ...this would probably clear any motor/gb that was manually added
    % no great way around this 

methods (Access = private)

    function [problem_data, problem_type] = validate_problem_data(obj, problem_data)
    %
    %
    %
    %
    %

        % TODO add OPTIIONAL x_ub/x_lb as more efficient 

        assert(isfield(problem_data, 'omega'), 'Missing omega');
        assert(isfield(problem_data, 'Q'), 'Missing Q');
        assert(isfield(problem_data, 'c'), 'Missing c');
        assert(isfield(problem_data, 'M'), 'Missing M');
        assert(isfield(problem_data, 'r'), 'Missing r');
        assert(isfield(problem_data, 'beta'), 'Missing beta');
        assert(isfield(problem_data, 'H'), 'Missing H');
        assert(isfield(problem_data, 'b'), 'Missing b');
        assert(isfield(problem_data, 'T'), 'Missing T');
        assert(isfield(problem_data, 'd'), 'Missing d');
        assert(isfield(problem_data, 'I_max'), 'Missing I_max');
        assert(isfield(problem_data, 'V_max'), 'Missing V_max');

        assert(isnumeric(problem_data.I_max) && problem_data.I_max > 0, 'I_max must be positive');
        assert(isnumeric(problem_data.V_max) && problem_data.V_max > 0, 'V_max must be positive');


        % Get dummy motor and dummy gearbox for input validation 
        test_motor = obj.test_motor; 
        test_gearbox = obj.test_gearbox; 

        omega = problem_data.omega(:); 
        Q = problem_data.Q; 
        c = problem_data.c;
        M = problem_data.M; 
        r = problem_data.r; 
        bet = problem_data.beta; 
        H = problem_data.H; 
        b = problem_data.b; 
        T = problem_data.T; 
        d = problem_data.d; 
        I_max = problem_data.I_max;
        V_max = problem_data.V_max; 

        % Optional Inputs H_ineq, b_ineq
        % where H_ineq x + b_ineq = 0
        H_ineq = []; 
        b_ineq = [];
        if isfield(problem_data, 'H_ineq') || isfield(problem_data, 'b_ineq') 
            assert(isfield(problem_data, 'H_ineq'),...
                     'Need to supply H_ineq AND b_ineq');
            assert(isfield(problem_data, 'b_ineq'),...
                     'Need to supply H_ineq AND b_ineq');
            H_ineq = problem_data.H_ineq;
            b_ineq = problem_data.b_ineq; 
        end 

        n = length(omega);  
        m = numel(Q) - 1; 


        % NOTE -- if its faster to have NO depedndeance on inpuits
        % might check sensitity and then use null
        % inputs to speed up alot 

        % Check that others are the right size too
        if ~isa(H, 'function_handle');  H = @(motor, gearbox) H;      end 
        if ~isa(H_ineq, 'function_handle')
                        H_ineq = @(motor, gearbox) H_ineq;            end 
        if ~isa(b, 'function_handle');  b = @(motor, gearbox) b(:);   end 
        if ~isa(b_ineq, 'function_handle')
                        b_ineq = @(motor, gearbox) b_ineq(:);         end 
        if ~isa(T, 'function_handle');  T = @(motor, gearbox) T;      end 
        if ~isa(d, 'function_handle');  d = @(motor, gearbox) d(:);   end 



        % Same for the rest 
        for j = 1:m + 1
            % Check if input is function handle, if not make it so 
            % NOTE: may want to change these too 
            if ~isa(Q{j}, 'function_handle'); Q{j} = @(~, ~) Q{j}; end 
            if ~isa(M{j}, 'function_handle'); M{j} = @(~, ~) M{j}; end 
        end 


        % Validate Remainin Inputs      
        T_test = T(test_motor, test_gearbox);
        w = size(T_test, 2); 
        d_test = d(test_motor, test_gearbox); 
        assert(size(T_test, 1) == n, 'Incorrect number of rows in T');
        assert(numel(d_test) == n || isempty(d_test), 'Incorrect d length');

        H_test = H(test_motor, test_gearbox); 
        b_test = b(test_motor, test_gearbox); 
        assert(size(H_test, 2) == w || isempty(H_test),...
                                         'Incorrect number of columns in H');
        assert(size(H_test, 1) == length(b_test), 'Size H and b incompatible')
        p = length(b_test); 

        H_ineq_test = H_ineq(test_motor, test_gearbox); 
        b_ineq_test = b_ineq(test_motor, test_gearbox); 
        assert(size(H_ineq_test, 2) == w || isempty(H_ineq_test),...
                            'Incorrect number of columns in H_ineq');
        assert(size(H_ineq_test, 1) == length(b_ineq_test),...
                         'Size H_ineq and b_ineq incompatible')

  

        %% Check for optional inputs -- Acceleration 

        if isfield(problem_data, 'omega_dot')
            omega_dot = problem_data.omega_dot(:); 
            assert(length(omega_dot) == n,...
                    'omega and omega_dot must be same size'); 
        else 
            omega_dot = zeros(n, 1); % treat as zero
        end 


        % when f_j is encoding linear inequality -- its more  
        % efficient to explicitly encode these as linear constrainrts
        % intead of SOC constraints with empty matrices 
        % We will convert Q to vectors containing the diagonal 
        Q_empty = zeros(n, 1);  % the diagonal 
        M_empty = sparse(w, w); 
        c_empty = zeros(n, 1);    
        r_empty = zeros(w, 1);   
        lin_ineq = zeros(m, 1);   % Indicator (1 = linear)

        % Letting c,r,beta be fixed instead of function handles can 
        % greatly speed up code 
        for j = 1:m + 1
            Qj = Q{j}(test_motor, test_gearbox); % to test validity 
            Mj = M{j}(test_motor, test_gearbox); % to test validity 

            if j == 1
                Q0_test = Qj;  % used in veryfying fractional inputs 
                M0_test = Mj;
            end 

            if isa(c{j}, 'function_handle')
                cj = c{j}(test_motor, test_gearbox);
            else
                if isempty(c{j})
                    c{j} = c_empty;
                end  
                cj = c{j};
            end 

            if isa(r{j}, 'function_handle')
                rj = r{j}(test_motor, test_gearbox);
            else
                if isempty(r{j})
                    r{j} = r_empty;
                end 
                rj = r{j};
            end 

            if isempty(Qj); Q{j} = @(~, ~) Q_empty; Qj = Q_empty; end  % fill in with vector 
            if isempty(Mj); M{j} = @(~, ~) M_empty; Mj = M_empty; end  % fill in with empty sparse 


            assert(issparse(Mj) || isempty(Mj),...
                             'update_problem: M matrices must be sparse');
            assert(issymmetric(Mj), 'update_problem: M matrices must be symmetric');
            assert(size(Mj, 1) == w || isempty(Mj), 'size of T or M%d incorrect', j-1);

            assert(size(rj, 1) == w, ['update_problem: r_%d incorrect length, ',...
                            'expected %d, got %d'], j, w, size(rj, 1) );
            assert(size(cj, 1) == n, ['update_problem: c_%d incorrect length, ',...
                            'expected %d, got %d'], j, n, size(cj, 1) );
            assert(size(rj, 2) == 1, 'update_problem: r_%d must be col vector', j-1);
            assert(size(cj, 2) == 1, 'update_problem: c_%d must be col vector', j-1);

            assert(min(cj.*omega) >= 0, ['update_problem: c_%d'' must ',...
                                            'satisfy c .* omega >= 0'], j-1); 


            % TODO -- size check on Q
            % NOTE: is there a cleaner code way to verify size 
            assert(isempty(Qj) || ((size(Qj, 1) == n) && (size(Qj, 2) == 1)) ||...
                         (issparse(Qj) && isdiag(Qj)) ,...
                     ['update_problem: Q matrices must be sparse diagonal '...
                     'matrix or non-sparse N x 1 vector specify diagonal elements ']);

            % Convert All Qs to vectors of diagonal - more efficient 
            if size(Qj, 2) > 1 % if not col vector 
                % We should either force the user to specify as a dense 
                % col vector specifying diagonal or at least issue a warning
                warning(['Specifying Q as a matrix instead of a vector ',...
                                    'containing the diagonal can be slow']); 
                Qj = full(diag(Q{j}(test_motor, test_gearbox)));  % This is very slow 
                % Now always returns a vcetor 
                % NOTE: may note need "FULL"
                Q{j} = @(motor, gearbox) full(diag(Q{j}(motor, gearbox)));
            end 

            % TODO -- clean up or at least group some of the Q business 
            assert(min(Qj) >= 0,...
                   'Diagonal entries of Q matrices must be non-negative');

            if (nnz(Mj) == 0) && (j > 1)  % because using linear on Q
                lin_ineq(j - 1) = 1; 
            end

            % TODO -- replace with simpler verion that skips eigen decomp 
            if nnz(Mj) > 0
                [row, col, val] = find(Mj)
                %unique_col = unique(col); 
                unique_col = find(any(Mj));  % faster  

                M_small = Mj(:, unique_col)
                M_small = full(M_small(unique_col, :)); 
                [V, D] = eig(M_small); 

                num_neg_eig = sum([D < 0]); % May need to play with tolerancing 
                if num_neg_eig == 1  % STANDARD SOC or ROTATED SOC 

                    assert(nnz(rj) == w, 'Linear term must be empty for SOC constraints'); 

                    % TODO -- what about constant term???? 
                    % ...... if solving with gurobi dont need to do anything 
                    % NOTE: should check that its still valid
                    % one negative eig does not neccesarilt 
                    % Diagonal of D in increasing order 
                    % eigenvector corresponding to negative eigenvalue 
                    v = D(:, 1); 
                    v([abs(v) < 1e-12]) = 0; % tolerancing  
                    
                    if nnz(v) > 2 % Invalid eigen vector 
                        error('update_problem:invalidInput',...
                        'Error. The eigenvector associate with the',...
                        'smallest eigenvalue of M_%d',...
                        'must have at most 2  non-zeros',...
                        'but has %d', j, nnz(v)); 
                    end 
                elseif num_neg_eig > 1  
                    error('update_problem:invalidInput',...
                        'Error. M_%d must have at most 1 negative',...
                        'eigenvalue, has %d', j, num_neg_eig); 
                end 
            end 
        end 

        %
        %
        %         Validating inputs for linear fractional problems 
        %
        %
        if (isfield(problem_data, 'cost_ub') && ~isempty(problem_data.cost_ub)) ||...
           (isfield(problem_data, 'cost_lb') && ~isempty(problem_data.cost_lb)) ||...
           (isfield(problem_data, 'r_num') && ~isempty(problem_data.r_num)) ||...
           (isfield(problem_data, 'r_den') && ~isempty(problem_data.r_den))  || ...
           (isfield(problem_data, 'beta_num') && ~isempty(problem_data.beta_num)) ||...
           (isfield(problem_data, 'beta_den') && ~isempty(problem_data.beta_den)) 


           % If it has any one of those field and no empty 
           % need to ensure it has ALL of them 
            assert(isfield(problem_data, 'cost_ub') &&...
                 ~isempty(problem_data.cost_ub),...
                 'Cost upper bound required for fractional problems'); 
            assert(isfield(problem_data, 'cost_lb') &&...
                 ~isempty(problem_data.cost_lb),...
                 'Cost lower bound required for fractional problems'); 
            assert(isfield(problem_data, 'r_num') &&...
                 ~isempty(problem_data.r_den),...
                 'Numerator vector required for fractional problems'); 
            assert(isfield(problem_data, 'r_den') &&...
                 ~isempty(problem_data.r_num),...
                 'Denominator vector required for fractional problems'); 
            assert(isfield(problem_data, 'beta_num') &&...
                 ~isempty(problem_data.beta_num),...
                 'Numerator offset required for fractional problems'); 
            assert(isfield(problem_data, 'beta_den') &&...
                 ~isempty(problem_data.beta_den),...
                 'Denominator offset required for fractional problems'); 

            assert(isnumeric(problem_data.cost_ub), 'Costs must be numeric');
            assert(isnumeric(problem_data.cost_lb), 'Costs must be numeric');
            assert(problem_data.cost_ub - problem_data.cost_lb > 0,...
                    'Cost upper bound must be greater than lower bound'); 

            % make col vec regardless of input 
            problem_data.r_num = problem_data.r_num(:); 
            problem_data.r_den = problem_data.r_den(:); 

            % Size + Range Check on inputs 
            assert(isscalar(problem_data.cost_ub), 'Cost upper bound must be scalar');
            assert(~isinf(problem_data.cost_ub), 'Cost upper bound must be finite');
            assert(isscalar(problem_data.cost_lb), 'Cost lower bound must be scalar');
            assert(~isinf(problem_data.cost_lb), 'Cost lower bound must be finite');
            assert(isscalar(problem_data.beta_num), 'beta_num must be scalar');
            assert(~isinf(problem_data.beta_num), 'beta_num must be finite');
            assert(isscalar(problem_data.beta_den), 'beta_den must be scalar');
            assert(~isinf(problem_data.beta_den), 'beta_den must be finite');
            assert(length(problem_data.r_num) == w, 'r_num must be length %d', w);
            assert(length(problem_data.r_den) == w, 'r_den must be length %d', w);

            % If linear fractional - cost terms must all be zero/empty 
            error_txt = 'Cost inputs must be zero or empty for fractional problem';
            assert(isempty(Q0_test) || nnz(Q0_test) == 0, error_txt);
            assert(isempty(c{1}) || nnz(c{1}) == 0, error_txt);
            assert(isempty(M0_test) || nnz(M0_test) == 0, error_txt);
            assert(isempty(r{1}) || nnz(r{1}) == 0, error_txt);
            assert(isempty(bet{1}) ||  bet{1} == 0, error_txt);
            problem_type = 'fractional'; 
        else 
            problem_type = 'standard'; 
        end 


        %
        %
        % Once all validated 
        %
        %
        problem_data.Q = Q;   % etc 
        problem_data.c = c; 
        problem_data.M = M; 
        problem_data.r = r; 
        problem_data.beta = bet; 
        problem_data.H = H; 
        problem_data.H_ineq = H_ineq; 
        problem_data.b = b; 
        problem_data.b_ineq = b_ineq; 
        problem_data.T = T; 
        problem_data.d = d; 
        problem_data.omega = omega;
        problem_data.omega_dot = omega_dot; 
        problem_data.zero_vel_idxs = find(omega == 0); 
        problem_data.lin_ineq = lin_ineq; 
        problem_data.I_max = I_max; 
        problem_data.V_max = V_max;

        % update relevant dimensions 
        problem_data.n = n;
        problem_data.w = w;
        problem_data.m = m;
        problem_data.p = p; 

        if strcmp(problem_type, 'fractional')
            problem_data.cost_ub = problem_data.cost_ub;
            problem_data.cost_lb = problem_data.cost_lb;
            problem_data.r_num = problem_data.r_num;
            problem_data.r_den = problem_data.r_den; 
            problem_data.beta_num = problem_data.beta_num;
            problem_data.beta_den = problem_data.beta_den; 
        end 

  
    end % end validate problem data 

    function [sol_structs, cost_list, mincost, exitflag] = ...
                           optimize_standard(obj, motors, gearboxes, num_return)

        start_tic = tic; 

        num_combinations = length(motors); 

        cost_list = nan(num_combinations, 1); 
        cutoff = inf; % no valid solutions yet 
        mincost = inf; % NOTE: not same as cutoff if want to hold on to multiple sols 
        best_combo_idx = 0; % no valid solutions yet 

        % runs the outer loop of the optimization 
        num_return = min(num_combinations, num_return); % cant return more than num options 


        table_line = [repmat('-', 1, 80), '\n']; 
        obj.vprintf(1, table_line);


        header_r1 = [" Combo x  |",  " Repeated |", " Mixed Int |",...
                          "     |",  "   |", "     "];
        combo_frac_txt = sprintf(' of %d |', num_combinations);
        header_r2 = [combo_frac_txt, "  Solves |", " Solves |",...
                          "  Cutoff  |", "Best Cost |", "   Time  "];

        header_txt1 = sprintf('%+14s%+12s%+12s%+13s%+13s%+11s\n', header_r1);
        header_txt2 = sprintf('%+14s%+12s%+12s%+13s%+13s%+11s\n', header_r2);
        obj.vprintf(1, header_txt1);
        obj.vprintf(1, header_txt2);
        obj.vprintf(1, table_line);

        
        % cutoff corresponds to max value of solutions being kept 
        % use a priority queue 
        pq = PQ(true); % 

        num_rep_solves = 0;     % repeated SOCP solves 
        num_mi_solves = 0;      % MISOCP solves 
        disp_txt = [];      % for print outs

        for j = 1:num_combinations 

            motor = motors(j);
            gearbox = gearboxes(j); 

            [combo_cost, tmp_sol, comp_time, exitflag] = obj.combo_solve(motor,...
                                         gearbox, cutoff); 
            if exitflag > 0; num_rep_solves = num_rep_solves + 1; end 
            if exitflag > 1; num_mi_solves  = num_mi_solves + 1;  end 

            % may keep a seperate pseud-cost list for updating rankings 
            if ~isinf(combo_cost)
                cost_list(j) = combo_cost;

                if combo_cost < cutoff

                    % define struct to insert into the pq
                    sol_struct.cost = combo_cost; 
                    sol_struct.sol = tmp_sol;
                    sol_struct.motor = motor; 
                    sol_struct.gearbox = gearbox; 

                    pq.insert(combo_cost, sol_struct); 
                    if pq.size() > num_return  % if inserting put us over limit 
                        pq.pop(); % remove max element from pq 
                        % then update the cutoff      
                        [cutoff, ~] = pq.peek(); % update the cutoff 
                    end 

                    % For the actual minimum cost 
                    if  combo_cost < mincost
                        mincost = combo_cost; 
                    end 
                end 
            else
                % Think on this more 
                % could be better to keep track of some mone 
                % Longer computation time typically more iterations
                % to reach dual cutoff. This acts as a pseudocost for 
                % combinations where we havent computed the full
                % solution but havent proven infeasibility either
                cost_list(j) = cutoff + (1/comp_time); 
            end 

            %
            %                   Print Outs to screen 
            %
            if  (j == 2) || (j == num_combinations) || ...
                ((mod(j - 1, obj.settings.print_freq) == 0) && j > 1)
                % Print a new line 
                obj.vprintf(1, '\n'); 
            else 
                % update the current line (by first removing)
                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
            end 
            disp_txt = sprintf('%13d%12d%12d%s%s%11.2f',...
                    j, num_rep_solves, num_mi_solves,...
                    sciprint(cutoff,'13.3'), sciprint(mincost, '13.3'),...
                                 toc(start_tic)); 
            obj.vprintf(1, disp_txt); 
        end 

        num_sols = pq.size(); % may not have found num_return feas sols 

        for ii = num_sols:-1:1
            [~, sol_struct] = pq.pop(); 
            sol_structs(ii) = sol_struct;
        end 

    end % end optimize standard 

    function [sol_structs, cost_list, mincost, exitflag] = ...
                        optimize_fractional(obj, motors, gearboxes, num_return)
    %
    %
    %
    %
    %

        % Initial ranking handled by reorganizing combos in caller 


        % Would be cool animation to see all the
        % individual lower and upper bounds change 
        % would need some logging 

 
        start_tic = tic; 

        assert(num_return >= 1, 'num_return must be positive int');


        mincost = inf; % no feasible solution yet 

        global_ub = obj.problem_data.cost_ub;   % global upper bound for solution POOL 
        global_lb = obj.problem_data.cost_lb;
        
        num_combinations = length(motors); 

        lb_list = global_lb * ones(num_combinations, 1);  
        %ub_list = global_ub * ones(num_combinations, 1); 
        ub_list = inf(num_combinations, 1); % no valid solutions yet


        % storing solutions separately so dont need to always recompute
        % if bounds already convered for a given design 
        sols = cell(num_combinations, 1); 



        outer_loop_iter = 1; 

        table_line = [repmat('-', 1, 80), '\n']; 
        obj.vprintf(1, table_line);
                % frac_txt = sprintf('%d/%d', j, num_combinations);

        headers1 = ["  |", "Combo x |", " Global |",...
                        "Pool |", "Min  |",  "Rel |",  " "];
        frac_txt = sprintf(' of %d |', num_combinations); 
        headers2 = [" Iter |", frac_txt, " LB  |",...
                        " UB |", "  Cost |" , " Gap |",  " Time "];
        header_txt1 = sprintf('%+7s%+12s%+13s%+13s%+13s%+8s%+7s\n', headers1);
        header_txt2 = sprintf('%+7s%+12s%+13s%+13s%+13s%+8s%+7s\n', headers2);

        obj.vprintf(1, header_txt1);
        obj.vprintf(1, header_txt2);
        obj.vprintf(1, table_line);
        disp_txt = []; 


        max_abs_bnd_diff = inf;
        max_rel_bnd_diff = inf; 
        bad_lower_bound_flag = false;   % flag for initial lower bounds too high
        bad_upper_bound_flag = false;   % flag for initial upper bounds too low 

        % Totsl convrthence criteria 
        while (max_abs_bnd_diff > obj.settings.qcvx_abstol) && ...
              (max_rel_bnd_diff > obj.settings.qcvx_reltol) && ...
               ~bad_lower_bound_flag && ~bad_upper_bound_flag

            pq = PQ(true); % max priority queue -- new every loop 

            for j = 1:num_combinations % Our loop thoufg   
                
                motor = motors(j);
                gearbox = gearboxes(j); 

                % while not converged on this + break condition below
                % Need each upper and lower bound to approach each other every iter

                infeas_lb_flag = false; 
                if outer_loop_iter == 1
                    init_flag = true;  
                end 

                abs_diff_j = ub_list(j) - lb_list(j);
                rel_diff_j = abs_diff_j/min(abs(ub_list(j)), abs(lb_list(j))); 

           
                while (lb_list(j) <= global_ub)  && ~infeas_lb_flag ...
                      &&  (rel_diff_j > obj.settings.qcvx_reltol)  && ...        
                          (abs_diff_j > obj.settings.qcvx_abstol) || init_flag 

                   
                    if init_flag 
                        bound = global_ub;
                        init_flag = false; 
                    else 
                        % upper bound found for this combo may be 
                        % lower than the global boudn for the solution pool 
                        %bound = (lb_list(j) + min(ub_list(j), global_ub))/2; 

                        % Split the difference between lb and ub
                        % BUT if this is higher than the cutoff (global ub)
                        % this is a waste of compuation so do whichever is lower 
                        bound = min((lb_list(j) + ub_list(j))/2, global_ub); 
                    end 

                    % Solve 
                    %
                    
                    [combo_cost, tmp_sol, comp_time] = obj.combo_solve(motor,...
                                                 gearbox, bound); 
             

                    % bound update 
                    if isinf(combo_cost)    % infeasible - update lb 
                        lb_list(j) = bound; % if init bounds infeas, will set lb/ub to same 
                        infeas_lb_flag = true; % will stop moving these bounds till next iter 
                    else    % feasible 
                        % this will always be a better solution than
                        % previously had for this design 
                        sols{j} = tmp_sol;
                        ub_list(j) = bound; 
                    end 
                
                %      ((ub_list(j) - lb_list(j)) > obj.settings.qcvx_reltol*max(min(ub_list(j), global_ub), abs(lb_list(j)))) &&...
                    abs_diff_j = ub_list(j) - lb_list(j); 
                    rel_diff_j = abs_diff_j/min(abs(ub_list(j)), abs(lb_list(j))); 
                end  % end while update bounds for combo j 


                % Check if upper bound for design we just considered goes in the PQ 
                %if ub_list(j) < global_ub   % ie. this goes into sol pool

                if lb_list(j) >= global_ub
                    lb_list(j) = ub_list(j); % just for computing rel diffs 
                end 


                if (ub_list(j) < global_ub) || ...
                    ((ub_list(j) < global_ub + eps) && (pq.size() < num_return))    
                    

                    % Heres THE PLAN
                    % store cell array of solutions for all options 
                    % if solution is found 
                    % then new PQ every outer loop 
                    % even if already converged can grab the data and throw into PQ 
                    sol_struct.cost = ub_list(j); 
                    sol_struct.sol = sols{j};
                    sol_struct.motor = motor; 
                    sol_struct.gearbox = gearbox; 

                    pq.insert(ub_list(j), sol_struct); 


                    if pq.size() > num_return  % if inserting put us over limit 
                        pq.pop(); % remove max element from pq                    
                    end 

                    if pq.size() == num_return
                        % There may be fewer feasible solutions than num return
                        % If so, the global ub is always the init ub 
                        [global_ub, ~] = pq.peek(); % update the cutoff 
                    end 

                    % For the actual minimum cost 
                    if  ub_list(j) < mincost
                        mincost = ub_list(j); 

                        % May have closed the gap and lower bound not low enough 
                        if outer_loop_iter == 1 % only need to check on first 

                            cost_diff = mincost - obj.problem_data.cost_lb;  

                            if (cost_diff < obj.settings.qcvx_abstol) || ...
                                    (cost_diff < obj.settings.qcvx_reltol*...
                                        max(abs([mincost, obj.problem_data.cost_lb])));
                                obj.vprintf(1, ['\nLower bound (lb) is feasible. ',...
                                            'Reduce lower bound and reoptimize']);
                                bad_lower_bound_flag = true; 
                                pq = PQ(true); % clear it -- return nothing 
                                break;  % break the while on this 
                            end 
                        end 
                    end 
                end 


                % Prints 
        %header_txt = sprintf('%+7s%+9s%+13s%+13s%+13s%+10s%+10s%+9s\n', headers);

                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
                disp_txt = sprintf('%6d%12d%s%s%s%8.4f%7.1f',...
                        outer_loop_iter, j, sciprint(global_lb, '13.3'),...
                        sciprint(global_ub,'13.3'), sciprint(mincost, '13.3'),...
                         max_rel_bnd_diff,  toc(start_tic));                
                obj.vprintf(1, disp_txt);
            end    % finish loop through combos 

            bnd_diff = ub_list - lb_list; 
            rel_bound_diff = bnd_diff./max(ub_list, abs(lb_list));
            max_abs_bnd_diff = max(bnd_diff);   % inf - inf = nan
            max_rel_bnd_diff = max(rel_bound_diff); 
            % Update global lower bounds 
            global_lb = min(lb_list); % no design can beat this 

            % update the print 
            obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
            disp_txt = sprintf('%6d%12d%s%s%s%8.4f%7.1f',...
                        outer_loop_iter, j, sciprint(global_lb, '13.3'),...
                        sciprint(global_ub,'13.3'), sciprint(mincost, '13.3'),...
                                max_rel_bnd_diff, toc(start_tic));                
            obj.vprintf(1, disp_txt);

            % If no solution found at end of first outer loop, break 
            if (outer_loop_iter == 1) && isinf(mincost)
                obj.vprintf(1, '\nNo solutions found. Try increasing the upper bound');
                bad_upper_bound_flag = true; 
            end 

            outer_loop_iter = outer_loop_iter + 1; 
            disp_txt = []; 
            obj.vprintf(1, '\n');   % print new line            
        end 


        % TODO -- add a final print here 
        
        num_sols = pq.size(); % may not have found num_return feas sols 

        if num_sols == 0
            sol_structs = struct(); % empty struct; 
        elseif num_sols < num_return
            obj.vprintf(1, 'Only %d feasible solution(s) found\n', num_sols); 
        end 

        for i = num_sols:-1:1
            [~, sol_struct] = pq.pop(); 
            sol_structs(i) = sol_struct;
        end 
        cost_list = ub_list; % NOTE: no pseudocost incorporated here 
        


        if (max_abs_bnd_diff < obj.settings.qcvx_abstol) 
            exitflag = 1; 
        elseif (max_rel_bnd_diff < obj.settings.qcvx_reltol) 
            exitflag = 2; 
        elseif bad_lower_bound_flag
            exitflag = 3;
        elseif bad_upper_bound_flag 
            exitflag = 4; 
        else 
            error('Unknown exit criteria');
        end 
    end  % end optimize fractional 
   

    function [combo_cost, sol, solve_time, exitflag] = combo_solve(obj,...
                                                        motor, gearbox,...
                                                                bound)
    %
    %
    %
    %
    %
    %
    %

    	start_tic = tic; 		% to get timing 
    	[matrices, comp_torques, cutoff, index_map] = ...
                                     obj.build_matrices(motor, gearbox, bound); 

        [y_sol, combo_cost] = obj.socp_solve(matrices, cutoff);
        [sol, bad_idxs, directions] = obj.parse_solution(y_sol,...
                                     index_map, comp_torques,  motor, gearbox); 
        exitflag = 0; % no issues with solve 

        % If mu/gamma decomposition inccorect at some indices, try to fix it 
        if ~isempty(bad_idxs)  % need to run next solve 
            init_cost = combo_cost; 

            [y_sol, combo_cost] = obj.clean_solution(y_sol, combo_cost, index_map,...
                                            matrices, comp_torques, motor, gearbox);
            [sol, bad_idxs_clean, directions] = obj.parse_solution(y_sol,...
                                     index_map, comp_torques,  motor, gearbox); 
            exitflag = 1; 
            
            if ~isempty(bad_idxs_clean)
                disp('cleaning has failed you -- trying fix gm mu')
                [sol, combo_cost, exitflag] = obj.fix_gm_mu_errors(init_cost,...
                                    cutoff, bad_idxs, directions, index_map,...
                                        matrices, comp_torques, motor, gearbox);
            end

        end 
        solve_time = toc(start_tic);
    end 



        % Inputs: combo_cost (initial) directions, bad_idxs, index_map, matrices
        %                                         motor, gearbox(for parsing sol)
        % Outputs: sol, combo_cost, exitflag,  

    function [sol, combo_cost, exitflag] = fix_gm_mu_errors(obj, init_cost,...
                                cutoff, bad_bin_idxs, directions, index_map,...
                                         matrices, comp_torques, motor, gearbox)
    %
    %
    %   Inputs:
    %       init_cost - cost of original solution without fixing any if the bin cals 
    %

        % could distinguish between direct fixing and iterative fixing 
        exitflag = 1; % repeated solve -- will get set higher if fails 

        % Augment the matrices 
        dim_y = length(matrices.ub);

        %nbi = length(bad_idxs); % nbi = number of bad indices 
        %[~, shifted_idxs] = intersect(index_map.binary, bad_idxs, 'stable');


        % NOTE: calling without cuttoffs here - if use cutoff and get 
        % nan result its possible that its feasbile just over the cutoff
        % NOTE: if treating our cutoffs as real though maybe still want a cutoff ohere 
        % think on this more

        % Try fixing the directions to the assumed directions 
        matrices_bin_aug = matrices;    % a copy for augmenting 
        matrices_bin_aug.lb(index_map.del(bad_bin_idxs)) = directions;
        matrices_bin_aug.ub(index_map.del(bad_bin_idxs)) = directions;

        [y_sol_new, combo_cost_new] = obj.socp_solve(matrices_bin_aug, inf);
        [sol_new, bad_idxs_check] = parse_solution(obj, y_sol_new, index_map,...
                                                comp_torques,  motor, gearbox); 
        % 
        %   If cost for fixed version is too different than cost 
        %   for non-fixed version -- call the mixed int solver 
        % 
        cost_diff = combo_cost_new - init_cost; 
        cost_diff_rel = cost_diff/min(abs(init_cost), abs(combo_cost_new));

        % If trying to fix directions failed (inf cost) OR cost was sufficiently
        % different from the initial cost (meaning that this feasible solution
        % might not be close enough to optimal for the mixed integer problem)
        if ((cost_diff_rel > obj.settings.reltol) && ...
            (cost_diff > obj.settings.abstol)) ||  ~isempty(bad_idxs_check)

        
            init_bad_idx = bad_bin_idxs(1);     
    

            bin_indicator = false(length(matrices.ub), 1);
            bin_indicator(index_map.del) = true;   % these values need to be solve binary 

            matrices = presolve(matrices, bin_indicator); 


            [sol, combo_cost, skip_mi_flag] = obj.infeasible_gmmu_search(init_cost, cutoff,....
                            init_bad_idx, index_map, matrices, comp_torques,...
                                                            motor, gearbox);



            % Remove Fixed Augmentation from binary 
            if ~skip_mi_flag

                matrices.binary = bin_indicator;    % tell it to solve mixed int

                disp('cost too diff call mixed into....')

                % as a NOTE, sometimes the presolve for these on mixed int 
                % completely does the trick 
                % perhaps these are even LP presolve tricks 
                % if so, if we can write our own presolver, we could 
                % FIRST check a feasibility presolve 
                % intesting to check if actially is just LP presolve 
                % because presolve is frickin fast 
                %
                % 

                [y_sol_mi, combo_cost_mi] = obj.socp_solve(matrices, cutoff);



                %... For the mixed integer sol -- want to use cutoffs 
                %
                %   really just need to force delta values to binary 
                % ... do we REALLY want to write a seperate B&B module??
                % could always use the ECOS one but its pretty bad
                disp('retuned out with mi sol')
                % keyboard 

                combo_cost = combo_cost_mi; 

                exitflag = 2; 

            elseif (combo_cost < cutoff)

                
                [y_sol_init, combo_cost_init] = obj.socp_solve(matrices, cutoff);

                [sol_init, bad_idxs_init] = parse_solution(obj, y_sol_init, index_map,...
                                                comp_torques,  motor, gearbox); 

                %bin_indicator = false(length(matrices.ub), 1);
                %bin_indicator(index_map.del) = true;   % these values need to be solve binary 
                %matrices.binary = bin_indicator;  

                %[y_sol_mi, combo_cost_mi] = obj.socp_solve(matrices, cutoff);
                %
                %[sol_mi, bad_idxs_mi] = parse_solution(obj, y_sol_mi, index_map,...
                %                                comp_torques,  motor, gearbox); 

                %display('sanity check')
                %display(combo_cost)
                %keyboard
  
            end 
        else % ie. the costs were close enough 
            combo_cost = combo_cost_new; 
            sol = sol_new; 

            disp('fixing worked :)')
            display(directions); 


        end 

        if cost_diff == 0

            disp('how zero -- zero zero?????')

            keyboard
        end 


    end 


    function [sol, combo_cost, skip_mi_flag] = ...
                          infeasible_gmmu_search(obj, init_cost, cutoff, init_bad_idx,...
                             index_map, matrices, comp_torques, motor, gearbox);

        %bad_idxs_init = bad_idxs;  
        %nbi_init = bad_idxs_init; 
        %bad_idxs_comb = unique([bad_idxs; index_map.binary], 'stable');
        %[~, shifted_idxs_comb] = ismember(bad_idxs_comb, index_map.binary); 
        %nbi = length(shifted_idxs_comb);            
        disp('infeas_gmmu_search')

        skip_mi_flag = false; % will set to true if we prove infeasible 

        num_bin = length(index_map.del); 


        %del_aug_ub = ones(num_bin, 1);      % upper bound for feasible assignments 
        %del_aug_lb = zeros(num_bin, 1);     % lower bound for feasible assignments


        % ADD AN LP Presolve to hopefully speed things up - will need to profile 
        % -- would require sectioning this out even further
        % maybe make a function "infeasibility search"

        % if after these steps the upper and lower bound of feasible assignments
        % is the same AND we we have not created any new bad idxs, then
        % there exists only 1 feasible mixed integer solution and we do 
        % not need to call the mixed integer solver
        % TODO -- clean way of moving this out to function
        % so can call again in the worst case 



        %next_idx = shifted_idxs_comb(1);

        %for i = 1:nbi  % NOTE: combined bad idxs -- actually go through all of them 
        

        % Loop through all POTENTIALLY bad idxs, and break early if whatever criteria met 

        combo_cost_one = inf; 
        combo_cost_zero = inf; 

        check_order_dbg = [];

        %for i = 1:num_bin           % maybe switch to awhile 

        max_outer_loops = 3; % check and profile 
        outer_iter = 1;

        % Will want to break before hit a cylce 
        % if outer loops stop moving bounds -- TODO -- add a compare 
        % check for cycles 
        while outer_iter <= max_outer_loops  && ~skip_mi_flag % other cond?? 

            num_checks = 0; 
            del_checked = false(num_bin, 1); % indicator of if shifted idx has been checked 

            % set a next idx 

            if outer_iter == 1
                next_idx = init_bad_idx;    % next one to check 
            else 
                if ~isempty(bad_idxs_zero) && ~isempty(bad_idxs_one) % neither empty
                    % max below because may be empty
                    % could prbably combine the next elseifs into this condiion
                    if tau_error_one(1) > tau_error_zero(1)
                        next_idx = bad_idxs_one(1);
                    else 
                        next_idx = bad_idxs_zero(1); 
                    end 
                elseif isempty(bad_idxs_zero) 
                    next_idx = bad_idxs_one(1);
                elseif isempty(bad_idxs_one)
                    next_idx = bad_idxs_zero(1);
                else 
                    disp('shouildnt be posssible')
                    keyboard
                end 
            end 

            while ~isempty(next_idx) && (num_checks < num_bin) % may want to allow repeated checking 

                % If we have a feas sol already (ie. one of the is not inf)
                % then we should treat any varaible fixing that is higher as 
                % infeasible 
                feas_cost = min(combo_cost_zero, combo_cost_one);
                feas_cutoff = feas_cost + 1e-5; % adding nomimal cost for tolerance


                zero_infeas = false;
                one_infeas = false; 

                check_order_dbg = [check_order_dbg, next_idx];
                del_idx = index_map.del(next_idx);  % true index in actual problem bounds 

                
                %display(num_checks)
                %display(next_idx)
                %display(del_idx)
                if del_checked(next_idx)
                    disp('trying to fix and index aleady fixed')     
                    keyboard
                    error('wwwaaaaa')
                end 
                del_checked(next_idx) = true;
                

                %
                %       Try fixing it at zero
                %
                matrices.lb(del_idx) = 0; 
                matrices.ub(del_idx) = 0; % upper bound at 0
                [y_sol_zero, tmp_cost_zero] = obj.socp_solve(matrices, feas_cutoff);

                if isinf(tmp_cost_zero)
                    zero_infeas = true;     % this value cannot be zero 
                    bad_idxs_zero = []; % dont bother parsing 
                else

                    [sol_zero, bad_idxs_zero, directions_zero, tau_error_zero] = ...
                                parse_solution(obj, y_sol_zero, index_map,...
                                                    comp_torques,  motor, gearbox);
                    if ~isempty(bad_idxs_zero) % try to clean and repase 
                        [y_sol_zero, tmp_cost_zero] = obj.clean_solution(y_sol_zero,...
                                         inf, index_map, matrices, comp_torques,...
                                                                 motor, gearbox);
                        [sol_zero, bad_idxs_zero, directions_zero, tau_error_zero] = ...
                                parse_solution(obj, y_sol_zero, index_map,...
                                                    comp_torques,  motor, gearbox);
                    end 

                    if isempty(bad_idxs_zero)
                        combo_cost_zero = tmp_cost_zero; 
                    end 
                end 
                 

                % other thing to note: any integer feasible solution can be used 
                % as a cutoff 


                %
                %        Try fixing it at one 
                %
                matrices.lb(del_idx) = 1;    % lower bound at 1 
                matrices.ub(del_idx) = 1;
                [y_sol_one, tmp_cost_one] = obj.socp_solve(matrices, feas_cutoff);
                if isinf(tmp_cost_one)
                    one_infeas = true;
                    bad_idxs_one = []; % dont bother parsing 
                else 
                    [sol_one, bad_idxs_one, directions_one, tau_error_one] = ...
                                parse_solution(obj, y_sol_one, index_map,...
                                                    comp_torques,  motor, gearbox);
                    if ~isempty(bad_idxs_one) % trey to clean and reparse 
                        [y_sol_ones, tmp_cost_one] = obj.clean_solution(y_sol_one,...
                                         inf, index_map, matrices, comp_torques,...
                                                                     motor, gearbox);
                        [sol_one, bad_idxs_one, directions_one, tau_error_one] = ...
                                parse_solution(obj, y_sol_one, index_map,...
                                                    comp_torques,  motor, gearbox);
                    end 
                    
                    if isempty(bad_idxs_one)  % int feas 
                        combo_cost_one = tmp_cost_one; % might be old 
                    end 
                end  

                if zero_infeas && one_infeas     % we've proven infeasibility 
                    combo_cost = inf; 
                    skip_mi_flag = true; 
                    sol = struct(); 
                    break; 
                elseif zero_infeas   % but 1 is feasible 
                    matrices.lb(del_idx) = 1;
                    matrices.ub(del_idx) = 1; 
                    [~, ~, next_idx] = find(bad_idxs_one.*~del_checked(bad_idxs_one),1);  
                elseif one_infeas   % but zero is feasiblbe 
                    matrices.lb(del_idx) = 0;
                    matrices.ub(del_idx) = 0;
                    [~, ~, next_idx] = find(bad_idxs_zero.*~del_checked(bad_idxs_zero),1);               
                else    % both feasible -- but one may have no bad indices 
                    matrices.lb(del_idx) = 0;
                    matrices.ub(del_idx) = 1; 

                    [i1, ~, next_idx_one] = find(bad_idxs_one.*~del_checked(bad_idxs_one),1);  
                    [i0, ~, next_idx_zero] = find(bad_idxs_zero.*~del_checked(bad_idxs_zero),1);  

                    % max below because may be empty
                    if max([tau_error_one(i1),0]) > max([tau_error_zero(i0),0])
                        next_idx = next_idx_one;
                    else 
                        next_idx = next_idx_zero; 
                    end 
                end 

                tmp_cost = min(combo_cost_one, combo_cost_zero); 
                cost_diff = tmp_cost - init_cost;
                cost_diff_rel = cost_diff/min(abs(tmp_cost), abs(init_cost));
                
                if (cost_diff_rel < obj.settings.reltol) || ...
                                            (cost_diff < obj.settings.abstol) 

                    close_enough = true;
                else 
                    close_enough = false; 
                end 

                % TODO -- add costs checks -- if cost close enough to init cost 
                % ONLY for an integer feasibe then done. may not need though? 

                % if we get here at all, it means at least 0 or 1 feasible on most recent 
                % also, costs ONLY get worse as we fix more vars
                % might have checked them all but not actually a int feas
                if (isempty(bad_idxs_one) && isempty(bad_idxs_zero)) || close_enough  %  WE HAVE A SOLUTION THATS EQUIVALENT TO AN INTEGEVER FEASIBILE SOLUTON  
                    skip_mi_flag = true; % we have found a feasible mixed int solutino
                    % that is not worse than any other feasible 
                    % NOTE: coulld be both feasible in which case pick lowest cost 
                    if tmp_cost_zero <= tmp_cost_one 
                        sol = sol_zero; 
                        combo_cost = tmp_cost_zero; 
                    else % combo_cost_one < combo_cost_zero
                        sol = sol_one; 
                        combo_cost = tmp_cost_one; 
                    end 
                    disp('can skip - over here!!!!!!!!')
                    %keyboard
                    break;
                    % NOTE would be good to perform a sanity check by fixinf the 
                    % vars that this does in fact correspond to a true mixed int feasible soluib
                end 
                % probliem with current setup -- if an index could be feasible 
                % either way, one we move past it, it could come up again 
                % ok lets say this did happen, we never get it twice in a row 
                % should we just deal with getting it again? we only ever make
                % the problem tighter -- however, consdier a case where lets say
                % 2 indices independanty could be ambiguous with same costs -- would keep bouncing between them 
                % this would be a case where mixed integer is really the only way 
                % 

                % also adding checks for cost
                

                num_checks = num_checks + 1; 

            end 

            outer_iter = outer_iter + 1; 
        end 


        if ~skip_mi_flag  % still false 
            sol = struct([]); 
            combo_cost = nan; 
            display(check_order_dbg)
            disp('wasnt able to prove shgit ')
            keyboard 
        end 

        %{
        if ~any(del_aug_ub - del_aug_lb) % if bounds same - we have out sol
            % if these are the same it means we made it all the way 
            % through the bad idxs and only found 1 feasible solution
            % if this new solution doesnt create any new bad idxs we are done 
            % NOTE: this now reduendatnt with the parsing done above 
            [sol_new, bad_idxs_check, directions_check, tau_error_check] = ...
                            parse_solution(obj, y_sol_new, index_map,...
                                                comp_torques,  motor, gearbox);
            sol = sol_new; 
            combo_cost = combo_cost_new; 

            if isempty(bad_idxs_check)  % solution is mixed-integer feasible 
                skip_mi_flag = true; 
            end 
        elseif ~skip_mi_flag % We have not shown infeasible nor that only 1 sol exists 
            sol = struct([]); 
            combo_cost = nan;   
            disp('boo')
        end  
        %} 

    end % end infeasible gmmu search 




    function [matrices, compensation_torques, cutoff, index_map]  = ...
                                    build_matrices(obj, motor, gearbox, bound)
    %
    %
    %		Builds the matrices for the problems 
    %
    %
    %
     % rename cutoff to "bound" and then check problem type 
        if strcmp(obj.problem_type, 'standard')
            cutoff = bound; 
        elseif strcmp(obj.problem_type, 'fractional')
            cutoff = inf; 
            assert(~isinf(bound), 'Bound must be finite for fractional problems');
        else 
            error('Invalid problem type');
        end 


        % Big block comment otlinin whats going on here 

        %
        %
        %
        %
        %		 A review of the math would be great  
        %
        %
        %
        %
        %

        pd = obj.problem_data;  % making a local copy has HUGE positive impact on speed 

        % How we actually "solve" will depend on solver
        % Gurobi OR Sedumi OR ECOS 
        % 
        % SOCP implicit for Gurobi solve (looks like a stanrd QCQP)
        omega = pd.omega; 
        omega_dot = pd.omega_dot; % may be all zeros 

        eta = gearbox.efficiency; 
        ratio = gearbox.ratio .* gearbox.direction; 
        k_t = motor.k_t; 

        % calculate current limits 
        I_u = min(pd.I_max, (pd.V_max - k_t*ratio*omega)/motor.R); 
        I_l = max(-pd.I_max, (-pd.V_max - k_t*ratio*omega)/motor.R);
        %Hmm, and if negative? 

        H = pd.H(motor, gearbox);
        b = pd.b(motor, gearbox);
        H_ineq = pd.H_ineq(motor, gearbox);
        b_ineq = pd.b_ineq(motor, gearbox); 
         % TODO -- remove confusion between A/b in solving socp and H/b in problem setuo
         % probablyt keep A/b and rename the 'b' in the problem setup context 

        T = pd.T(motor, gearbox);
        d = pd.d(motor, gearbox); 

        tau_gearbox_inertia = gearbox.inertia * (ratio^2) * omega_dot;  % multiply ny ratio to add to output 
        

        % dimensions 
        n = pd.n; 
        w = pd.w;
        p = pd.p;
        m = pd.m; 

        zero_vel_idxs = pd.zero_vel_idxs; 
        nzvi = length(zero_vel_idxs); 

        % NOTE the distinction at omega = 0 (also, could precomp)
        sign_omega = sign(omega); 		% = 0 if omega = 0 
        so = round(sign_omega + 0.1); 	% = 1 if omega = 0  

        %
        % 		Friction/Drag and Inertia Compensation
        %

        % TODO -- whats the best general way to handle static friction
        % Should there be a call to a function which determines 
        % friction model based of settings? 
        


        % tau_motor_friction is JUST kinetic 
        [tau_motor_friction, tau_static_friction] = obj.friction_model(motor, gearbox); 


        % Since BEFORE gearbox only multiplied by ratio NOT ratio^2 
        % will effectively get multiplied by ratio again through gearbox
        tau_motor_inertia = motor.inertia * ratio * omega_dot; 
        
  		% Dynamic Motor Friction and Inertia Compensation  (f - fric, j- inerits )
        % TODO -- rename this guy and move into frictino model function
        % and have it be returned by that 

        bo = 1e-3; % bound offset for upper bounds to make it more clear 
        % which bound is directly responsible for infeasibility 

        tau_mfj = tau_motor_friction + tau_motor_inertia; 
        I_comp = tau_mfj/k_t; % corresponding current 

        % FOR SOME DEBUG 

        % Index Map for Optimization 
        I_idxs = 1:n;
        Isq_idxs = I_idxs(end) + (1:n);	 % equivalent to I squared 
        tau_idxs = Isq_idxs(end) + (1:n); 

        if nzvi > 0 
            sf_idxs = tau_idxs(end) + (1:nzvi);      % for static friction (sf)
            x_idxs = sf_idxs(end) + (1:w); 
        else 
            sf_idxs = []; 
            x_idxs = tau_idxs(end) + (1:w);
        end 

        index_map.I = I_idxs; 
        index_map.Isq = Isq_idxs; 
        index_map.tau = tau_idxs;       % where tau = Tx + d AND Tau + tau_gb_inertia = ratio*kt(eta*gamma + mu/eta)
        index_map.sf = sf_idxs;
        index_map.x = x_idxs;

        Ico = I_comp.*omega;        
        binary_aug = false; 
        if eta < 1
        	gm_idxs = x_idxs(end) + (1:n);
            mu_idxs = gm_idxs(end) + (1:n); 
            s_idxs = mu_idxs(end) + (1:n);  

        	index_map.gm = gm_idxs;
            index_map.mu = mu_idxs; 
            index_map.s = s_idxs; 

        	if min(Ico) >= 0
            	dim_y = s_idxs(end) + nzvi;  % total dimension of opt vector 
        		lb = -inf(dim_y, 1);		
        		ub = inf(dim_y, 1); 
        	else 
        		% TODO -- ONLY ADD WHERE NEEDED TO REDUCE PROBLEM
        		% SIZE AND REDUNDANT CONSTRAINTS 
                binary_aug = true;  % Add more variables 

                % NOTE: this does NOT include omega = 0
                aug_idxs = find(Ico < 0); % indices where we need the extra help 
                num_aug = length(aug_idxs);


                index_map.binary = aug_idxs; % NOTE: unlike everything else NOT an index into sol vector

                del_idxs = s_idxs(end) + (1:num_aug); 
                sig_idxs = del_idxs(end) + (1:num_aug); 
                gmsq_idxs = sig_idxs(end) + (1:num_aug);
                musq_idxs = gmsq_idxs(end) + (1:num_aug);

            	index_map.del = del_idxs;
            	index_map.sig = sig_idxs;
            	index_map.gmsq = gmsq_idxs;
            	index_map.musq = musq_idxs; 

            	dim_y = musq_idxs(end);  % total dimension of opt vector 

            	% Bounds on optimizatoin vatiable 
        		lb = -inf(dim_y, 1);		
        		ub = inf(dim_y, 1); 
            	
                lb(gmsq_idxs) = 0;  
                ub(gmsq_idxs) = max((I_l(aug_idxs) - I_comp(aug_idxs)).^2,...
                                         (I_u(aug_idxs) - I_comp(aug_idxs)).^2) + bo; 
                lb(musq_idxs) = 0;
                ub(musq_idxs) = ub(gmsq_idxs); 

				lb(del_idxs) = 0; 	ub(del_idxs) = 1; 
				lb(sig_idxs) = 0; 	ub(sig_idxs) = 1; 
       	
        	end 

        	% Bounds on gm, mu, s 
			lb(gm_idxs) = min(-sign_omega .* (I_l - I_comp- bo), 0);
			ub(gm_idxs) = max(sign_omega .* (I_u - I_comp + bo), 0);

			lb(mu_idxs) = min(so .* (I_l - I_comp - bo), 0);
			ub(mu_idxs) = max(-so .* (I_u - I_comp + bo), 0);
            
			lb(s_idxs) = 0; 
			ub(s_idxs) = max((I_l - I_comp).^2, (I_u - I_comp).^2) + bo; 

			%
            %   Rows 1:n        Tx + d = tau_out  (delivered to robot)
            %   Rows n+1:2n     tau_out + tau_gb_inertia = motor/gb torque  
			%
            % sign of motor/gb torque term determines driving/driven 		
            num_vals = nnz(T) + 4*n + nzvi; 
            A_eq_tau_x = sparse([], [], [], 2*n, dim_y, num_vals);
            b_eq_tau_x = zeros(2*n, 1);     % to be filled in 

            A_eq_tau_x(1:n, tau_idxs) = eye(n);
            A_eq_tau_x(1:n, x_idxs) = -T; 
            b_eq_tau_x(1:n) = d; 

            A_eq_tau_x(n + (1:n), tau_idxs) = -eye(n); 
            A_eq_tau_x(n + (1:n), gm_idxs) = ratio*eta*k_t*eye(n);
            A_eq_tau_x(n + (1:n), mu_idxs) = (ratio*k_t/eta)*eye(n);
            b_eq_tau_x(n + (1:n)) = tau_gearbox_inertia;

           
            %		
            % 			gamma + mu = I - I_comp   % may want to define I shift
            %
            %
            A_eq_gm_mu = sparse([], [], [], n, dim_y, 3*n);  
            A_eq_gm_mu(:, gm_idxs) = eye(n);
            A_eq_gm_mu(:, mu_idxs) = eye(n);
            A_eq_gm_mu(:, I_idxs) = -eye(n); 
            b_eq_gm_mu = -I_comp; 


			%
			%		s + 2I*I_comp - I_comp^2 = Isq
			%
			A_eq_cost = sparse([], [], [], n, dim_y, 3*n); % might renmae 
			A_eq_cost(:, s_idxs) = eye(n);
			A_eq_cost(:, I_idxs) = 2*diag(I_comp);
			A_eq_cost(:, Isq_idxs) = -eye(n);
			b_eq_cost = I_comp.^2; 

      		A_eq_other = [A_eq_gm_mu; A_eq_cost];
            b_eq_other = [b_eq_gm_mu; b_eq_cost];

            %
            %		Extra Linear Constraints 
            %
            if binary_aug 

                % For assignment of delta with driving/driven 
				%epss = 1e-12; % tolerancing 
                epss = 0;
				%
				%    Relate Delto to Driving/Driven 
				%		[Delta = 1] <===> [Driving]
				%	
				% 	When all else fails, we will force Delta 
				%   to take on binary values and call a mixed integer
				%	solver to handle this 

                % ONLY DO AT INDICES WHERE ITS NEEDED 
                so_aug = so(aug_idxs); 
                gm_aug_idxs = gm_idxs(aug_idxs);  % USEFUL ABOVE TODO 
                mu_aug_idxs = mu_idxs(aug_idxs); 

                % f(x) = -sign(omega)(I - I_comp)
                I_comp_aug = I_comp(aug_idxs);
                I_u_aug = I_u(aug_idxs);
                I_l_aug = I_l(aug_idxs);

                f_max = I_u_aug + so_aug.*I_comp_aug + 0.1;
                f_min = I_l_aug + so_aug.*I_comp_aug - 0.1;

                A_del = zeros(2*num_aug, dim_y);
                
                A_del(1:num_aug, I_idxs(aug_idxs)) = -diag(so_aug);
                A_del(1:num_aug, del_idxs) = diag(f_max);

                A_del(num_aug + (1:num_aug), I_idxs(aug_idxs)) = diag(so_aug);
                A_del(num_aug + (1:num_aug), del_idxs) = diag(f_min - epss);

                b_del = [f_max - I_comp_aug.*so_aug; so_aug.*I_comp_aug - epss];

				% gm \equiv to delta*(I - I_comp)
                %tmp_max = I_u_aug - I_comp_aug + 0.1;
                %tmp_min = I_l_aug - I_comp_aug - 0.1;     


                tmp_max = I_u_aug - I_comp_aug + 0.1;
                tmp_min = I_l_aug - I_comp_aug - 0.1;     

                %A_del_gm = zeros(4*num_aug, dim_y);
                %
                %   comment on the paper 
                %
                %
                A_del_gm = zeros(2*num_aug, dim_y);

                A_del_gm(1:num_aug, gm_aug_idxs) = eye(num_aug);
                A_del_gm(1:num_aug, del_idxs) = -diag(tmp_min);
                A_del_gm(1:num_aug, I_idxs(aug_idxs)) = -eye(num_aug);
            
                A_del_gm(num_aug + (1:num_aug), gm_aug_idxs) = -eye(num_aug);
                A_del_gm(num_aug + (1:num_aug), del_idxs) = diag(tmp_max);
                A_del_gm(num_aug + (1:num_aug), I_idxs(aug_idxs)) = eye(num_aug);

                %{
                A_del_gm(2*num_aug + (1:num_aug), gm_aug_idxs) = eye(num_aug);
                A_del_gm(2*num_aug + (1:num_aug), del_idxs) = -diag(tmp_max);

                A_del_gm(3*num_aug + (1:num_aug), gm_aug_idxs) = -eye(num_aug);
                A_del_gm(3*num_aug + (1:num_aug), del_idxs) = diag(tmp_min);

                b_del_gm = [-I_comp_aug - tmp_min; I_comp_aug + tmp_max;...
                                        zeros(2*num_aug, 1) ];
			    %}
                b_del_gm = [-I_comp_aug - tmp_min; I_comp_aug + tmp_max];

                %A_del = [];
                %b_del = [];
                %A_del_gm = [];
                %b_del_gm = [];



                % TODO SPEED UP             
				A_ineq_other = [A_del; A_del_gm];
				b_ineq_other = [b_del; b_del_gm];


                %
                %        delta + sigma = 1 
                %
                A_del_sig = zeros(num_aug, dim_y);
                A_del_sig(:, del_idxs) = eye(num_aug);
                A_del_sig(:, sig_idxs) = eye(num_aug);
                b_del_sig = ones(num_aug, 1);

            
                %
                %        gm_sq + mu_sq = s 
                %
                A_gmmu_sq = zeros(num_aug, dim_y);
                A_gmmu_sq(:, gmsq_idxs) = eye(num_aug);
                A_gmmu_sq(:, musq_idxs) = eye(num_aug);
                A_gmmu_sq(:, s_idxs(aug_idxs)) = -eye(num_aug);
                b_gmmu_sq = zeros(num_aug, 1);


				A_eq_other = [A_eq_other; A_del_sig; A_gmmu_sq];
				b_eq_other = [b_eq_other; b_del_sig; b_gmmu_sq];
			else 
                A_ineq_other = [];
                b_ineq_other = [];
            end 
        else  % NO GEARBOX (Direct Drive) -- explicitly encode because faster
            dim_y = x_idxs(end); 
            lb = -inf(dim_y, 1);		
        	ub = inf(dim_y, 1);

            A_eq_tau_x = zeros(2*n, dim_y);    % sparse? 

            b_eq_tau_x = zeros(2*n, 1);
            A_eq_tau_x(1:n, tau_idxs) = eye(n);
            A_eq_tau_x(1:n, x_idxs) = -T; 
            b_eq_tau_x(1:n) = d; 

            A_eq_tau_x(n + (1:n), tau_idxs) = -eye(n);
            A_eq_tau_x(n + (1:n), I_idxs) = ratio*k_t*eye(n);
            b_eq_tau_x(n + (1:n)) = 0; % no gb inertia to account for 

            A_eq_other = [];	% no others 
            b_eq_other = []; 
            A_ineq_other = [];	% no others 
            b_ineq_other = []; 
        end 

        % Fill in variable bounds on I, Isq
        lb(I_idxs) = I_l; 
        ub(I_idxs) = I_u; 

        lb(Isq_idxs) = 0;    % non-negativity 
        ub(Isq_idxs) = max(abs(I_u), abs(I_l)).^2; 

        lb(tau_idxs) = -gearbox.max_int_torque;
        ub(tau_idxs) = gearbox.max_int_torque; 

        %
        %
        %		Account for Static Friction Cone
        %		for points with zero velocity
        %
        %		Adjust A_eq_tau_x and b_eq_tau_x accordingly
        %
        if nzvi > 0 
        	% TODO -- probably consider gb fully efficient at zero vel?
        	% think through a little more (but then just ratio on next line)

            % add the static frinction value (to be computed) into the torque equlity 
            % for the motor itslef
            A_eq_tau_x(n + zero_vel_idxs, sf_idxs) = ratio*eye(nzvi); 

            % Overwrite the other zero vel points, treat gb as fully eff 
            if eta < 1
            	% Replace ratio*kt*eta with ratio*kt on those rows 
            	A_eq_tau_x(n + zero_vel_idxs, gm_idxs(1) + zero_vel_idxs - 1) = ratio*k_t*eye(nzvi);
            	A_eq_tau_x(n + zero_vel_idxs, mu_idxs) = 0;  % dont consider it driven at this step
            end 

            % Static Friction Bounded 
            ub(sf_idxs) = tau_static_friction;
            lb(sf_idxs) = -tau_static_friction; 
        end 

        if ~isempty(H)
            A_eq_H = sparse([], [], [], size(H, 1), dim_y, nnz(H));
            A_eq_H(:, x_idxs) = -H;   % note signs
            b_eq_H = b; 
        else 
            A_eq_H = [];
            b_eq_H = [];
        end

        % very slow 
        A_eq = [A_eq_tau_x; A_eq_other; A_eq_H]; 
        b_eq = [b_eq_tau_x; b_eq_other; b_eq_H]; % NOTE signs 

        % there is some processing to do here for if 
        % general constraints are SOCP OR QCP or simple inequality 
        % Dont want to need to redo analyis 

        % COULD define a seperate general inequality 
        % for when all quadratic terms are empty
        % fill this up in preprocessing and then this step
        % becomes easier 
        num_lin_ineq = nnz(pd.lin_ineq);
        if num_lin_ineq > 0 
            A_ineq_extra = zeros(num_lin_ineq, dim_y);
            b_ineq_extra = zeros(num_lin_ineq, 1); 
            idxs = find(pd.lin_ineq); 
        else 
            A_ineq_extra = []; 
            b_ineq_extra = [];
        end 

        
        quadcon = struct([]); % struct for quadratic/SOC constraints 
      	
      	%
      	%		Incorporate Ineqaulity Constraints 
        %		Specified with Q, r, M, c, beta 
        %
        %qc_idx = 0;   % n
        qc_idx = numel(quadcon);

        lin_ineq_idx = 0; % will increment 
        for j = 1:m    % incorporate the quadratic constraints + other linear ineqs 
            cj = pd.c{j + 1};  % +1 bc 1 indexing 
            if isa(cj, 'function_handle')
                c_tmp = cj(motor, gearbox); 
            else 
                c_tmp = cj;
            end 

            rj = pd.r{j + 1};   % WHY SO SLOW ? - NOT EMPTY             
            if isa(rj, 'function_handle')
                r_tmp = rj(motor, gearbox); % slow 
            else 
                r_tmp = rj; 
            end 
            
            bet = pd.beta{j + 1}; 
            if isa(bet, 'function_handle')
                beta_tmp = bet(motor, gearbox); % slow 
            else 
                beta_tmp = bet; 
            end 

            q_tmp = zeros(dim_y, 1); 
            q_tmp(I_idxs) = c_tmp; 
            q_tmp(Isq_idxs) =  pd.Q{j + 1}(motor, gearbox); 
            q_tmp(x_idxs) = r_tmp; 

            if pd.lin_ineq(j)  % if linear 
                lin_ineq_idx = lin_ineq_idx + 1; 
                A_ineq_extra(lin_ineq_idx, :) = q_tmp'; 
                b_ineq_extra(lin_ineq_idx) = -beta_tmp;
            else  % quad constraints 
                qc_idx = qc_idx + 1; 

                Mj_tmp = pd.M{j + 1}(motor, gearbox); 

                num_vals = nnz(Mj_tmp); 
                
                % NOTE -- could speed these up 
                Qc_tmp = sparse([], [], [], dim_y, dim_y, num_vals, nnz(Mj_tmp));
                Qc_tmp(x_idxs, x_idxs) = Mj_tmp; 

                quadcon(qc_idx).Qc = Qc_tmp; 
                quadcon(qc_idx).q = q_tmp; 
                quadcon(qc_idx).sense = '<'; 
                quadcon(qc_idx).rhs = -beta_tmp; 
            end 

        end 

       
        % TODO -- initial correct size of quadon 
        % TODO -- move this up and group all the if eta < 1 code 
	    qc_idx = numel(quadcon);
        if eta < 1 
        	%
        	% 			Add Quadratic Constraits 
        	%				(gamma - mu)^2 <= s

	        for i = 1:n 
		        qc_idx = qc_idx + 1; 

		        row = [gm_idxs(i); gm_idxs(i); mu_idxs(i); mu_idxs(i)];
		        col = [gm_idxs(i); mu_idxs(i); gm_idxs(i); mu_idxs(i)];
		        vals = [1; -1; -1; 1]; 

		        % gm^2 - 2 gm*mu + mu_^2 
	            Qc_tmp = sparse(row, col, vals, dim_y, dim_y); 
		        q_tmp = zeros(dim_y, 1);
		        q_tmp(s_idxs(i)) = -1; 

		        quadcon(qc_idx).Qc = Qc_tmp; 
	            quadcon(qc_idx).q = q_tmp; 
	            quadcon(qc_idx).sense = '<'; 
	            quadcon(qc_idx).rhs = 0; 
	        end 

	        %... its possible dont need both of these 
	        %
	        %      Rotated SOC Constraints 
	        %		gm^2 <= gm_sq.*delta
	        %		mu^2 <= mu_sq.*sigma
            %
            %
            %   NOTE: not clear that we need this, may be fine without??
            % This may actually be detrimental? Lets try with crazier accels   
            if binary_aug  % These can help -- without this get stuck in a recusion
            % when trying to clean solutions                         
                for i = 1:num_aug 

	        		% gm^2 <= gm_sq .*delta 
                    qc_idx = qc_idx + 1; 
                    row = [gm_aug_idxs(i); gmsq_idxs(i); del_idxs(i)];
                    col = [gm_aug_idxs(i); del_idxs(i); gmsq_idxs(i)];
                    vals = [1; -0.5; -0.5]; 
                    Qc_tmp = sparse(row, col, vals, dim_y, dim_y);
                    quadcon(qc_idx).Qc = Qc_tmp; 
                    quadcon(qc_idx).q = zeros(dim_y, 1);
                    quadcon(qc_idx).sense = '<'; 
                    quadcon(qc_idx).rhs = 0; 

		            % mu^2 <= mu_sq .*sigma 
                    qc_idx = qc_idx + 1; 
                    row = [mu_aug_idxs(i); musq_idxs(i); sig_idxs(i)];
                    col = [mu_aug_idxs(i); sig_idxs(i); musq_idxs(i)];
                    vals = [1; -0.5; -0.5]; 
                    Qc_tmp = sparse(row, col, vals, dim_y, dim_y);
                    quadcon(qc_idx).Qc = Qc_tmp; 
                    quadcon(qc_idx).q = zeros(dim_y, 1);
                    quadcon(qc_idx).sense = '<'; 
                    quadcon(qc_idx).rhs = 0; 
		        end 
	        end

        end 
        %{
	    else    % eta = 1 
	    	%
	    	%	Add quadratic constraint I^2 <= I_sq 
	    	%	
	    	for i = 1:n 
		        qc_idx = qc_idx + 1; 

	            Qc_tmp = sparse(I_idxs(i), I_idxs(i), 1, dim_y, dim_y); 
		        q_tmp = zeros(dim_y, 1);
		        q_tmp(Isq_idxs(i)) = -1; 

		        quadcon(qc_idx).Qc = Qc_tmp; 
	            quadcon(qc_idx).q = q_tmp; 
	            quadcon(qc_idx).sense = '<'; 
	            quadcon(qc_idx).rhs = 0; 
	        end 
	    end 
        %} 
        %9/14/20 -- add regarcelss 
        for i = 1:n 
            qc_idx = qc_idx + 1; 

            Qc_tmp = sparse(I_idxs(i), I_idxs(i), 1, dim_y, dim_y); 
            q_tmp = zeros(dim_y, 1);
            q_tmp(Isq_idxs(i)) = -1; 

            quadcon(qc_idx).Qc = Qc_tmp; 
            quadcon(qc_idx).q = q_tmp; 
            quadcon(qc_idx).sense = '<'; 
            quadcon(qc_idx).rhs = 0; 
        end


        if strcmp(obj.problem_type, 'fractional')
            % Augment H with equality 
            A_lin_frac = zeros(2, dim_y); 
            A_lin_frac(1, x_idxs) = pd.r_num - bound*pd.r_den;
            
            % Constrain denominator to be positive  
            A_lin_frac(2, x_idxs) = -pd.r_den; % nonnegativitiy 
            b_lin_frac = [bound*pd.beta_den - pd.beta_num; pd.beta_den];
        
        elseif strcmp(obj.problem_type, 'standard')
            A_lin_frac = []; 
            b_lin_frac = []; 
        else 
            error('Invalid Problem type ');
        end 

        % Add the linear inequalities in H_ineq, b_ineq
        if ~isempty(H_ineq)
        	% Add these to A_ineq other 
        	A_ineq_H = sparse([], [], [], size(H_ineq, 1), dim_y, nnz(H_ineq));
        	A_ineq_H(:, x_idxs) = H_ineq; 
        	b_ineq_H = b_ineq; 

        	A_ineq_extra = [A_ineq_extra; A_ineq_H];
        	b_ineq_extra = [b_ineq_extra;  -b_ineq_H]; % NOTE signs 
        end  

        %% ----- Incorporate Actual Cost --------------
        [M0_row, M0_col, M0_val] = find(pd.M{1}(motor, gearbox)); 
        Q_cost = sparse(x_idxs(1) + M0_row - 1, x_idxs(1) + M0_col - 1,...
        									 M0_val, dim_y, dim_y); 

        r0_tmp = pd.r{1};
        if isa(r0_tmp, 'function_handle')
            r0 = r0_tmp(motor, gearbox)
        else 
            r0 = r0_tmp;
        end 
        
        beta0_tmp = pd.beta{1}; 
        if isa(beta0_tmp, 'function_handle')
            beta0 = beta0_tmp(motor, gearbox)
        else 
            beta0 = beta0_tmp;
        end 
        
        Q0 = pd.Q{1}(motor, gearbox); % diag vector 
        c0_tmp = pd.c{1};
        if isa(c0_tmp, 'function_handle')
            c0 = c0_tmp(motor, gearbox)
        else 
            c0 = c0_tmp;
        end 
        
        c_cost_init = zeros(dim_y, 1); 
        c_cost_init(I_idxs) = c0; 
        c_cost_init(Isq_idxs) = Q0; % diag 
        c_cost_init(x_idxs) = r0; 

        c_rho_aug = zeros(dim_y, 1); 

        if eta < 1  % TODO -- group with other things 
        	c_rho_aug(gm_idxs) = so.* obj.solve_settings.rho; 
        	c_rho_aug(mu_idxs) = -so .* obj.solve_settings.rho; 
        end 

        c_cost = c_cost_init + c_rho_aug;

        %
        %	
        %		Assign Outputs 
        %
        %
    	matrices.Q_cost = Q_cost;
    	matrices.c_cost = c_cost; 
    	matrices.c_rho_aug = c_rho_aug; % 
    	matrices.beta0 = beta0;
    	matrices.A_eq = A_eq;
    	matrices.b_eq = b_eq; 
    	matrices.A_ineq = [A_ineq_other; A_ineq_extra; A_lin_frac];
    	matrices.b_ineq = [b_ineq_other; b_ineq_extra; b_lin_frac];
    	matrices.lb = lb; 
    	matrices.ub = ub; 
    	matrices.quadcon = quadcon; 

       
    	% 
        compensation_torques.I_comp = I_comp;  % maybe inclue in comp values? 
    	compensation_torques.tau_motor_friction = tau_motor_friction;
    	compensation_torques.tau_motor_inertia = tau_motor_inertia; 
    	compensation_torques.tau_gearbox_inertia = tau_gearbox_inertia;
    	compensation_torques.tau_static_friction = tau_static_friction; % scalar

    end % 



    function  [sol, bad_bin_idxs, directions, tau_error] = parse_solution(obj, y_sol, index_map,...
                                                comp_torques,  motor, gearbox)
    %
    %   Inputs:
    %
    %
    %   Outputs:
    %       bad_bin_idxs used to index into delta so does not correspond
    %       directky to indexes into omega (only for points where decomp may go bad )
    %
    %
        bad_bin_idxs = []; % initialize empty 
        directions = [];  % initialize empty 
        tau_error = []; 

        if isnan(y_sol(1))   % TODO -- handle non-solutions cleaner 
            % Return nans 
            sol = struct(); % empty 
            return; 
        end 
        
        omega = obj.problem_data.omega;

        % If Actual solution exists....
        k_t = motor.k_t; 
        ratio = gearbox.direction .* gearbox.ratio; 

        eta = gearbox.efficiency; 

        I_comp = comp_torques.I_comp; 

        sol.I = y_sol(index_map.I); 
        sol.x = y_sol(index_map.x); 

        sol.I_comp = I_comp; 

        I_shift = sol.I - I_comp; 


        % TODO -- remove some unnuessary ronse 
        sol.I_shift = I_shift;
        sol.Isq = y_sol(index_map.Isq);
         
        zero_vel = [sign(omega) == 0];
        static_motor_friction = zeros(obj.problem_data.n, 1); 
        static_motor_friction(obj.problem_data.zero_vel_idxs) = y_sol(index_map.sf);

        tau_motor_friction = comp_torques.tau_motor_friction;
        tau_motor_inertia = comp_torques.tau_motor_inertia;
        tau_gearbox_inertia = comp_torques.tau_gearbox_inertia;

        % sol.tau -- torque felt at the joint (afte accounting for torque to acell gearbox )
        if eta < 1
            sol.I_gm = y_sol(index_map.gm);
            sol.I_mu = y_sol(index_map.mu);
            sol.tau = ratio*k_t*eta*(sol.I_gm) + (ratio*k_t/eta)*(sol.I_mu)...
                     + ratio*static_motor_friction - tau_gearbox_inertia;
        else 
            sol.tau = ratio*k_t*(I_shift) ...
                    + ratio*static_motor_friction - tau_gearbox_inertia;
        end 

        Isq = y_sol(index_map.Isq);


        % calculate compensation torque 
        % TODO -- make sure to account for zero vel frictoi cone in outputs 
        
        % For debug 
        tau_comp = sol.tau + tau_gearbox_inertia;  % tau delivered to + torque eateb up by gb inetita 
       
        % in this context, strictly greater 
        driving = [tau_comp .* sign(omega) > 0];  % note OR equals 
        driven = [tau_comp .* sign(omega) < 0];

        
        % TODO -- rename zero vel things to make it more clear its torque 
        % ADD THESE
        sol.tau_friction = -ratio*(tau_motor_friction.*(eta*driving + ...
                            driven/eta) -  static_motor_friction);
        sol.tau_inertia = -ratio*tau_motor_inertia.*(eta*driving + ...
                        driven/eta + zero_vel) - tau_gearbox_inertia;
        tau_m = k_t * sol.I; % from current 

        % Just electromechanical 
        sol.tau_em = ratio*(tau_m.*(eta*driving + driven/eta + zero_vel)); 

        sol.driving = driving; 

        % TODO Add debug to settings 
        

        % omega.*compensation torque goes negative (need to go through proof)

      
        gmmu_tol = 1e-3; 
        gm = y_sol(index_map.gm); 
        mu = y_sol(index_map.mu);

        min_gmmu_all = min(abs(gm), abs(mu));


        if isfield(index_map, 'binary')


            min_gmmu_sanity = min_gmmu_all;
            min_gmmu_sanity(index_map.binary) = []; 

            if max(min_gmmu_sanity) > 1e-3 % once get below rho -- these has very small impact 
                disp('concering???') 
                disp(max(min_gmmu_sanity))
                
                % unclear how this happens -- and may need to adjust rho?    

                %keyboard
            end 

            % Only pick out the indices we care about 
            gm_bin = gm(index_map.binary);
            mu_bin = mu(index_map.binary); 
            min_gmmu = min_gmmu_all(index_map.binary);
            init_directions = driving(index_map.binary); 

            sol.del = y_sol(index_map.del);


            % Only really concerned with where decomp may be bad (aug_idxs)
            
            gmmu_bad_idxs = find(min_gmmu > gmmu_tol); 
            bad_vals = min_gmmu(gmmu_bad_idxs);
            bad_bin_idxs = gmmu_bad_idxs; 
            
            % ... only works with the added Rotated SOC Perspective Constraints 
            %{
            del_error = abs(sol.del - round(sol.del));
            bad_bin_idxs = find(del_error > 1e-3);
            bad_vals = del_error(bad_bin_idxs);

            %} 

            % other option would be to sort by expected influence on the problem itself 


            [~, reindex] = sort(bad_vals, 'descend'); 



            bad_bin_idxs = bad_bin_idxs(reindex); % now sorted 
            directions = init_directions(reindex); 

            tau_check = sol.tau - (sol.tau_friction + sol.tau_inertia);
            tau_error = abs(tau_check - sol.tau_em); 

            % index out to return 
            tau_error = tau_error(index_map.binary(bad_bin_idxs));
            tau_error = tau_error(reindex);

        end 
        

        %directions = driving(bad_idxs); 

        % TODO -- compare with bad idxs from tau check


        %{
        tau_error = tau_check - sol.tau_em;     
        tau_tol = 1e-3; 
        tau_bad_idxs = find(abs(tau_error) > tau_tol);

        if isempty(bad_idxs) && ~isempty(tau_bad_idxs)
            % ok, well with huge gear ratios sure 
            disp('curious,,,,')
            %keyboard
        end 


        if obj.settings.debug 
            % TODO -- add more feautures to solution
        end 
        %} 

        %{
        alt_dirs = (sign(I_shift(bad_idxs) .* obj.omega(bad_idxs)) + 1)/2;
        tau_m_sol = motor.k_t*((eta*gm) + (mu/eta)); 


        if ~isempty(directions) 
            if any(directions - alt_dirs)
                disp('diff diers')
                keyboard
            end 
        end 
        %}

        % If we were to keep 



    end 

    % this needs matrices but parse normally wouldnt -- could call this after parse 
    % if there is a problem
    function [new_sol, new_cost] = clean_solution(obj, y_sol, init_cost, index_map,...
                                                     matrices, comp_torques, motor, gearbox)

           

        dim_y = length(y_sol);
        eta = gearbox.efficiency;
        ratio = gearbox.direction .* gearbox.ratio;
        k_t = motor.k_t; 
        I_comp = comp_torques.I_comp;
        omega = obj.problem_data.omega; 

        gmmu_tol = 1e-3;    %  TODO -- link up with settings 

        % find initial bad indices
        I_init = y_sol(index_map.I);
        gm_init = y_sol(index_map.gm);
        mu_init = y_sol(index_map.mu); 
        min_gmmu = min(abs(gm_init), abs(mu_init));
        bad_idxs = find(min_gmmu > gmmu_tol);        % absolute indexing 



        % We will fix the output torques to what they were 
        tmp_sol = nan(dim_y, 1); 

        tau = y_sol(index_map.tau);   % tau
        tau_comp = tau + comp_torques.tau_gearbox_inertia; 
        % determine corresponding mu/gamma 
        % NOTE: neither driving NOR driven if velocity is exactly zero
        driving = [tau_comp .* sign(omega) > 0]; 
        driven = [tau_comp .* sign(omega) < 0]; 
        moving = [sign(omega) ~= 0];
        still = not(moving); % not moving 

    
        gm = (tau_comp .* driving)/(ratio*eta*k_t);
        mu = (tau_comp .* driven * eta)/(ratio*k_t);
        I = gm + mu + comp_torques.I_comp; 
        Isq = I.^2;
        s = (gm - mu).^2; 
        gmsq = gm(index_map.binary).^2;
        musq = mu(index_map.binary).^2;
        del = driving(index_map.binary); 
        sig = 1 - del; 


        % TODO 
        %% 
        tmp_sol(index_map.I(moving)) = I(moving);
        tmp_sol(index_map.gm(moving)) = gm(moving);

        tmp_sol(index_map.tau(moving)) = tau(moving); 

        tmp_sol(index_map.s) = s; 
        tmp_sol(index_map.del) = del;

        tmp_sol(index_map.gmsq) = gmsq;

        % removing redundancies helps numerical issues 
        %tmp_sol(index_map.mu) = mu; 
        %tmp_sol(index_map.sig) = sig;
        %tmp_sol(index_map.Isq) = Isq;
        %tmp_sol(index_map.musq) = musq; 


        not_nan_idxs = find(~isnan(tmp_sol)); 

        % This feasTol fudge factor is because otherwise we have redundant 
        % contraints which in a perfect world would be deteced but it
        % can overconstrain the system depending on the solver settings
        % and the solver used 
        
        % now -- how to determine the rest of the x vector? 
        % with all these vars already fixed may be able to do as lp solve 
        % in the context of debuggin the problem thats annoying you right now,
        % could remove quadcons -- althou not in general true if therse 
        % additional use defined things 
        %clean_matrices.quadcon = struct([]);

        feasTol = 1e-5;   % TODO -- link up with solver settings 1e-6 works without static friction

        % Want to fix all of the values we decided on above 

        % Check for direct upper bound issues 

        % Need a feas tol here because solvers will sometimes slightly violate
        % the constraints 
        ub_bad_idxs = find(tmp_sol > matrices.ub + feasTol);  
        lb_bad_idxs = find(tmp_sol < matrices.lb - feasTol); 

        clean_matrices = matrices; 

        if any(ub_bad_idxs) || any(lb_bad_idxs)
 
  
            % alter this so 
            for idx = [ub_bad_idxs', lb_bad_idxs'] 
                display(idx)
                if (idx <= index_map.I(end)) && (idx >= index_map.I(1))
                    % issue is with bound on currents 
                    if gm(idx) == 0
                        clean_matrices.lb(index_map.gm(idx)) = 0;
                        clean_matrices.ub(index_map.gm(idx)) = 0;
                    else % mu(idx) == 0
                        clean_matrices.lb(index_map.mu(idx)) = 0;
                        clean_matrices.ub(index_map.mu(idx)) = 0;
                    end 
                elseif (idx <= index_map.tau(end)) && (idx >= index_map.tau(1))
                    % issure with torques -- since the torques have been 
                    % fixed in the above process 
                    % this indicates a tolerancing error 
                else 
                    disp('aaaa ---------------------- ')
                    keyboard
                end 
            end 

            % now should be cleanable without violating bounds however 
            % may create new bad idxs 

            % do a recursive clean you made man -- make sure to set base
            % case or recusrion limtis 
            [new_sol, new_cost, result] = obj.socp_solve(clean_matrices, inf);

            disp('about to make recurs')


            % NOTE: making the recursive call with initial matrices
            [y_rec, rec_cost] = obj.clean_solution(new_sol, init_cost, index_map,...
                                                 matrices, comp_torques, motor, gearbox); 

            [sol_rec, bad_idxs_rec] = obj.parse_solution(y_rec, index_map,...
                                                    comp_torques,  motor, gearbox);


            disp('popped out of recusive call')

            if isempty(bad_idxs_rec)
                new_sol = y_rec; 
                new_cost = rec_cost; 
            else % wasnt able to figure out - return originals  
                 disp('sad day for recursive functions everywhere')
                new_sol = y_sol;
                new_cost =  init_cost;
            end 
        else 
            %ff = 1e-4; % fudge factor on specified bounds to help with numerics
            ff = 1e-3; % fudge factor on specified bounds to help with numerics
            % Anything that was fixed to zero we want to keep numerically zero
            % for the rest we will loosen the bounds a little bit to help with numerical issues
            tmp_lb = max(matrices.lb, tmp_sol -  [tmp_sol ~= 0]*ff);
            tmp_ub = min(matrices.ub, tmp_sol +  [tmp_sol ~= 0]*ff);


            clean_matrices.lb(not_nan_idxs) = tmp_lb(not_nan_idxs);
            clean_matrices.ub(not_nan_idxs) = tmp_ub(not_nan_idxs);

            [new_sol, new_cost, result, model] = obj.socp_solve(clean_matrices, inf);

            % TODO consider parsing here
            if isnan(new_sol(1))
                disp('ok so this fauled ')
                keyboard
                iis = gurobi_iis(model)
                keyboard 
            end 
        end  
    end 

    function [tau_motor_friction, tau_static_friction] = ...
                                            friction_model(obj, motor, gearbox)
    %
    %   TODO - thorough explanation
    %
    %
    %
    %
        % TODO -- incorporate 'settings' or whatever
        
        % TODO
        % TODO      How do we handle missing inputs?????????
        % TODO
        % TODO 
        omega = obj.problem_data.omega;
        sign_omega = sign(omega); 


        if strcmp(motor.type, 'DC')

            if ~isnan(motor.coulomb_friction)
                tau_static_friction = motor.coulomb_friction;
                coulomb_friction = sign_omega*motor.coulomb_friction;
            else 
                keyboard
                %tau_static_friction = 
                %coulomb_friction = 
            end 

            if isnan(motor.viscous_friction)
                viscous_friction = 0;
            else 
                viscous_friction = omega*motor.viscous_friction;
            end 
        elseif strcmp(motor.type, 'BLDC')
            if ~isnan(motor.coulomb_friction)
                tau_static_friction = motor.coulomb_friction; 
                coulomb_friction = sign_omega*motor.coulomb_friction;
            else
                tau_static_friction = 0; 
                coulomb_friction = 0;
            end 

            if ~isnan(motor.viscous_friction)
                viscous_friction = omega*motor.viscous_friction;
            else 
                % estimate from no-load tests
                if isnan(motor.I_nl)
                    I_nl = (motor.V - motor.k_t*motor.omega_nl)/motor.R; 
                else
                    I_nl = motor.I_nl;
                end  
                    % compute what it woud be 
                drag_torque = motor.k_t*I_nl;
                c_v = drag_torque/motor.omega_nl;
                viscous_friction = c_v*omega;  
            end 
        else 
            error('friction_model: Invalid motor type');
        end 
            
        % combine coulomb and viscous 
        tau_motor_friction = coulomb_friction + viscous_friction; 
    end 


    function [y_sol, combo_cost, result, model] = socp_solve(obj, matrices, cutoff)
    %
    %
    %   TODO -- this all needs cleaning -- might want to add a seperate 
    %   'gurobi solve' file for consistency
    %
        model = [];

        Q_cost = matrices.Q_cost;
        c_cost = matrices.c_cost;
        c_rho_aug = matrices.c_rho_aug;
        beta0 = matrices.beta0;
        A_eq = matrices.A_eq;
        b_eq = matrices.b_eq;
        A_ineq = matrices.A_ineq; 
        b_ineq = matrices.b_ineq; 
        lb = matrices.lb; 
        ub = matrices.ub; 
        quadcon = matrices.quadcon;
        dim_y = length(lb);


        if isfield(matrices, 'binary') && nnz(matrices.binary) > 0
            mixed_integer = true;
        else 
            mixed_integer = false; 
        end 

        if strcmpi(obj.settings.solver, 'gurobi')  || mixed_integer

            %% Gurobi Model 
        
            sense = [repmat('=', length(b_eq), 1); repmat('<', length(b_ineq), 1)]; 
            A_all = sparse([A_eq; A_ineq]); 
            b_all = [b_eq; b_ineq]; 

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
        elseif strcmpi(obj.settings.solver, 'ecos') || ...
                            strcmpi(obj.settings.solver, 'sedumi')

            [c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc] = prep_socp(Q_cost,...
                        c_cost, beta0, A_eq, b_eq, A_ineq, b_ineq, quadcon,...
                                         lb, ub,  cutoff);
            if strcmpi(obj.settings.solver, 'ecos')
                result = ecos_solve(c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc);
            else
                result = sedumi_solve(c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc); 
            end 

            % ...maybe copy outside if for reuse? 
            if ~isinf(result.objval)
                y_sol = result.x; 
                combo_cost = result.objval; 
            else
                y_sol = nan(dim_y, 1); 
                combo_cost = inf;  
            end 
        else
            error('invalid solver'); % shouldnt be possible 
        end 

    end % socp_solve


    function vprintf(obj, priority, varargin)
    %
    %
    %
    %    
        if priority >= obj.settings.verbose
            fprintf(varargin{:}); 
        end 
    end 




end % end private methods 




%==============================================================================
end % end CLASSDEF

%%%% Non Class Methods 


function settings = validate_settings(init_settings)
    %
    %   Takes in settings which may be missing fields and returns 
    %   valid settings or errors out if settings were bad 
    %
    settings = default_settings(); 

    % Loop through the fields
    fn = fieldnames(init_settings);

    for ii = 1:length(fn)
        field = fn{ii};
        if ~isfield(settings, field)
            error('validate_settings: invalid field %s', field);
        end 
    end 

    if isfield(init_settings, 'verbose')
        settings.verbose = init_settings.verbose;         
    end
    if isfield(init_settings, 'print_freq')
        settings.print_freq = init_settings.print_freq; 
    end

    valid_solvers = {'gurobi', 'ecos', 'sedumi'};
    has_gurobi = ~isempty(which('gurobi.m'));
    has_ecos = ~isempty(which('ecos.m'));
    has_sedumi = ~isempty(which('sedumi.m'));

    if ~any([has_gurobi, has_ecos, has_sedumi])
        error('validate_settings: no valid solvers found in path');
    end 

    if isfield(init_settings, 'solver')
        settings.solver = init_settings.solver; 
    end

    if strcmp(settings.solver, 'gurobi')
        if ~has_gurobi
            if has_ecos; settings.solver = 'ecos'; else settings.solver = 'sedumi'; end 
            warning('Gurobi solver not found, using %s', settings.solver);                
        end 
    end 
    if strcmp(settings.solver, 'ecos')
        if ~has_ecos
            if has_gurobi; settings.solver = 'gurobi'; else settings.solver = 'sedumi'; end 
            warning('ECOS solver not found, using %s', settings.solver);
        end 
    end 
    if strcmp(settings.solver, 'sedumi')
        if ~has_sedumi
            if has_gurobi; settings.solver = 'gurobi'; else settings.solver = 'ecos'; end 
            warning('Sedumi solver not found using: %s', settings.solver);
        end 
    end 
end 

function solve_settings = validate_solve_settings(init_solve_settings)

    iss = init_solve_settings; % just a shorter name 
    nss = default_solve_settings(); % new solve settings 
    % TODO -- add validation on ranges 
    if isfield(iss, 'rho'); nss.rho = iss.rho; end 
    if isfield(iss, 'gmmu_tol'); nss.gmmu_tol = iss.gmmu_tol; end 
    if isfield(iss, 'feastol'); nss.feastol = iss.feastol; end 
    if isfield(iss, 'reltol'); nss.reltol = iss.reltol; end 
    if isfield(iss, 'abstol'); nss.abstol = iss.abstol; end 
    if isfield(iss, 'qcvx_reltol'); nss.qcvx_reltol = iss.qcvx_reltol; end 
    if isfield(iss, 'qcvx_abstol'); nss.qcvx_abstol = iss.qcvx_abstol; end
    solve_settings = nss; % new solve settings 
end 

function def_settings = default_settings()
    def_settings = struct('solver', 'gurobi',...
                           'verbose', 1,...
                           'print_freq', 500);

 
end 

function def_solve_settings = default_solve_settings()
    def_solve_settings = struct('rho', 1e-4,...
                                'gmmu_tol', 1e-3, ... % 1 ma 
                                'feastol', 1e-5,...
                                'reltol', 1e-3,... % 0.1 percent optimality
                                'abstol', 1e-7,...  % TODO hmm, compare with ecos 
                                'qcvx_reltol', 1e-3,...
                                'qcvx_abstol', 1e-7);

end 


function [test_motor, test_gearbox] = test_motor_gearbox()

    test_motor = struct('key', 'none',...
                        'manufacturer', 'none',...
                        'ID', 'none',...
                        'type', 'DC',...        % TODO -- double check if this can matter
                        'V', 1,...
                        'k_t', 1,...
                        'R', 0.1,...
                        'L', 1e-3,...
                        'mass', 1,...
                        'inertia', 1e-6,...
                        'omega_nl', 1000,...
                        'I_nl', 0.1,...
                        'max_int_torque', inf,...
                        'max_int_speed', inf,...
                        'max_cont_speed', inf,...
                        'max_cont_power', inf,...
                        'coulomb_friction', 1e-3,...
                        'viscous_friction', 1e-4,...
                        'Rth1', 1,...
                        'Rth2', 1);

    test_gearbox = struct('key', 'none',...
                          'manufacturer', 'none',...
                            'ID', 'none',...
                            'type', 'planetary',...        % TODO -- double check if this can matter
                            'stages', 1,...
                            'ratio', 2,...
                            'mass', 0.1,...
                            'inertia', 1e-6,...
                            'efficiency', 0.9,...
                            'direction', 1,...
                            'max_int_torque', inf,...
                            'max_cont_torque', inf); 
end 

% TODO -- remove this or come up with better ptinting system (probs juist always scinot)
function str = sciprint(num, specifier)
    fstr = sprintf(['%',specifier,'f'], num);
    estr = sprintf(['%',specifier,'e'], num);
    if length(fstr) <= length(estr) && ((num > 1e-2) || num == 0)
        str = fstr;
    else 
        str = estr; 
    end 
end 
    %{
     function ranked_combos = apply_rankings(obj, combos)
    %
    %
    %
    
        % if no rankings, just return
        if nnz(obj.ranking_matrix) == 0
            ranked_combos = combos;
            return;
        end 

        ranked_combos = zeros(size(combos) + [0, 1]);  % extra col 
        num_combos = size(combos, 1);
        % O(n) -- get all the rankings  
        for j = 1:num_combos
            motor_idx = combos(j, 1);
            gear_idx = combos(j, 2);  
            ranking = obj.ranking_matrix(motor_idx, gear_idx); 
            ranked_combos(j, :) = [ranking, motor_idx, gear_idx]; 
        end 
        % O(nlogn) -- sort by the rankings 
        [~, sort_idxs] = sort(ranked_combos(:, 1)); 
        ranked_combos = ranked_combos(sort_idxs, :); 
    end 



    function [combo_list] = get_combinations(obj)
    %**********************************************************************
    % Method: get_combinations 
    % 
    % Inputs: 
    %       None
    %
    % Outputs:
    %       N x 2 array where N is the number of combinatioms. First col 
    %       gives index of motor in motor table
    %       and second col gives the index of the gearbox. For direct 
    %       drive (no gearbox) a 0 is used in the second column as a flag
    %       for no gearbox 
    %
    %   Added:   
    %**********************************************************************    

        % NOTE if there are filter criteria like motor velocity 
        % that needs to be included here because function of both 
        % motor max speed AND gear ratio

        %if obj.settings.verbose; fprintf('\nGetting combinations...'); end 


        % Loop through motors (NOTE: list will typically be filtered before)
        num_motors = size(obj.motors, 1);
        combo_list = {}; % start as cell because dont know how big it needs to be 
        for i = 1:num_motors 
            motor_idx = i; % For now --- later -- use filtered idxs 
            % indices in gear table 
            compatible_gears = cell2mat(obj.motors.('gearboxes')(motor_idx)); 
            for j = 1:numel(compatible_gears)
                % TODO: if 'filtered' this combo may be excluded 
                gear_idx = compatible_gears(j); 
                combo_list{end + 1, 1} = [motor_idx, gear_idx]; 
            end 
        end 
        % convert cell to N x 2 array 
        combo_list = cell2mat(combo_list); 

        %combo_list = combo_list(1:min(length(combo_list), 400), :); 
        %warning('remember to comment this debug thing out limiting combos to 200')
    

    end 
    %} 


    %{
    function new_combos = velocity_filter(obj, combos)
    %
    %
    %
        motors = table2struct(obj.motors);
        gears = table2struct(obj.gearboxes);

        num_init_combos = size(combos, 1); 
        obj.vprintf(1, 'Found %d initial feasible combinations\n', num_init_combos);

        % Starting at end of combo list and remove based on velocty 
        max_output_vel = max(abs(obj.omega)); 

        for j = length(combos):-1:1
            motor_idx = combos(j, 1); 
            gear_idx = combos(j, 2); 

            max_motor_vel = max_output_vel * gears(gear_idx).ratio;  
            if max_motor_vel > motors(motor_idx).omega_max
                combos(j, :) = []; % clear it out 
            end 
        end 
        new_combos = combos; 


        num_updated_combos = size(combos, 1); 
        obj.vprintf(1, 'Filtering by max velocity removed: %d combinations\n',...
                            num_init_combos - num_updated_combos);
    end 
    %} 



    %{
    function update_rankings(obj, cost_list, combos)
    %
    %
    %

        % May be called by user so add validation
        cost_list = cost_list(:); 

        assert(size(combos, 1) == length(cost_list),...
                 'Number of combinations must match length of costs');
        assert(size(combos, 2) == 2, 'Combinations must be 2xN matrix'); 

        [~, rank_idxs] = sort(cost_list);   % Need to index this back in 
        tmp_table = [cost_list, combos];
        obj.rankings = tmp_table(rank_idxs, :); 

        % row - motor idx, col - gear idx, val = ranking - 0 is flag for inf 
        ranking_matrix = sparse(combos(:, 1), combos(:, 2), cost_list,...
                                     obj.num_motors, obj.num_gearboxes); 
        obj.ranking_matrix = ranking_matrix; 
    end 
    %} 