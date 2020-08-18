classdef MotorSelection < matlab.mixin.Copyable 
% MotorSelection    Summary goes here  
%
%    This needs to work well with 'help'
%
%
%   MotorSelection Methods:
%       update_problem - 
%       optimize - 
%
%   MotorSelecton Properties: 
%       settings - 
%       rankings - 3 x N array. Cols - [expected_cost, motor_idx, gear_idx]
%                   Updated whenever opitimization is run. Motor/gear 
%                   indices refer to rows in the motor and gear table 
%
%
%
%   Blah. blah. blah 
%
%   A reference to the github
%
%   A reference to the eventual paper 
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 7/27/20
%       Stanford University, Biomechatronics Lab 
%
properties (GetAccess = public, SetAccess = private)

    
    settings % for solver, rho, verbose
    rankings % for combos - may want to rename 
    ranking_matrix 

    % ----- Problem Data -- May want more dscriptive names?
    omega
    omega_dot 
    % indices in omega corresponding to omega == 0 
    zero_vel_idxs  	% computed once for each inputs to help solve time 
    Q
    c
    M   % can encode SOCP  
    r
    beta % beta 
    H
    H_ineq
    b
    b_ineq
    T
    d 
    I_u
    lin_ineq % 1xm binary indicator matrix. 1 iff Q_j = 0, M_j = 0 

  
    % ----------- Dimensions of matrices/vectors ------------
    n   % length of trajectory/trajectories
    m   % number of gen ineqs 
    w   % dimension of x 
    p   % 

    problem_type   % char array, 'standard' (SOCP) or 'fractional'

    %---- Motor and Gear data tables ----------
    motors  
    gearboxes
    test_motor          % for validating input properties 
    test_gearbox        % for validating input properitse 
    num_motors
    num_gearboxes 

    % --- Data specifially for Quasiconvex Problems -----

    % TODO - can generalize to any linear fractional program 
    % with offset at same compuational cost 
    % will need restrucutre inputs 
    % c_num, c_den, beta_num, beta_den 
    cost_ub 
    cost_lb
    % Fractional Objective (r_num'*x + beta_num)/(r_den'*x + beta_den) 
    r_num
    r_den
    beta_num
    beta_den


    % Friction and other model settings? 
end 


methods (Access = public)

    function obj = MotorSelection(varargin) % we will parse the inputs directly 
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

        % NOTE settings as an input or no? 

        % For now -- emptpy 
        % Can call the update problem later if want to instantiate with 
        % problem 

        % Default settings -- NOTE: may want to to be able to instantiate with settings 
        if ~isempty(varargin)
        	settings = varargin{1};  % then validate 
        else 
        	settings = struct(); 
        end 

        % Fill in default settings 
        if ~isfield(settings, 'debug'); settings.debug = true; end 
        assert(islogical(settings.debug), 'debug setting must be true/false');
        if settings.debug; fprintf('Debug is on...\n'); end; 
        if ~isfield(settings, 'rho'); settings.rho = 1e-4; end
        if ~isfield(settings, 'verbose'); settings.verbose = 1; end 
        if ~isfield(settings, 'qcvx_reltol')
        	settings.qcvx_reltol = 1e-3; 
        end 
        if ~isfield(settings, 'qcvx_abstol')
        	settings.qcvx_abstol = 1e-6; 
        end 


        valid_solvers = {'gurobi', 'ecos', 'sedumi'};
        has_gurobi = ~isempty(which('gurobi.m'));
        has_ecos = ~isempty(which('ecos.m'));
        has_sedumi = ~isempty(which('sedumi.m'));

        if ~isfield(settings, 'solver')
        	settings.solver = 'gurobi'; % default, fastest
        end

        if strcmp(settings.solver, 'gurobi')
        	if ~has_gurobi
        		settings.solver = 'ecos';
        		warning('Gurobi solver not found');
        	end 
        end 
        if strcmp(settings.solver, 'ecos')
        	if ~has_ecos
        		settings.solver = 'sedumi';
        		warning('ECOS solver not found');
        	end 
        end 
        if strcmp(settings.solver, 'sedumi')
        	if ~has_sedumi
        		error('No valid solvers found in path');
        	end 
        end 

        obj.settings = settings; 
        % Assign the default motor and gear tables from some directory? 
        % https://www.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html
        % FOR NOW --- hard coding in location but when package
        % this up its fine (see line)
        % NOTE: could have default directory to look for path 
        % and the option to change that path to load in from elswhere 


        % TODO -- make finding these files robust 
        %load('/home/erez/Documents/MotorSelection/database/maxon_motor_gb_data.mat',...
        %            'motor_table', 'gear_table'); 

        load('database/maxon_motor_gb_data.mat',...
                    'motor_table', 'gear_table'); 

        % Load in from database too 
        % also here will need to play around with matlab packaging 
        %load('/home/erez/Documents/MotorSelection/database/test_vals.mat',...
        %            'test_motor', 'test_gearbox'); 
        load('database/test_vals.mat',...
                    'test_motor', 'test_gearbox');
        obj.test_motor = test_motor;
        obj.test_gearbox = test_gearbox; 


        var_names = gear_table.Properties.VariableNames;    % get names 
        var_types = varfun(@class, gear_table, 'OutputFormat', 'cell'); % get var types
        % create a copatible 1 row table for direct drive 
        T_direct_drive = table('Size', [1, numel(var_names)], 'VariableTypes',...
                                         var_types, 'VariableNames', var_names); 

        % all 'double' properties will be initialized to zeros

        % TODO -- Move this direct drive stuff and other database
        % aspects to an external code, will make things easier to manage
        % perhaps its own class??? (copyable) 
        T_direct_drive.eta = 1; 
        T_direct_drive.alpha = 1; 
        T_direct_drive.mass = 0;  T_direct_drive.inertia = 0;  T_direct_drive.Price = 0;
        T_direct_drive.Max_cont_torque = inf;
        T_direct_drive.Max_int_torque = inf;
        gear_table = [T_direct_drive; gear_table]; % augment with direct drive 

        % Increment ALL of the compatible gear indices AND append 1 to them 
        % as all are compatible with direct drive dummy gearbox 
        for j = 1:size(motor_table, 1) 
            % append a 0 and then increment (since it may be empty)
            tmp = cell2mat(motor_table.gearboxes(j));
            new_gear_list = [0; tmp(:)] + 1; 
            motor_table.gearboxes(j) = {new_gear_list}; 
        end 

        obj.motors = motor_table;
        obj.gearboxes = gear_table;
        obj.num_gearboxes = size(gear_table, 1); 
        obj.num_motors = size(motor_table, 1); 


        % TODO - validation on motor/gear set 
  


      
        % INITIALIZE OTHER THINGS?

        %{
        % set the properties 
        if nargin > 0 % allows for zero argument constructor for allocation
            p = inputParser; 
            addRequired(p, 'velocity', valid_vel);
            addParameter(p, 'limits', defaultLims, validLimits); % add checking on limits 
            parse(p, varargin{:});
        end % end if nargin > 0 
        %} 
    end %end constructor 

    


    %**********************************************************************
    % Method: update_problem
    % 
    % Inputs: 

    %
    %   Added:   
    %**********************************************************************
    function update_problem(obj, problem_data, varargin)
        
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
        assert(isfield(problem_data, 'I_u'), 'Missing I_u');


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
        I_u = problem_data.I_u; 

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
        		 		b_ineq = @(motor, gearbox) b_ineq(:);   				  end 
        if ~isa(T, 'function_handle');  T = @(motor, gearbox) T;      end 
        if ~isa(d, 'function_handle');  d = @(motor, gearbox) d(:);   end 
        if ~isa(I_u, 'function_handle'); I_u = @(motor, gearbox) I_u; end 


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

        I_u_test = I_u(test_motor, test_gearbox);
        assert(length(I_u_test) == n, 'I_u incorrect length');
        assert(~isinf(max(I_u_test)), 'I_u must be finite'); 
        assert(min(I_u_test) > 0, 'I_u must be strictly positive'); 

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
        Q_empty = sparse(n, n);
        M_empty = sparse(w, w); 
        c_empty = zeros(n, 1);    
        r_empty = zeros(w, 1);   

        lin_ineq = zeros(m, 1); 

        % Letting c,r,beta be fixed instead of function handles can 
        % greatly speed up code 
        for j = 1:m + 1
            Qj = Q{j}(test_motor, test_gearbox); 
            Mj = M{j}(test_motor, test_gearbox); 

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

            if isempty(Qj); Q{j} = @(~, ~) Q_empty; end 
            if isempty(Mj); M{j} = @(~, ~) M_empty; end 

            assert(issparse(Qj) || isempty(Qj),...
                             'update_problem: Q matrices must be sparse');
            assert(issparse(Mj) || isempty(Mj),...
                             'update_problem: M matrices must be sparse');
            assert(issymmetric(Qj), 'update_problem: Q matrices must be symmetric');
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


            if nnz(Qj) == 0 % empty 
                if (nnz(Mj) == 0) && (j > 1)
                    lin_ineq(j - 1) = 1; 
                end 
            else % check PSD 
                % Index out block matrix
                [row, col, ~] = find(Qj);  
                % find works in order of increasing col. idx 
                % within each col, increasing row idx 
                %unique_col = unique(col); 
                unique_col = find(any(Qj)); % faster 

                Q_small = Qj(:, unique_col); 
                Q_small = full(Q_small(unique_col, :)); 
                eig_vals = eig(Q_small); 
                assert(min(eig_vals) >= 0, 'Each Q must be positive semidefinite');
            end 

            if nnz(Mj) > 0
                [row, col, val] = find(Mj)
                %unique_col = unique(col); 
                unique_col = find(any(Mj));  % faster  

                M_small = Mj(:, unique_col)
                M_small = full(M_small(unique_col, :)); 
                [V, D] = eig(M_small); 

                num_neg_eig = sum([D < 0]); % May need to play with tolerancing 
                if num_neg_eig == 1  

                    assert(nnz(rj) == w, 'Linear term must be empty for SOC constraints'); 

                    % TODO -- what about constant term???? 

                    % ...... if solving with gurobi dont need to do anything 
                    % NOTE: should check that its still valid
                    % one negative eig does not neccesarilt 
                    % Diagonal of D in increasing order 
                    % eigenvector corresponding to negative eigenvalue 
                    v = D(:, 1); 
                    v([abs(v) < 1e-12]) = 0; % tolerancing  
                    
                    if nnz(v) > 2 % Invalid 
                        error('update_problem:invalidInput',...
                        'Error. The eigenvector associate with the',...
                        'smallest eigenvalue of M_%d',...
                        'must have at most 2  non-zeros',...
                        'but has %d', j, nnz(v)); 
                    else        
                        tmp_idxs = find(v)
                        idxs = unique_col(tmp_idxs);
                        lb(x_start_idx + idxs - 1) = 0; % non-negativity if not already there 
                        % Given that its symmetric, not sure if need any more 
                        %{ 
                        if nnz(v) == 2 % Rotated SOC 
                        else  % if nnz(v) = 1 -- standard SOC 
                            % TODO 
                            % add non-negativity constraint on variable
                            % if not already present 

                        end 
                        %} 
                    end 
                elseif num_neg_eig > 1  
                    error('update_problem:invalidInput',...
                        'Error. M_%d must have at most 1 negative',...
                        'eigenvalue, has %d', j, num_neg_eig); 
                end 
            end 
        end 


        %
        %	Validating inputs for linear fractional problems 
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
            obj.problem_type = 'fractional'; 
        else 
            obj.problem_type = 'standard'; 
        end 


        %
        %
        % Once all validated 
        %
        %
        obj.Q = Q;   % etc 
        obj.c = c; 
        obj.M = M; 
        obj.r = r; 
        obj.beta = bet; 
        obj.H = H; 
        obj.H_ineq = H_ineq; 
        obj.b = b; 
        obj.b_ineq = b_ineq; 
        obj.T = T; 
        obj.d = d; 
        obj.omega = omega;
        obj.omega_dot = omega_dot; 
        obj.zero_vel_idxs = find(omega == 0); 
        obj.I_u = I_u; 
        obj.lin_ineq = lin_ineq; 

        % update relevant dimensions 
        obj.n = n;
        obj.w = w;
        obj.m = m;
        obj.p = p; 

        if strcmp(obj.problem_type, 'fractional')
            obj.cost_ub = problem_data.cost_ub;
            obj.cost_lb = problem_data.cost_lb;
            obj.r_num = problem_data.r_num;
            obj.r_den = problem_data.r_den; 
            obj.beta_num = problem_data.beta_num;
            obj.beta_den = problem_data.beta_den; 
        end 

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
    %
    %   The problem type is assumed from the setup (standard or fractional)
    %
    	start_tic = tic; 
        if isempty(varargin) || isempty(varargin{1})
            num_return = 1;
        else 
            num_return = varargin{1};
        end 

        % NOTE these operations might be slow 
        motors = table2struct(obj.motors);
        gearboxes = table2struct(obj.gearboxes);
    
        % Get combinations from filtered list 
        init_combos = obj.get_combinations(); 
        filtered_combos = obj.velocity_filter(init_combos); 
        combos = obj.apply_rankings(filtered_combos); 

    
        if strcmp(obj.problem_type, 'standard')  
            obj.vprintf(1, 'Combination: ');

            num_combinations = size(combos, 1); % TODO - combine with above? 
            cost_list = nan(num_combinations, 1); 
            cutoff = inf; % no valid solutions yet 
            mincost = inf; % NOTE: not same as cutoff if want to hold on to multiple sols 
            best_combo_idx = 0; % no valid solutions yet 

            % runs the outer loop of the optimization 
            disp_txt = []; 
            num_return = min(num_combinations, num_return); % cant return more than num options 

            % cutoff corresponds to max value of solutions being kept 
            % use a priority queue 
            pq = PQ(true); % 

            for j = 1:num_combinations 
                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
                       disp_txt = sprintf('%d of %d, Best Cost: %0.4f, Time: %0.2f',...
                         j, num_combinations, mincost, toc(start_tic));
                obj.vprintf(1, disp_txt);

                motor = motors(combos(j, 1));
                gearbox = gearboxes(combos(j, 2)); 

                [combo_cost, tmp_sol, comp_time] = obj.combo_solve(motor,...
                                             gearbox, cutoff); 

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
            end 

            num_sols = pq.size(); % may not have found num_return feas sols 
            for i = num_sols:-1:1
                [~, sol_struct] = pq.pop(); 
                sol_structs(i) = sol_struct;
            end 

            % Returns cost_list and sol_structs 
        elseif strcmp(obj.problem_type, 'fractional')

            [cost_list, sol_structs] = obj.optimize_fractional(num_return, combos); 
            mincost = cost_list(1); 
        else 
            error('Invalid problem type. Must be "standard" or "fractional" '); 
        end 

        obj.update_rankings(cost_list, combos); 

        if isinf(mincost)
            sol_structs = struct(); % empty 
            warning('No feasible solutions found');
        end 
    end 



    % MOTOR TABLE -- EACH MOTOR HAS A LIST OF COMPATIBLE GB 
    % ADDING TO TABLES WILL REQUIRE UPDATES WHICH IS SLOW 
    % BUT FINE BECAUSE DOESNT NEED TO BE DONE OFTEN 
    % CAN ALSO BATCH ADD OR MAYBE INCLUDE ADDITIONAL
    % CSV FILES? BATCH ADD -- DONT UPDATE THE REFERENCES UNTIL AFTER 
    % ADDING ALL THE THINGS 
    % MAY NEED TO STORE A COMPATIBILTY MATRIX SOMWHERE OR SOMETHING LIKE THAT 

 
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

        %combo_list = combo_list(1:min(length(combo_list), 30), :); 
        %warning('remember to comment this debug thing out limiting combos to 200')
    
        % maybe add number of distinct motors and gearboxes too 
        %if obj.settings.verbose; fprintf('%d total combinations'); end 

    end 
    



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

            max_motor_vel = max_output_vel * gears(gear_idx).alpha;  
            if max_motor_vel > motors(motor_idx).omega_max
                combos(j, :) = []; % clear it out 
            end 
        end 
        new_combos = combos; 


        num_updated_combos = size(combos, 1); 
        obj.vprintf(1, 'Filtering by max velocity removed: %d combinations\n',...
                            num_init_combos - num_updated_combos);
    end 





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

    % NOTE: some plotting methods could be good 
    % Selection Filtering?
    % Private methods? 

    % Function to CLEAR FILTERS on motor/gb -- go back to original 
    % ...this would probably clear any motor/gb that was manually added
    % no great way around this 

end % end public methods 


methods (Access = private)

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



    function [combo_cost, sol, solve_time] = combo_solve(obj, motor, gearbox,...
                                                 bound)


    	start_tic = tic; 		% to get timing 

        % rename cutoff to "bound" and then check problem type 
        if strcmp(obj.problem_type, 'standard')
            cutoff = bound; 
        elseif strcmp(obj.problem_type, 'fractional')
            cutoff = inf; 
            assert(~isinf(bound), 'Bound must be finite for fractional problems');
        else 
            error('Invalid problem type');
        end 

        % How we actually "solve" will depend on solver
        % Gurobi OR Sedumi OR ECOS 
        % 
        % SOCP implicit for Gurobi solve (looks like a stanrd QCQP)
        omega = obj.omega; 
        omega_dot = obj.omega_dot; % may be all zeros 

        eta = gearbox.eta; 
        alph = gearbox.alpha; 
        k_t = motor.k_t; 

        H = obj.H(motor, gearbox);
        b = obj.b(motor, gearbox);
        H_ineq = obj.H_ineq(motor, gearbox);
        b_ineq = obj.b_ineq(motor, gearbox);


        T = obj.T(motor, gearbox);
        tau_gearbox_inertia = gearbox.inertia * (alph^2) * omega_dot; 
        d = obj.d(motor, gearbox); 
        d_comp = d + tau_gearbox_inertia; % NOTE: gb addition
        
        I_u = obj.I_u(motor, gearbox);  % symmetric bounds, dont need I_l

        % dimensions 
        n = obj.n; 
        w = obj.w;
        p = obj.p;
        m = obj.m; 

        zero_vel_idxs = obj.zero_vel_idxs; 
        nzvi = length(zero_vel_idxs); 

        % NOTE the distinction at omega = 0 (also, could precomp)
        sign_omega = sign(omega); 		% = 0 if omega = 0 
        so = round(sign_omega + 0.1); 	% = 1 if omega = 0  

        %
        % 		Friction and Inertia 
        %

        % TODO -- whats the best general way to handle static friction
        % Should there be a call to a function which determines 
        % friction model based of settings? 
        tau_friction_static = 1.1 * k_t * motor.I_nl;  

        % tau_motor_friction is JUST kinetic 
        tau_motor_friction = k_t * motor.I_nl * sign_omega; % NOTE: Incorp other friction models 
        

        % Since BEFORE gearbox only multiplied by alpha NOT alpha^2 
        % will effectively get multiplied by alpha again when it goes 
        % throug the gearbox 
        tau_motor_inertia = motor.inertia * alph * omega_dot; 
        
  		% Dynamic Motor Friction and Inertia Compensation  (f - fric, j- inerits )
        tau_mfj = tau_motor_friction + tau_motor_inertia; 
        I_comp = tau_mfj/k_t; % corresponding current 


        % Index Map for Optimization 
        I_start_idx = 1;
        I_end_idx = n; 

        % For code reuse, keep +/- decomp even if eta == 1
        Ip_start_idx = n + 1; 
        Ip_end_idx = 2*n; 

        In_start_idx = 2*n + 1;
        In_end_idx = 3*n; 

        if eta < 1 
            dim_y = 5*n + w + nzvi;  % total dimension of opt vector 
        else 
            dim_y = 3*n + w + nzvi; 
        end 

        lb = -inf(dim_y, 1); 
        ub = inf(dim_y, 1); 

        %  Ip - In = I 
        A_eq_tau1 = zeros(n, dim_y);    % sparse? 
        A_eq_tau1(:, Ip_start_idx:Ip_end_idx) = eye(n);
        A_eq_tau1(:, In_start_idx:In_end_idx) = -eye(n);
        A_eq_tau1(:, I_start_idx:I_end_idx) = -eye(n);
        b_eq_tau1 = zeros(n, 1); 



        % Remove after fix 
        A_eq_tau3 = []; 
        b_eq_tau3 = []; 

   

        if eta < 1 
            gm_start_idx = 3*n + 1;
            gm_end_idx = 4*n;

            mu_start_idx = 4*n + 1; 
            mu_end_idx = 5*n; 

            x_start_idx = 5*n + 1; 
            x_end_idx = x_start_idx + w - 1; 

            % Equality Constraints on these variables
            % mu + gamma - sign(omega)*I_nl = I 
            A_eq_tau2 = zeros(n, dim_y);   % sparse? 
            A_eq_tau2(:, gm_start_idx:gm_end_idx) = eye(n);
            A_eq_tau2(:, mu_start_idx:mu_end_idx) = eye(n);
            % TODO -- remove current from the formulation 
            A_eq_tau2(:, I_start_idx:I_end_idx) = -eye(n); %------------------------
			%A_eq_tau2(:, Ip_start_idx:Ip_end_idx) = -eye(n); %++++++++++++++++++++++
			%A_eq_tau2(:, In_start_idx:In_end_idx) = eye(n);  %+++++++++++++++++++++
            b_eq_tau2 = I_comp; 

            % Tx + d = motor/gearbox torque 
            A_eq_tau_x = zeros(n, dim_y);    % sparse? 
            A_eq_tau_x(:, x_start_idx:x_end_idx) = -T; 
            A_eq_tau_x(:, gm_start_idx:gm_end_idx) = alph*eta*k_t*eye(n);
            A_eq_tau_x(:, mu_start_idx:mu_end_idx) = (alph*k_t/eta)*eye(n);
            %b_eq_tau_x = d + alph*k_t*(eta + 1/eta)*so.*I_nl; 
            b_eq_tau_x = d_comp + alph*(eta + 1/eta)*tau_mfj; 



            % Ineqaulity constraints containting multiple variables 
            % (if one variable, more computationally efficient to handle with bound
            % constraints )

            % sign(omega)*(gamma- mu) - I_nl - Ip - In <= 0 
            % sign(omega)*(gamma - mu) - abs(tau_fric/kt) - tau_inertia? 
            A_ineq_tau = zeros(n, dim_y);  
            A_ineq_tau(:, gm_start_idx:gm_end_idx) = diag(so);
            A_ineq_tau(:, mu_start_idx:mu_end_idx) = -diag(so);
            A_ineq_tau(:, Ip_start_idx:Ip_end_idx) = -eye(n);
            A_ineq_tau(:, In_start_idx:In_end_idx) = -eye(n);  
            %b_ineq_tau = so.*I_comp; %----------------------------------------
            
            %b_ineq_tau = motor.I_nl - so.*(tau_motor_inertia/k_t); 
            %b_ineq_tau = -(I_comp);

            
            b_ineq_tau = 100 * ones(n, 1); % removes thus 
            %
            %b_ineq_tau = zeros(n, 1);
            for i = 1:n 

            	if (so(i) == 1)
            		if I_comp(i) >= 0  % standard friction case 
            			b_ineq_tau(i) = I_comp(i);  % CORRECT (STANDARD )
            		else 
            			%b_ineq_tau(i) = eta * I_comp(i); % I think????


            			% Not wrong but not helpful enough?
            			b_ineq_tau(i) = -I_comp(i);

            			% ending up with gm = 0 
            			% and mu = -In + I_comp 
            			% want -- mu = -In, gm = I_comp 


            			% IS IT POSSIBLE THAT WE ARE GETTING THESE 
            			% ERRORS BUT THE SOLUTOIN IS EQUIVALENT?? 

            			% HOW TO CHECK THIS IS WITH A CLEAN UP 

            			% CLEAN THE SOLUTION AND CHECK HOW MUCH 
            			% THE COST IS ALTERED 
            			% THEN WE ARE TALKING ABOUT AN OPTIMAL POINT 
            			% VS THE OPTIMAL POINT

            			% ASSUME CURRENT AND WORK BACKWARD FROM THERE 
            			% OK. TRY THIS NEXT
            			%{
            			if isempty(A_eq_tau3); disp('fffff'); end 

            			Aug3 = zeros(1, dim_y);
            			Aug3(1, gm_start_idx + i - 1) = 1;
            			Aug3(1, mu_start_idx + i - 1) = -1;
            			Aug3(1, Ip_start_idx + i - 1) = -1;
            			Aug3(1, In_start_idx + i - 1) = -1;

            			A_eq_tau3 = [A_eq_tau3; Aug3];
            			b_eq_tau3 = [b_eq_tau3; -I_comp(i)];
            			%}

            			%b_ineq_tau(i) = 0; 
            		end 
            	else
            		if I_comp(i) <= 0 % standard frictoin case
            			b_ineq_tau(i) = -I_comp(i);    % CORRECT (STANDARD)
            		else 
            			b_ineq_tau(i) = - (1/eta) * I_comp(i); % HELP ME 
            		end 
            	end 

            end 
            

            % FILL IN 

                 % Experimental 
      		  %Tx + d = outpiut torque <= kt*alpha*min{eta(I - Icomp), (I - Icomp)/eta}
        	
        	% gets us further without error -- maybe some merit -- think on it more 

        	% WE ARE HITTING ERRORS AT THE PEAK ACCCELERATOINS JUST FYI 
        	%
        	%
        	%
        	A_ineq_tau2 = []; 
        	b_ineq_tau2 = [];

            A_ineq_tau2 = zeros(2*n, dim_y);
            A_ineq_tau2(:, x_start_idx:x_end_idx) = [T; T]; 
            A_ineq_tau2(1:n, I_start_idx:I_end_idx) = -k_t*alph*eta*eye(n);
            A_ineq_tau2(n+1:end, I_start_idx:I_end_idx) = -k_t*(alph/eta)*eye(n);

            b_ineq_tau2 = [-d_comp - k_t*alph*eta*I_comp;...
            				 -d_comp - k_t*(alph/eta)*I_comp]; 

            A_ineq_tau = [A_ineq_tau; A_ineq_tau2];
            b_ineq_tau = [b_ineq_tau; b_ineq_tau2]; 


            % Bounds on individual variables (ie. box constraints)
            % TODO -- what happens with infeasibly large accelerations 
            % where I_comp is very big??? (and causes to exceed limits)
            % If sign(omega) = 1, lower bound on gm is I_nl
            % and upper bound I_u + I_nl 
            % If sign(omega) = -1, upper bound on gm is -I_nl 
            % lower bound is -I_u - I_nl 


            %I_nl = motor.I_nl; 
            % lb(gm_start_idx:gm_end_idx) = min(so.*I_nl,  so.*(I_nl + I_u));
            %ub(gm_start_idx:gm_end_idx) = max(so.*I_nl,  so.*(I_nl + I_u));

			lb(gm_start_idx:gm_end_idx) = min(I_comp,  I_comp + (so.*I_u));
            ub(gm_start_idx:gm_end_idx) = max(I_comp,  I_comp + (so.*I_u));


            % If sign(omega) = 1, upper bound on mu is I_nl
            % and lower bound -I_u + I_nl 
            % If sign(omega) = -1, lower bound on mu is -I_nl 
            % upper bound is I_u - I_nl
            % We force mu = I_comp when omega = 0 (hence sign_omega instead of so)      
			lb(mu_start_idx:mu_end_idx) = min(I_comp,...
										  I_comp - (sign_omega.*I_u));
            ub(mu_start_idx:mu_end_idx) = max(I_comp,...
            							  I_comp - (sign_omega.*I_u));


          	%lb(mu_start_idx:mu_end_idx) = min(so.*I_nl,  so.*(I_nl - I_u));
            %ub(mu_start_idx:mu_end_idx) = max(so.*I_nl,  so.*(I_nl - I_u));

        else   % Direct drive or fully efficient gearbox 
            x_start_idx = 3*n + 1; 
            x_end_idx = x_start_idx + w - 1; 

           
            A_eq_tau_x = zeros(n, dim_y);    % sparse? 
            A_eq_tau_x(:, x_start_idx:x_end_idx) = -T; 
            A_eq_tau_x(:, I_start_idx:I_end_idx) = alph*k_t*eye(n); %--------------------
            %A_eq_tau_x(:, Ip_start_idx:Ip_end_idx) = alph*k_t*eye(n); %++++++++++++++++++
            %A_eq_tau_x(:, In_start_idx:In_end_idx) = -alph*k_t*eye(n); %+++++++++++++++++
            b_eq_tau_x = d_comp + alph*k_t*I_comp; 

            A_eq_tau2 = []; 
            b_eq_tau2 = []; 

            A_ineq_tau = [];
            b_ineq_tau = []; 
        end 

    
 		% Zero Vel - Use Static Friction 
        zv_start_idx = x_end_idx + 1; 
        zv_end_idx = zv_start_idx + nzvi - 1; 

        if nzvi > 0 
        	% TODO -- probably consider gb fully efficient at zero vel?
        	% think through a little more (but then just alpha on next line)
            A_eq_tau_x(zero_vel_idxs, zv_start_idx:zv_end_idx) = alph;

            % Overwrite the other zero vel points, treat gb as fully eff 
            if eta < 1
            	A_eq_tau_x(zero_vel_idxs, gm_start_idx + zero_vel_idxs - 1) = alph*k_t;
            	A_eq_tau_x(zero_vel_idxs, mu_start_idx:mu_end_idx) = 0;
            	b_eq_tau_x(zero_vel_idxs) = d_comp(zero_vel_idxs) + alph*k_t*I_comp(zero_vel_idxs);
            end 

            % Friction Cone 
            A_ineq_zv = zeros(2*nzvi, dim_y);
            A_ineq_zv(1:nzvi, zv_start_idx:zv_end_idx) = 1; 
            A_ineq_zv(nzvi + (1:nzvi), zv_start_idx:zv_end_idx) = -1; 
            b_ineq_zv = tau_friction_static * ones(2*nzvi, 1);

            A_ineq_tau = [A_ineq_tau; A_ineq_zv];
            b_ineq_tau = [b_ineq_tau; b_ineq_zv];
        end 


        %A_eq_tau = [A_eq_tau1; A_eq_tau2; A_eq_tau_x];
        %b_eq_tau = [b_eq_tau1; b_eq_tau2; b_eq_tau_x];
        % TODO -----------------------------------------------------------------
        A_eq_tau = [A_eq_tau1; A_eq_tau2; A_eq_tau3; A_eq_tau_x];
        b_eq_tau = [b_eq_tau1; b_eq_tau2; b_eq_tau3; b_eq_tau_x];


        % Fill in variable bounds on I, Ip, In 
        lb(I_start_idx:I_end_idx) = -I_u; 
        ub(I_start_idx:I_end_idx) = I_u; 

        lb(Ip_start_idx:Ip_end_idx) = 0;    % non-negativity 
        ub(Ip_start_idx:Ip_end_idx) = I_u; 

        lb(In_start_idx:In_end_idx) = 0;    % non-negativity
        ub(In_start_idx:In_end_idx) = I_u; 


        if ~isempty(H)
            A_eq_h = zeros(size(H, 1), dim_y);
            A_eq_h(:, x_start_idx:x_end_idx) = -H;   % note signs
        else 
            A_eq_h = [];
        end

        A_eq = [A_eq_tau; A_eq_h]; 
        b_eq = [b_eq_tau; b]; % NOTE signs 

        % there is some processing to do here for if 
        % general constraints are SOCP OR QCP or simple inequality 
        % Dont want to need to redo analyis 

        % COULD define a seperate general inequality 
        % for when all quadratic terms are empty
        % fill this up in preprocessing and then this step
        % becomes easier 
        num_lin_ineq = nnz(obj.lin_ineq);
        num_quadcon = m - num_lin_ineq;

        if num_lin_ineq > 0 
            A_ineq_other = zeros(num_lin_ineq, dim_y);
            b_ineq_other = zeros(num_lin_ineq, 1); 
            idxs = find(obj.lin_ineq); 
        else 
            A_ineq_other = []; 
            b_ineq_other = [];
        end 

        if num_quadcon > 0 
            quadcon = struct(); % struct for quadratic/SOC constraints 
        end 

        qc_idx = 0; 
        lin_ineq_idx = 0; 

        for j = 1:m    % incorporate the quadratic constraints + other linear ineqs 
            cj = obj.c{j + 1};  % +1 bc 1 indexing 
            if isa(cj, 'function_handle')
                c_tmp = cj(motor, gearbox); 
            else 
                c_tmp = cj;
            end 

            rj = obj.r{j + 1};   % WHY SO SLOW ? - NOT EMPTY             
            if isa(rj, 'function_handle')
                r_tmp = rj(motor, gearbox); % slow 
            else 
                r_tmp = rj; 
            end 
            
            bet = obj.beta{j + 1}; 
            if isa(bet, 'function_handle')
                beta_tmp = bet(motor, gearbox); % slow 
            else 
                beta_tmp = bet; 
            end 
            q_tmp = zeros(dim_y, 1); 
            q_tmp(I_start_idx:I_end_idx) = c_tmp; 	%----------------------------------------------
			%q_tmp(Ip_start_idx:Ip_end_idx) = c_tmp;   %+++++++++++++++++++++++++++++
			%q_tmp(In_start_idx:In_end_idx) = -c_tmp;   %+++++++++++++++++++++++++++++++++++
            q_tmp(x_start_idx:x_end_idx) = r_tmp; 

            if obj.lin_ineq(j)  % if linear 
                lin_ineq_idx = lin_ineq_idx + 1; 
                A_ineq_other(lin_ineq_idx, :) = q_tmp'; 
                b_ineq_other(lin_ineq_idx) = -beta_tmp;
            else  % quad constraints 
                qc_idx = qc_idx + 1; 
                Qj_tmp = obj.Q{j + 1}(motor, gearbox); % +1 bc 1 indexing 
                Mj_tmp = obj.M{j + 1}(motor, gearbox); 

                num_vals = 2*nnz(Qj_tmp) + nnz(Mj_tmp); 
                
                % NOTE -- could speed these up 
                Qc_tmp = sparse([], [], [], dim_y, dim_y, num_vals); % NOTE: initializing as sparse will help speed 
                Qc_tmp(Ip_start_idx:Ip_end_idx, Ip_start_idx:Ip_end_idx) = Qj_tmp; 
                Qc_tmp(In_start_idx:In_end_idx, In_start_idx:In_end_idx) = Qj_tmp; 
                
                Qc_tmp(x_start_idx:x_end_idx, x_start_idx:x_end_idx) = Mj_tmp; 

                quadcon(qc_idx).Qc = Qc_tmp; 
                quadcon(qc_idx).q = q_tmp; 
                quadcon(qc_idx).sense = '<'; 
                quadcon(qc_idx).rhs = -beta_tmp; 
            end 

        end 


        if strcmp(obj.problem_type, 'fractional')
            % Augment H with equality 
            A_aug = zeros(2, dim_y); 
            A_aug(1, x_start_idx:x_end_idx) = obj.r_num - bound*obj.r_den;
            
            % Constrain denominator to be positive  
            A_aug(2, x_start_idx:x_end_idx) = -obj.r_den; % nonnegativitiy 
            b_aug = [bound*obj.beta_den - obj.beta_num; obj.beta_den];
        
        elseif strcmp(obj.problem_type, 'standard')
            A_aug = []; 
            b_aug = []; 
        else 
            error('Invalid Problem type ');
        end 

        % Add the linear inequalities in H_ineq, b_ineq
        if ~isempty(H_ineq)
        	% Add these to A_ineq other 
        	A_ineq_H = sparse([], [], [], size(H_ineq, 1), dim_y, nnz(H_ineq));
        	A_ineq_H(:, x_start_idx:x_end_idx) = H_ineq; 
        	b_ineq_H = b_ineq; 

        	A_ineq_other = [A_ineq_other; A_ineq_H];
        	b_ineq_other = [b_ineq_other; -b_ineq_H]; % NOTE signs 
        end  


        % TODO -- naming -- dont over b_ineq name 
        A_ineq = [A_ineq_tau; A_ineq_other; A_aug];
        b_ineq = [b_ineq_tau; b_ineq_other; b_aug];


        num_eq = length(b_eq);  
        num_ineq = length(b_ineq); 
        sense = [repmat('=', num_eq, 1); repmat('<', num_ineq, 1)]; 

        A_all = sparse([A_eq; A_ineq]); 
        b_all = [b_eq; b_ineq]; 


        %% ----- Incorporate Actual Cost --------------
        Q_cost = sparse([], [], [], dim_y, dim_y, 2*n^2 + w^2); % TODO -- sparse 

        Q0 = obj.Q{1}(motor, gearbox); 
        M0 = obj.M{1}(motor, gearbox); 

        c0_tmp = obj.c{1};
        if isa(c0_tmp, 'function_handle')
            c0 = c0_tmp(motor, gearbox)
        else 
            c0 = c0_tmp;
        end 
        
        r0_tmp = obj.r{1};
        if isa(r0_tmp, 'function_handle')
            r0 = r0_tmp(motor, gearbox)
        else 
            r0 = r0_tmp;
        end 
        
        beta0_tmp = obj.beta{1}; 
        if isa(beta0_tmp, 'function_handle')
            beta0 = beta0_tmp(motor, gearbox)
        else 
            beta0 = beta0_tmp;
        end 
        
        Q_cost(Ip_start_idx:Ip_end_idx, Ip_start_idx:Ip_end_idx) = Q0;
        Q_cost(In_start_idx:In_end_idx, In_start_idx:In_end_idx) = Q0; 
        Q_cost(x_start_idx:x_end_idx, x_start_idx:x_end_idx)= M0; 

        c_cost_init = zeros(dim_y, 1); 
        c_cost_init(I_start_idx:I_end_idx) = c0; %---------------------------------------------
		%c_cost_init(Ip_start_idx:Ip_end_idx) = c0;%+++++++++++++++++++++++++++++++++++++++++++
		%c_cost_init(In_start_idx:In_end_idx) = -c0;%++++++++++++++++++++++++++++++++++++++++++
        c_cost_init(x_start_idx:x_end_idx) = r0; 

        c_rho_aug = zeros(dim_y, 1); 
        c_rho_aug(Ip_start_idx:Ip_end_idx) = obj.settings.rho; 
        c_rho_aug(In_start_idx:In_end_idx) = obj.settings.rho; 

        c_cost = c_cost_init + c_rho_aug;

        if strcmpi(obj.settings.solver, 'gurobi')
            %% Gurobi Model 
            model.A = A_all;
            model.Q = Q_cost; 
            model.rhs = b_all; 
            model.sense = sense;   % equality/inequality sense 
            model.obj = c_cost;  % linear objective term
            model.objcon = beta0; % constant objective term 
            model.lb = lb; 
            model.ub = ub; 
            if num_quadcon > 0 
                model.quadcon = quadcon;
            end  
            model.vtype = repmat('C', dim_y, 1); % all continuous variables 
            model.modelsense = 'min'; % minimize or maximize 


            % TODO -- make a separete functoin 
            % we call regardless of solver???? 

            % TODO -- logging? 
            % TODO -- a way for users to change solver params 
            % like tolerances (--- 'Advanced Users only ----')
            params.cutoff = cutoff; 
            params.outputflag = 0;		% TODO 

            params.FeasibilityTol = 1e-8; % Default 1e-6
            params.OptimalityTol = 1e-9; % TODO % default 1e-6 
            params.BarConvTol = 1e-9; % defaulat 1e-8 
            params.BarQCPConvTol = 1e-9; % default 1e-6

            result = gurobi(model, params); 
            if strcmp(result.status, 'OPTIMAL')
                y_sol = result.x; 
                % subtract off cost augmentation 
                combo_cost = result.objval - c_rho_aug'*y_sol; 
            else 
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

				% ...maybe copy outside if for reuse? 
	            if ~isinf(result.objval)
	                y_sol = result.x; 
	                combo_cost = result.objval - c_rho_aug'*y_sol; 
	            else
	                y_sol = nan(dim_y, 1); 
	                combo_cost = inf;  
	            end 

			else % SEDUMI 

				result = sedumi_solve(c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc);

				% ...maybe copy outside if for reuse? 
	            if ~isinf(result.objval)
	                y_sol = result.x; 
	                combo_cost = result.objval - c_rho_aug'*y_sol; 
	            else
	                y_sol = nan(dim_y, 1); 
	                combo_cost = inf;  
	            end 
			
			end 
        else
            error('invalid solver'); % shouldnt be possible 
        end 

        %%
        %
        %		 Parse solutons 
        %
        %

        if isinf(combo_cost)
        	% Return nans 
        	sol = struct(); % empty 
        else 
        	% TODO 
			Ip = y_sol(Ip_start_idx:Ip_end_idx);    % pos current 
	        In = y_sol(In_start_idx:In_end_idx);	% neg current 

			%sol.I = y_sol(I_start_idx:I_end_idx);
			sol.I = Ip - In; 

			I_check = y_sol(I_start_idx:I_end_idx);



        	sol.x = y_sol(x_start_idx:x_end_idx); 

	        sol.tau = T*sol.x + d; 	% Total torque output 

	        % calculate compensation torque 
	        % TODO -- make sure to account for zero vel frictoi cone in outputs 
	        driving = [sol.tau .* sign_omega > 0];
	        driven =  [sol.tau .* sign_omega < 0];
	        zero_vel = [sign_omega == 0];
	        static_motor_friction = zeros(n, 1); 
	        static_motor_friction(zero_vel_idxs) = y_sol(zv_start_idx:zv_end_idx);

	        sol.tau_friction = -alph*(tau_motor_friction.*(eta*driving + ...
	        					driven/eta) -  static_motor_friction);
	        sol.tau_inertia = -alph*tau_motor_inertia.*(eta*driving + ...
	        				driven/eta + zero_vel) - tau_gearbox_inertia;

	        tau_m = k_t * sol.I; % from current 

	        sol.tau_em = alph*(tau_m.*(eta*driving + driven/eta + zero_vel)); 


	        % TODO -- friction compensatoin torque 
	        % TODO -- inertial compensation torque

	        % Everything should line 

	        % Add debug to settings 
	        
	        % TODO -- Fill in more debug 
	        if obj.settings.debug % add more things to output 

	        	Ip = y_sol(Ip_start_idx:Ip_end_idx);    % pos current 
	        	In = y_sol(In_start_idx:In_end_idx);	% neg current 
	        	I = Ip - In;

	        	% Making sure everything adds up 
	        	tau_check = sol.tau - (sol.tau_friction + sol.tau_inertia);

	        	%assert(max(abs(tau_check - sol.tau_em)) < 1e-6,...
	        	%					 'Torques do not add up');





	        	% The real questoin is how much does this model 
	        	% of inertial compensatoin matter??????????
	        	% vs lumped inertial compensatoin past motor

	        	% Check Validity of Proof 

	        	% Ip OR  In always negative 
	        	%IpIn = Ip.*In; 
	        	[max_current_error, mce_idx] = max(min(Ip, In)); 

	        	% Maximum Error From Convex Reformulation 
	        	max_objective_error = obj.settings.rho*sum(Ip + In);% in objective value


	        	if eta < 1
	        		sol.I_gm = y_sol(gm_start_idx:gm_end_idx);
	        		sol.I_mu = y_sol(mu_start_idx:mu_end_idx);

	        		gm = sol.I_gm;
	        		mu = sol.I_mu;

	        		gm_lb = lb(gm_start_idx:gm_end_idx);
					gm_ub = ub(gm_start_idx:gm_end_idx);

					mu_lb = lb(mu_start_idx:mu_end_idx);
					mu_ub = ub(mu_start_idx:mu_end_idx);
		

	        		gm_diff = sol.I_gm - I_comp;
	        		mu_diff = sol.I_mu - I_comp;

	        		[max_gm_mu_error, mgme_idx] = max(min(abs(gm_diff), abs(mu_diff)));

	       %{ 
        % Index Map for Optimization 
        I_start_idx = 1;
        I_end_idx = n; 

        % For code reuse, keep +/- decomp even if eta == 1
        Ip_start_idx = n + 1; 
        Ip_end_idx = 2*n; 

        In_start_idx = 2*n + 1;
        In_end_idx = 3*n; 
        %} 

	        		%if max_gm_mu_error > 1e-2
	        		if (eta < 1) && (min(I_comp) < 0 )

	        			I_l = -I_u; 

						rho = obj.settings.rho;
	        			

		        		cvx_begin quiet 
		        			variable tau_out(n, 1); 
		        			variable gm_cvx(n, 1);
		        			variable mu_cvx(n, 1);
		        			variable I_abs(n, 1);
		        			variable I_cvx(n, 1);
		        			variable x_cvx(w, 1);
		        			variable y_cvx(dim_y, 1);
		        			%binary variable delta(n, 1);
		        			variable delta(n, 1);
		        			variable bet(n, 1); 
		        			%variable sig(n, 1); 
		        			%variable phi(n, 1); 
		        			variable z(n, 1); 
		        			variable theta(n, 1); 

		        			minimize rho*(sum(gm_cvx) - sum(mu_cvx)) + (diag(Q0)'*z) + (c0'*I_cvx) + beta0 + quad_form(x_cvx, M0) + (r0'*x_cvx);

		        			subject to 
		        				abs(I_cvx) <= I_abs;
		        				T*x_cvx + d_comp == tau_out; 
		        				tau_out == alph*k_t*(eta*gm_cvx + (1/eta)*mu_cvx);
		        				gm_cvx >= 0; 
		        				mu_cvx <= 0; 
		        				gm_cvx + mu_cvx == I_cvx - I_comp; 


		        				% SIGMA AND PHI DO NOTHING HERE 
		        				% 


		        				%tau_out == alph*eta*k_t*sig + (alph*k_t/eta)*phi; 
		        				%sig + phi == I - I_comp; 

		        				0 <= delta;
		        				delta <= 1; 
		        				bet + delta == 1; 

		        				% FUN = I_comp - I, Fun Min = I_comp - I_u
		        				% FIN mAx = I_com - I_l

		        				epss = 1e-13; 

		        				I_comp - I_cvx <= (I_comp - I_l).*(1 - delta);
		        				I_comp - I_cvx >= epss + delta.*(I_comp - I_u - epss);
		        				

		        				gm_cvx + epss + (I_l - epss).*delta <= 0; 
		        				mu_cvx + epss + (I_l - epss).*bet <= 0;   % REDUNDANT? 

		        				%sig <= I_u .* delta;
		        				%sig >= 0; 
		        				%sig >= gm_cvx - I_u.*(1 - delta); 

		        				%phi <= 0;
		        				%phi >= I_l .* bet; 
		        				%phi <= mu_cvx - I_l.*(1 - bet); 
		        				%phi >= mu_cvx - 0;

		        				z >= 0; 
		        				theta >= 0;

		        				theta + 2*I_cvx.*I_comp - I_comp.^2 <= z; 
		        				
		        				(gm_cvx - mu_cvx).^2 <= theta;


		        				% cvx somehow saying infeasible 
		        				% when setup as integer program 

		        				% perhaps how its hadling booleans? 

		        				I_abs.^2 <= z; 

		        				if ~isempty(H)
		        					H*x_cvx + b == 0; 
		        				end 

		        				if ~isempty(H_ineq)
		        					H_ineq*x_cvx + b_ineq_H <= 0 ;
		        				end 

		        				I_abs <= I_u; % should limit 

		        				y_cvx(1:n) == I_cvx;
		        				y_cvx(n+1:2*n) == I_abs; % for quad costs   
		        				y_cvx(end - w + 1:end) == x_cvx; 

		        				% Quadrati constrsaints 
		        				for j = 1:numel(quadcon)
		        					Qc_tmp = quadcon(j).Qc;
		        					q_tmp = quadcon(j).q; 
		        					beta_tmp = -quadcon(j).rhs;


		        					% Split into components 
		        					Qj = Qc_tmp(n+1:2*n, n+1:2*n); % quad cost part 
		        					cj = q_tmp(1:n);

		        					Mj = Qc_tmp(end - w + 1:end, end - w + 1:end); 
		        					rj = q_tmp(end - w + 1:end);

		        					qd = diag(Qj);
							        (qd'*z) + (cj'*I_cvx) + quad_form(x_cvx, Mj) + (rj'*x_cvx) - beta_tmp <= 0 ;
							    end 


							    for j = 1:n
							    	if I_comp(j) >= 0 
							    		gm_cvx(j) - mu_cvx(j) <= I_abs(j) + I_comp(j);
							    	else
							    		% same as before, not helpful							    	
							    		gm_cvx(j) - mu_cvx(j) <= I_abs(j) - I_comp(j);
							    	end 


									quad_over_lin(gm_cvx(j), delta(j)) + quad_over_lin(-mu_cvx(j), bet(j)) <= theta(j);
		        				



							    end 
						  
						cvx_end 

						display(result)
						display(cvx_optval)

		        		[max_gm_mu_cvx, mgme_cvx_idx] = max(min(abs(gm_cvx - (I_cvx - I_comp)),...
		        							 abs(mu_cvx - (I_cvx - I_comp))));
		        		


		        		display(max_gm_mu_cvx);


		        		if max_gm_mu_cvx > 5e-2
		        			disp('.......awww fuckkkkkkkkkkkkkkkkk');
		        			keyboard
		        		else 
		        			disp('we gucciiiiiiiiii?')
		        		end 



		        		min_I = min(I_cvx);
		        		display(min_I); 


		        		% HONESTLY WHAT REALLY MATTERS IS TORQUE ERROR 


		        	end 
						        		% Process results from CVX 




	        		if max_gm_mu_error > 1e-2

	        			disp('....')
	        			disp('max gm mu error')


	        			vel_at_error = omega(mgme_idx);
	        			comp_current_at_error = I_comp(mgme_idx);
	        			display(mgme_idx)


	        			I_comp_bad = I_comp(mgme_idx);
	        			gm_bad = gm(mgme_idx);
	        			mu_bad = mu(mgme_idx);

	        			Ip_bad = Ip(mgme_idx);
	        			In_bad = In(mgme_idx);
	        			I_bad = I(mgme_idx);



	        			display(max_current_error);
	        			display(max_gm_mu_error);
	        		    display(vel_at_error);
	        			display(comp_current_at_error);
	        			keyboard

	        		end 

	        	end 

	        	if max_current_error > 1e-2		% 10 mA 
	        		disp('max current error')

	        		vel_at_error = omega(mce_idx);
	        		comp_current_at_error = I_comp(mce_idx);


	        		I_bad = I(mce_idx);
	        		Ip_bad = Ip(mce_idx);
	        		In_bad = In(mce_idx); 

	        		display(max_current_error);
	        		display(max_gm_mu_error);
	        		display(vel_at_error);
	        		display(comp_current_at_error);
	        		keyboard;
	        	end 


	        	%if max(abs(tau_check - sol.tau_em)) > 5e-4
	        	if max(abs(tau_check - sol.tau_em)) > 1e-2

	        		tmp = abs(tau_check - sol.tau_em);
	        		max_tau_error = max(tmp);
	        		display(max_tau_error); 

	        		disp('Torque matchup error')
	        		keyboard
	        	end 




	        	% PRINTS 

	        end 



        end 

        


        solve_time = toc(start_tic);



    end 




    function [cost_list, sol_structs, exitflag] = optimize_fractional(obj, num_return, combos)
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

        motors = table2struct(obj.motors);
        gearboxes = table2struct(obj.gearboxes);
        mincost = inf; % no feasible solution yet 

        global_ub = obj.cost_ub;   % global upper bound for solution POOL 
        global_lb = obj.cost_lb;
        num_combinations = size(combos, 1); 

        lb_list = global_lb * ones(num_combinations, 1);  
        %ub_list = global_ub * ones(num_combinations, 1); 
        ub_list = inf(num_combinations, 1); % no valid solutions yet


        % storing solutions separately so dont need to always recompute
        % if bounds already convered for a given design 
        sols = cell(num_combinations, 1); 

        max_abs_bnd_diff = inf;
        max_rel_bnd_diff = inf; 

        outer_loop_iter = 1; 

        table_line = [repmat('-', 1, 80), '\n']; 
        obj.vprintf(1, table_line);
        headers = [" Outer Iter  |", "Combos   |", "   LB    |",...
        				"Pool UB   |", " Min Cost   |", "    Time  "];
        header_txt = sprintf('%+14s%+14s%+13s%+13s%+13s%+11s\n', headers);
        obj.vprintf(1, header_txt);
        obj.vprintf(1, table_line);
        disp_txt = []; 

        bad_lower_bound_flag = false; 
        bad_upper_bound_flag = false; 


        while (max_abs_bnd_diff > obj.settings.qcvx_abstol) && ...
              (max_rel_bnd_diff > obj.settings.qcvx_reltol) && ...
               ~bad_lower_bound_flag && ~bad_upper_bound_flag

            pq = PQ(true); % max priority queue -- new every loop 

            for j = 1:num_combinations % Our loop thoufg   
                motor = motors(combos(j, 1));
                gearbox = gearboxes(combos(j, 2));

                % while not converged on this + break condition below
                % Need each upper and lower bound to approach each other every iter
                if outer_loop_iter == 1
                	init_flag = true;  
                end 

                while ( ((ub_list(j) - lb_list(j)) > obj.settings.qcvx_abstol) && ...
                      ((ub_list(j) - lb_list(j)) > obj.settings.qcvx_reltol*max(min(ub_list(j), global_ub), abs(lb_list(j)))) &&...
                     (lb_list(j) <= global_ub) )  || init_flag %
                   
                    if init_flag 
                        bound = global_ub;
                        init_flag = false; 
                    else 
                        % upper bound found for this combo may be 
                        % lower than the global boudn for the solution pool 
                        bound = (lb_list(j) + min(ub_list(j), global_ub))/2; 
                    end 

                    [combo_cost, tmp_sol, comp_time] = obj.combo_solve(motor,...
                                                 gearbox, bound); 

                    % bound update 
                    if isinf(combo_cost)    % infeasible 
                        lb_list(j) = bound; % if init bounds infeas, will set lb/ub to same 
                    else    % feasible 
                        % this will always be a better solution than
                        % previously had for this design 
                        sols{j} = tmp_sol;
                        ub_list(j) = bound; 
                    end 

                    if lb_list(j) >= global_ub
                        lb_list(j) = ub_list(j); % set so we can ignore these 
                    end 

                    % true cost could actually be the lower bound 
                    % in this case, always feasible and never move lower bound
                    % so need to check convergence 
                    if lb_list(j) > global_lb  % if moved boud up beyond global lb 
                        % dont want to spend all the time right here completely
                        % tightening the bounds because this might be a bad
                        % design so move onto the next design for now 
                        break;  
                    end 
                end 



                % Check if upper bound for design we just considered goes in the PQ 
                %if ub_list(j) < global_ub   % ie. this goes into sol pool
                if (ub_list(j) < global_ub) || ...
                		((ub_list(j) <= global_ub) && (pq.size() < num_return))	
                    
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

                    % There may be fewer solutions than num return
                    % then update the cutoff      
                    [global_ub, ~] = pq.peek(); % update the cutoff 

                    % For the actual minimum cost 
                    if  ub_list(j) < mincost
                        mincost = ub_list(j); 

                        % May have closed the gap and lower bound not low enough 

                        cost_diff = abs(mincost - global_lb);  
                        if (cost_diff < obj.settings.qcvx_abstol) || ...
                    		(cost_diff < obj.settings.qcvx_reltol*...
                    				   max(abs([mincost, lb_list(j)])) );
            				obj.vprintf(1, ['\nLower bound (lb) is feasible. ',...
            		        	        'Reduce lower bound and reoptimize']);
            				
            				bad_lower_bound_flag = true; 
            				pq = PQ(true); % clear it -- return nothing 
            				break;  % break the while on this 
            			end 
            		end 
                end 

                if lb_list(j) > global_ub  
                    lb_list(j) = ub_list(j); % so we stop wasting time 
                end 

                % Prints 
                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
        		frac_txt = sprintf('%d/%d', j, num_combinations);
				disp_txt = sprintf('%13d %13s %12.2e %12.2e %12.2e %11.2f',...
								 outer_loop_iter, frac_txt,...
								global_lb, global_ub, mincost, toc(start_tic));                
                obj.vprintf(1, disp_txt);
            end 

            bnd_diff = ub_list - lb_list; 
            rel_bound_diff = bnd_diff./max(ub_list, abs(lb_list));
            max_abs_bnd_diff = max(bnd_diff);   % inf - inf = nan
            max_rel_bnd_diff = max(rel_bound_diff); 


            % Update global lower bounds 
            global_lb = min(lb_list); % no design can beat this 

            % If no solution found at end of first outer loop, break 
            if (outer_loop_iter == 1) && isinf(mincost)
            	obj.vprintf(1, '\nNo solutions found. Try increasing the upper bound');
            	bad_upper_bound_flag = true; 
            end 

            outer_loop_iter = outer_loop_iter + 1; 
            disp_txt = []; 
            obj.vprintf(1, '\n');
        end 
        
        num_sols = pq.size(); % may not have found num_return feas sols 

        if num_sols == 0
        	sol_structs = struct(); % empty struct; 
        elseif num_sols < num_return
        	obj.vprintf(1, 'Only %d feasible solutions found\n', num_sols); 
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
