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

    
    settings % for solver, rho, verbose, print frequency 
    rankings % for combos - may want to rename 
    ranking_matrix 

    % ----- Problem Data -- May want more dscriptive names?
    omega
    omega_dot 
    % indices in omega corresponding to omega == 0 
    zero_vel_idxs  	% computed once for each inputs to help solve time 
    Q   % cell array of the diagonal vectors of the Q matrices 
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
        if settings.debug; fprintf('Debug is on...\n');                 end; 
        if ~isfield(settings, 'rho'); settings.rho = 1e-4;              end
        if ~isfield(settings, 'verbose'); settings.verbose = 1;         end
        if ~isfield(settings, 'print_freq'); settings.print_freq = 500; end 
        if ~isfield(settings, 'qcvx_reltol')
        	settings.qcvx_reltol = 1e-3; 
        end 
        if ~isfield(settings, 'qcvx_abstol')
        	settings.qcvx_abstol = 1e-7; 
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
    		% NOTE: maybe rename 
    %
    %   Added:   
    %**********************************************************************
    function update_problem(obj, problem_data, varargin)
    %
    %
    %
    %
    %
    %
    %
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
        		 		b_ineq = @(motor, gearbox) b_ineq(:);   	  end 
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

        % We will convert Q to vectors containing the diagonal 
        % Do we want to change it in the input???????
        % NOTE: whats easier for the user 



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
            	Qj = full(diag(Q{j}(test_motor, test_gearbox)));
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



        disp_txt = []; 

    
        if strcmp(obj.problem_type, 'standard')  

            num_combinations = size(combos, 1); % TODO - combine with above? 
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

            for j = 1:num_combinations 
                motor = motors(combos(j, 1));
                gearbox = gearboxes(combos(j, 2)); 

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

        %combo_list = combo_list(1:min(length(combo_list), 200), :); 
        %warning('remember to comment this debug thing out limiting combos to 200')
    

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



    function [combo_cost, sol, solve_time, exitflag] = combo_solve(obj,...
                                                        motor, gearbox,...
                                                                bound)


        % TODO -- add more output info for repeated solves OR MIQP solves 

    	start_tic = tic; 		% to get timing 
    	[matrices, comp_torques, cutoff, index_map] = ...
                                     obj.build_matrices(motor, gearbox, bound); 

        % Move to subfunction 

        % Inputs: Matrices 

        % Outputs: y_sol, combo_cost, some flags
        [y_sol, combo_cost] = obj.socp_solve(matrices, cutoff);

        % instead if bad_idxs -- return which way to force the problem 
        [sol, bad_idxs, directions] = parse_solution(obj, y_sol, index_map,...
                                                comp_torques,  motor, gearbox); 

        exitflag = 0; % no issues with solve 


        if ~isempty(bad_idxs)  % need to run next solve 

            if ~isfield(index_map, 'del')
                disp('how the hell....')
                keyboard
            end 

            exitflag = 1; % repeated solve 

            % Try fixing the drive/driven direction for the bad idxs 
            matrices_debug = matrices; % TODO - remove 

            % Augment the matrices 
            nbi = length(bad_idxs); 
            dim_y = length(matrices.ub);
            
            %A_del_aug = sparse(1:nbi, index_map.del(bad_idxs), directions,...
            %                            nbi, dim_y); 
            %disp('sadsada')
            [~, shifted_idxs] = intersect(index_map.binary, bad_idxs, 'stable');
            A_del_aug = sparse(1:nbi, index_map.del(shifted_idxs), directions,...
                                        nbi, dim_y); 

            b_del_aug = directions; 

            matrices.A_eq = [matrices.A_eq; A_del_aug];
            matrices.b_eq = [matrices.b_eq; b_del_aug];

            % NOTE: calling without cuttoffs here - if use cutoff and get 
            % nan result its possible that its feasbile just over the cutoff
            [y_sol_new, combo_cost_new] = obj.socp_solve(matrices, inf);

            [sol_new, bad_idxs_new, directions_new] = parse_solution(obj,...
                                                 y_sol_new, index_map,...
                                                comp_torques,  motor, gearbox); 

            % 
            %   If cost for fixed version is too different than cost 
            %   for non-fixed version -- call the mixed int solver 
            % 

            % ideally this criteria is pretty loose while still being rigorous 

            % would be always > 0 except for the rho augmentation  
            cost_diff = combo_cost_new - combo_cost; 

            

            % TODO - rigor 
            if cost_diff > 1e-2 

        %       If still NOT good -- call mixed integer solver
        %           -- however, may want to save this problem for layer 
        %       -- ALSO definitely feed information back to caller 
        %       in logging want to know how many problem required multiple solves
        %       and how many required mixed integer solvers
        %

                disp('cost too diff call mixed into')
                keyboard 

                exitflag = 2; 
            end 


        end 





        solve_time = toc(start_tic);

    end 



    function [cost_list, sol_structs, exitflag] = ...
                                    optimize_fractional(obj, num_return, combos)
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
                motor = motors(combos(j, 1));
                gearbox = gearboxes(combos(j, 2));

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

                            cost_diff = mincost - obj.cost_lb;  

                            if (cost_diff < obj.settings.qcvx_abstol) || ...
                            		(cost_diff < obj.settings.qcvx_reltol*...
                              			max(abs([mincost, obj.cost_lb])));
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
        d = obj.d(motor, gearbox); 

        tau_gearbox_inertia = gearbox.inertia * (alph^2) * omega_dot; 
        
        I_u = obj.I_u(motor, gearbox);  % symmetric bounds, dont need I_l
        I_l = -I_u; 

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
        tau_friction_static = 1.1 * k_t * motor.I_nl;  % scalar 
        % tau_motor_friction is JUST kinetic 
        tau_motor_friction = k_t * motor.I_nl * sign_omega; % NOTE: Incorp other friction models 
      
        % Since BEFORE gearbox only multiplied by alpha NOT alpha^2 
        % will effectively get multiplied by alpha again through gearbox
        tau_motor_inertia = motor.inertia * alph * omega_dot; 
        
  		% Dynamic Motor Friction and Inertia Compensation  (f - fric, j- inerits )
        tau_mfj = tau_motor_friction + tau_motor_inertia; 
        I_comp = tau_mfj/k_t; % corresponding current 

        % FOR SOME DEBUG 

        % Index Map for Optimization 
        I_idxs = 1:n;
        Isq_idxs = I_idxs(end) + (1:n);	 % equivalent to I squared 
        tau_idxs = Isq_idxs(end) + (1:n); 
        x_idxs = tau_idxs(end) + (1:w); 

        index_map.I = I_idxs; 
        index_map.Isq = Isq_idxs; 
        index_map.tau = tau_idxs;
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
                aug_idxs = find(Ico < 0); % indices where we need the extra help 
                num_aug = length(aug_idxs);

                index_map.binary = aug_idxs; % NOTE: unlike everything else NOT an index into sol vector

        		%del_idxs = s_idxs(end) + (1:n); 
            	%sig_idxs = del_idxs(end) + (1:n); 
            	%gmsq_idxs = sig_idxs(end) + (1:n);
            	%musq_idxs = gmsq_idxs(end) + (1:n);
                del_idxs = s_idxs(end) + (1:num_aug); 
                sig_idxs = del_idxs(end) + (1:num_aug); 
                gmsq_idxs = sig_idxs(end) + (1:num_aug);
                musq_idxs = gmsq_idxs(end) + (1:num_aug);

            	index_map.del = del_idxs;
            	index_map.sig = sig_idxs;
            	index_map.gmsq = gmsq_idxs;
            	index_map.musq = musq_idxs; 

            	dim_y = musq_idxs(end) + nzvi;  % total dimension of opt vector 

            	% Bounds on optimizatoin vatiable 
        		lb = -inf(dim_y, 1);		
        		ub = inf(dim_y, 1); 

            	%lb(gmsq_idxs) = 0;  
            	%ub(gmsq_idxs) = max((I_l - I_comp).^2, (I_u - I_comp).^2); 
            	%lb(musq_idxs) = 0;
            	%ub(musq_idxs) = max((I_l - I_comp).^2, (I_u - I_comp).^2); 
                lb(gmsq_idxs) = 0;  
                ub(gmsq_idxs) = max((I_l(aug_idxs) - I_comp(aug_idxs)).^2, (I_u(aug_idxs) - I_comp(aug_idxs)).^2); 
                lb(musq_idxs) = 0;
                ub(musq_idxs) = ub(gmsq_idxs); 

				lb(del_idxs) = 0; 	ub(del_idxs) = 1; 
				lb(sig_idxs) = 0; 	ub(sig_idxs) = 1; 
       	
        	end 

        	% Bounds on gm, mu, s 
			lb(gm_idxs) = min(-sign_omega .* (I_l - I_comp), 0);
			ub(gm_idxs) = max(sign_omega .* (I_u - I_comp), 0);

			lb(mu_idxs) = min(so .* (I_l - I_comp), 0);
			ub(mu_idxs) = max(-so .* (I_u - I_comp), 0);

			lb(s_idxs) = 0; 
			ub(s_idxs) = max((I_l - I_comp).^2, (I_u - I_comp).^2); 

			%
            %   Rows 1:n        Tx + d = tau_out  
            %   Rows n+1:2n     tau_out + tau_gb_inertia = motor/gb torque  
			%		
            A_eq_tau_x = zeros(2*n, dim_y);    % sparse? 

            num_vals = nnz(T) + 4*n + nzvi; 
            A_eq_tau_x = sparse([], [], [], 2*n, dim_y, num_vals);



            b_eq_tau_x = zeros(2*n, 1);     % to be filled in 

            A_eq_tau_x(1:n, tau_idxs) = eye(n);
            A_eq_tau_x(1:n, x_idxs) = -T; 
            b_eq_tau_x(1:n) = d; 

            A_eq_tau_x(n + (1:n), tau_idxs) = -eye(n); 
            A_eq_tau_x(n + (1:n), gm_idxs) = alph*eta*k_t*eye(n);
            A_eq_tau_x(n + (1:n), mu_idxs) = (alph*k_t/eta)*eye(n);
            b_eq_tau_x(n + (1:n)) = tau_gearbox_inertia;

           
            %		
            % 			gamma + mu = I - I_comp
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

            A_ineq_other = []; 
            b_ineq_other = []; 

            %
            %		Extra Linear Constraints 
            %
            if binary_aug 
				% For assignment of delta with driving/driven 
				epss = 1e-9; % tolerancing 

				%
				%    Relate Delto to Driving/Driven 
				%		[Delta = 1] <===> [Driving]
				%	
				% 	When all else fails, we will force Delta 
				%   to take on binary values and call a mixed integer
				%	solver to handle this 

                % ONLY DO AT INDICES WHERE ITS NEEDED 
                so_aug = so(aug_idxs); 

                %{
                A_del = zeros(2*n, dim_y);

				A_del(1:n, I_idxs) = -diag(so);
				A_del(1:n, del_idxs) = diag(so.*I_comp + I_u);

				A_del(n + (1:n), I_idxs) = diag(so);
				A_del(n + (1:n), del_idxs) = diag(so.*I_comp - I_u - epss);
				b_del = [I_u; so.*I_comp - epss];
                %}
                I_comp_aug = I_comp(aug_idxs);
                I_u_aug = I_u(aug_idxs);
                I_l_aug = I_l(aug_idxs);

                A_del = zeros(2*num_aug, dim_y);
                A_del(1:num_aug, I_idxs(aug_idxs)) = -diag(so_aug);
                A_del(1:num_aug, del_idxs) = diag(so_aug.*I_comp_aug + I_u_aug);

                A_del(num_aug + (1:num_aug), I_idxs(aug_idxs)) = diag(so_aug);
                A_del(num_aug + (1:num_aug), del_idxs) = diag(so_aug.*I_comp_aug - I_u_aug - epss);
                b_del = [I_u_aug; so_aug.*I_comp_aug - epss];


				% gm \equiv to delta*(I - I_comp)
				%tmp_max = I_u - I_comp;
				%tmp_min = I_l - I_comp;
                tmp_max = I_u_aug - I_comp_aug;
                tmp_min = I_l_aug - I_comp_aug;			
                % TODO -- switch to sparse 
                %{
				A_del_gm = zeros(3*n, dim_y);


				A_del_gm(1:n, gm_idxs) = eye(n);
				A_del_gm(1:n, del_idxs) = -diag(tmp_max);

				
				A_del_gm(n + (1:n), gm_idxs) = -eye(n);
				A_del_gm(n + (1:n), del_idxs) = diag(tmp_min);

			
				A_del_gm(2*n + (1:n), gm_idxs) = eye(n);
				A_del_gm(2*n + (1:n), del_idxs) = diag(tmp_min);
				A_del_gm(2*n + (1:n), I_idxs) = -eye(n);
			
				A_del_gm(3*n + (1:n), gm_idxs) = -eye(n);
				A_del_gm(3*n + (1:n), del_idxs) = diag(tmp_max);
				A_del_gm(3*n + (1:n), I_idxs) = eye(n);
				b_del_gm = [zeros(2*n, 1); -I_comp - tmp_min; I_comp + tmp_max];
                %}
                A_del_gm = zeros(3*num_aug, dim_y);
                
                A_del_gm(1:num_aug, gm_idxs(aug_idxs)) = eye(num_aug);
                A_del_gm(1:num_aug, del_idxs) = -diag(tmp_max);

                A_del_gm(num_aug + (1:num_aug), gm_idxs(aug_idxs)) = -eye(num_aug);
                A_del_gm(num_aug + (1:num_aug), del_idxs) = diag(tmp_min);

            
                A_del_gm(2*num_aug + (1:num_aug), gm_idxs(aug_idxs)) = eye(num_aug);
                A_del_gm(2*num_aug + (1:num_aug), del_idxs) = diag(tmp_min);
                A_del_gm(2*num_aug + (1:num_aug), I_idxs(aug_idxs)) = -eye(num_aug);
            
                A_del_gm(3*num_aug + (1:num_aug), gm_idxs(aug_idxs)) = -eye(num_aug);
                A_del_gm(3*num_aug + (1:num_aug), del_idxs) = diag(tmp_max);
                A_del_gm(3*num_aug + (1:num_aug), I_idxs(aug_idxs)) = eye(num_aug);
                b_del_gm = [zeros(2*num_aug, 1);...
                         -I_comp_aug - tmp_min; I_comp_aug + tmp_max];
							
                % TODO SPEED UP             
				A_ineq_other = [A_ineq_other; A_del; A_del_gm];
				b_ineq_other = [b_ineq_other; b_del; b_del_gm];

                %{
				%
				%		 gm_sq + mu_sq = s 
				%
				A_gmmu_sq = zeros(n, dim_y);
				A_gmmu_sq(:, gmsq_idxs) = eye(n);
				A_gmmu_sq(:, musq_idxs) = eye(n);
				A_gmmu_sq(:, s_idxs) = -eye(n);
				b_gmmu_sq = zeros(n, 1);
	
				%
				%		 delta + sigma = 1 
				%
				A_del_sig = zeros(n, dim_y);
				A_del_sig(:, del_idxs) = eye(n);
				A_del_sig(:, sig_idxs) = eye(n);
				b_del_sig = ones(n, 1);
                %}
                
                %
                %        gm_sq + mu_sq = s 
                %
                A_gmmu_sq = zeros(num_aug, dim_y);
                A_gmmu_sq(:, gmsq_idxs) = eye(num_aug);
                A_gmmu_sq(:, musq_idxs) = eye(num_aug);
                A_gmmu_sq(:, s_idxs(aug_idxs)) = -eye(num_aug);
                b_gmmu_sq = zeros(num_aug, 1);
    
                %
                %        delta + sigma = 1 
                %
                A_del_sig = zeros(num_aug, dim_y);
                A_del_sig(:, del_idxs) = eye(num_aug);
                A_del_sig(:, sig_idxs) = eye(num_aug);
                b_del_sig = ones(num_aug, 1);

				A_eq_other = [A_eq_other; A_del_sig; A_gmmu_sq];
				b_eq_other = [b_eq_other; b_del_sig; b_gmmu_sq];
			end 
        else  % NO GEARBOX 
            dim_y = x_idxs(end) + nzvi; 
            lb = -inf(dim_y, 1);		
        	ub = inf(dim_y, 1);

            A_eq_tau_x = zeros(2*n, dim_y);    % sparse? 

            b_eq_tau_x = zeros(2*n, 1);
            A_eq_tau_x(1:n, tau_idxs) = eye(n);
            A_eq_tau_x(1:n, x_idxs) = -T; 
            b_eq_tau_x(1:n) = d; 

            A_eq_tau_x(n + (1:n), tau_idxs) = -eye(n);
            A_eq_tau_x(n + (1:n), I_idxs) = alph*k_t*eye(n);
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
        ub(Isq_idxs) = I_u.^2; 

        lb(tau_idxs) = -gearbox.Max_int_torque;
        ub(tau_idxs) = gearbox.Max_int_torque; 

        %
        %
        %		Account for Static Friction Cone
        %		for points with zero velocity
        %
        %		Adjust A_eq_tau_x and b_eq_tau_x accordingly
        %

 		% Zero Vel - Use Static Friction 
 		zv_idxs = x_idxs(end) + (1:nzvi);  % indices in opt vec NOT omega 
        index_map.zv_idxs = zv_idxs;
        if nzvi > 0 
        	% TODO -- probably consider gb fully efficient at zero vel?
        	% think through a little more (but then just alpha on next line)
            A_eq_tau_x(zero_vel_idxs, zv_idxs) = alph;

            % Overwrite the other zero vel points, treat gb as fully eff 
            if eta < 1
            	% Replace alpha*kt*eta with alph*kt*eta on those rows 
            	A_eq_tau_x(zero_vel_idxs, gm_idxs(1) + zero_vel_idxs - 1) = alph*k_t;
            	A_eq_tau_x(zero_vel_idxs, mu_idxs) = 0;
            	b_eq_tau_x(zero_vel_idxs) = d(zero_vel_idxs) + tau_gearbox_inertia(zero_vel_idxs); 
            end 

            % Friction Cone 
            A_ineq_zv = sparse([], [], [], 2*nzvi, dim_y, 2*nzvi);
            A_ineq_zv(1:nzvi, zv_idxs) = 1; 
            A_ineq_zv(nzvi + (1:nzvi), zv_idxs) = -1; 
            b_ineq_zv = tau_friction_static * ones(2*nzvi, 1);

            % Stack em up
            A_ineq_other = [A_ineq_other; A_ineq_zv];
            b_ineq_other = [b_ineq_other; b_ineq_zv];
        end 

        if ~isempty(H)
            %A_eq_H = zeros(size(H, 1), dim_y);
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
        num_lin_ineq = nnz(obj.lin_ineq);
        if num_lin_ineq > 0 
            A_ineq_extra = zeros(num_lin_ineq, dim_y);
            b_ineq_extra = zeros(num_lin_ineq, 1); 
            idxs = find(obj.lin_ineq); 
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
            q_tmp(I_idxs) = c_tmp; 
            q_tmp(Isq_idxs) = obj.Q{j + 1}(motor, gearbox); 
            q_tmp(x_idxs) = r_tmp; 

            if obj.lin_ineq(j)  % if linear 
                lin_ineq_idx = lin_ineq_idx + 1; 
                A_ineq_extra(lin_ineq_idx, :) = q_tmp'; 
                b_ineq_extra(lin_ineq_idx) = -beta_tmp;
            else  % quad constraints 
                qc_idx = qc_idx + 1; 

                Mj_tmp = obj.M{j + 1}(motor, gearbox); 

                num_vals = nnz(Mj_tmp); 
                
                % NOTE -- could speed these up 
                Qc_tmp = sparse([], [], [], dim_y, dim_y, num_vals); % NOTE: initializing as sparse will help speed 
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
	        %		gm^2 <= s.*delta
	        %		mu^2 <= s.*(beta) -- need to define beta 
  
	        if binary_aug 
                gm_aug_idxs = gm_idxs(aug_idxs);  % USEFUL ABOVE TODO 
                mu_aug_idxs = mu_idxs(aug_idxs); 
	        	%for i = 1:n 
                for i = 1:num_aug 

	        		% gm^2 <= gm_sq .*delta 
                    %{
			    	qc_idx = qc_idx + 1; 
			        row = [gm_idxs(i); gmsq_idxs(i); del_idxs(i)];
			        col = [gm_idxs(i); del_idxs(i); gmsq_idxs(i)];
			        vals = [1; -0.5; -0.5]; 
			        Qc_tmp = sparse(row, col, vals, dim_y, dim_y);
			        quadcon(qc_idx).Qc = Qc_tmp; 
		            quadcon(qc_idx).q = zeros(dim_y, 1);
		            quadcon(qc_idx).sense = '<'; 
		            quadcon(qc_idx).rhs = 0; 
                    %} 
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
                    %{
			    	qc_idx = qc_idx + 1; 
			        row = [mu_idxs(i); musq_idxs(i); sig_idxs(i)];
			        col = [mu_idxs(i); sig_idxs(i); musq_idxs(i)];
			        vals = [1; -0.5; -0.5]; 
			        Qc_tmp = sparse(row, col, vals, dim_y, dim_y);
			        quadcon(qc_idx).Qc = Qc_tmp; 
		            quadcon(qc_idx).q = zeros(dim_y, 1);
		            quadcon(qc_idx).sense = '<'; 
		            quadcon(qc_idx).rhs = 0; 
                    %}
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
	    else 
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


        if strcmp(obj.problem_type, 'fractional')
            % Augment H with equality 
            A_lin_frac = zeros(2, dim_y); 
            A_lin_frac(1, x_idxs) = obj.r_num - bound*obj.r_den;
            
            % Constrain denominator to be positive  
            A_lin_frac(2, x_idxs) = -obj.r_den; % nonnegativitiy 
            b_lin_frac = [bound*obj.beta_den - obj.beta_num; obj.beta_den];
        
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

        % TODO -- naming -- dont over b_ineq name 
        A_ineq = [A_ineq_other; A_ineq_extra; A_lin_frac];
        b_ineq = [b_ineq_other; b_ineq_extra; b_lin_frac];

        %% ----- Incorporate Actual Cost --------------
        [M0_row, M0_col, M0_val] = find(obj.M{1}(motor, gearbox)); 
        Q_cost = sparse(x_idxs(1) + M0_row - 1, x_idxs(1) + M0_col - 1,...
        									 M0_val, dim_y, dim_y); 

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
        
        Q0 = obj.Q{1}(motor, gearbox); % diag vector 
        c0_tmp = obj.c{1};
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
        	c_rho_aug(gm_idxs) = so.* obj.settings.rho; 
        	c_rho_aug(mu_idxs) = -so .* obj.settings.rho; 
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
    	matrices.A_ineq = A_ineq; 
    	matrices.b_ineq = b_ineq;
    	matrices.lb = lb; 
    	matrices.ub = ub; 
    	matrices.quadcon = quadcon; 

       
    	% 
        compensation_torques.I_comp = I_comp;  % maybe inclue in comp values? 
    	compensation_torques.tau_motor_friction = tau_motor_friction;
    	compensation_torques.tau_motor_inertia = tau_motor_inertia; 
    	compensation_torques.tau_gearbox_inertia = tau_gearbox_inertia;
    	compensation_torques.tau_friction_static = tau_friction_static; % scalar

    end % 



    function  [sol, bad_idxs, directions] = parse_solution(obj, y_sol, index_map,...
                                                comp_torques,  motor, gearbox)
    %
    %
    %
    %
    %
    %
        bad_idxs = []; % initialize empty 
        directions = [];  % initialize empty 

        if isnan(y_sol(1))   % TODO -- handle non-solutions cleaner 
            % Return nans 
            sol = struct(); % empty 
            return; 
        end 
        

        % If Actual solution  
        k_t = motor.k_t; 
        alph = gearbox.alpha; 
        eta = gearbox.eta; 


        I_comp = comp_torques.I_comp; 

        sol.I = y_sol(index_map.I); 
        sol.x = y_sol(index_map.x); 


        sol.I_comp = I_comp; 

        I_shift = sol.I - I_comp; 
         
        zero_vel = [sign(obj.omega) == 0];
        static_motor_friction = zeros(obj.n, 1); 
        static_motor_friction(obj.zero_vel_idxs) = y_sol(index_map.zv_idxs);

        tau_motor_friction = comp_torques.tau_motor_friction;
        tau_motor_inertia = comp_torques.tau_motor_inertia;
        tau_gearbox_inertia = comp_torques.tau_gearbox_inertia;
        tfs = comp_torques.tau_friction_static;

        % sol.tau -- torque felt at the joint (afte accounting for torque to acell gearbox )
        if eta < 1
            sol.I_gm = y_sol(index_map.gm);
            sol.I_mu = y_sol(index_map.mu);
            sol.tau = alph*k_t*eta*(sol.I_gm) + (alph*k_t/eta)*(sol.I_mu)...
                     + alph*static_motor_friction - tau_gearbox_inertia;
        else 
            sol.tau = alph*k_t*(I_shift) ...
                    + alph*static_motor_friction - tau_gearbox_inertia;
        end 

        Isq = y_sol(index_map.Isq);


        % calculate compensation torque 
        % TODO -- make sure to account for zero vel frictoi cone in outputs 
        
        % For debug 
        tau_comp = sol.tau + tau_gearbox_inertia;  % DOUBLE CHECK 
       
        % in this context, strictly greater 
        driving = [tau_comp .* sign(obj.omega) > 0];  % note OR equals 
        driven = [tau_comp .* sign(obj.omega) < 0];

        
        % TODO -- rename zero vel things to make it more clear its torque 
        % ADD THESE
        sol.tau_friction = -alph*(tau_motor_friction.*(eta*driving + ...
                            driven/eta) -  static_motor_friction);
        sol.tau_inertia = -alph*tau_motor_inertia.*(eta*driving + ...
                        driven/eta + zero_vel) - tau_gearbox_inertia;
        tau_m = k_t * sol.I; % from current 

        % Just electromechanical 
        sol.tau_em = alph*(tau_m.*(eta*driving + driven/eta + zero_vel)); 


        % Everything should line 

        % Add debug to settings 
        

        % 

        % TODO -- let the tolerancing on these be 
        % something that can be set (for tau and gm/mu)
        tau_tol = 1e-2; 
        tau_check = sol.tau - (sol.tau_friction + sol.tau_inertia);
        tau_error = tau_check - sol.tau_em; 

        tau_bad_idxs = find(abs(tau_error) > tau_tol); 
        bad_idxs = tau_bad_idxs;

        directions = driving(bad_idxs); 







        %{
        gmmu_tol = 1e-3; 

        if true % NOT A DEBUG THING -- BUT OTHERS MIGHT BE NOTE 


            % Making sure everything adds up 


            [max_tau_error, mte_idx] = max(abs(tau_error));


            gmmu_bad_idxs = []; % will overwrite 


            if max_tau_error > gmmu_tol
                % NOTE: if this get embeded may wanto return something on these conditoins 
                display(max_tau_error); 
                disp('Torque matchup error')
            end 

            if eta < 1

                s = y_sol(index_map.s);

                % TO help debug
                I = sol.I;
                gm = sol.I_gm;
                mu = sol.I_mu;

                % Maximum Error From Convex Reformulation 
                so = round(sign(obj.omega) + 0.1); 
                max_objective_error = obj.settings.rho*sum(so.*(gm - mu));% in objective value

    
                I_shift = sol.I - I_comp; 
                gmmu_error = min(abs(gm - I_shift), abs(mu - I_shift));

                [max_gm_mu_error, mgme_idx] = max(gmmu_error);
                    
                gmmu_bad_idxs = find(gmmu_error > gmmu_tol);     


                if (max_gm_mu_error > gmmu_tol) && (max_tau_error >= tau_tol)

                    if isfield(index_map, 'del') % if binary aug 

                        del = y_sol(index_map.del);
                        sig = y_sol(index_map.sig);

                        gmsq = y_sol(index_map.gmsq);
                        musq = y_sol(index_map.musq);
                        

                    else 

                        disp('how the FFFFFFFFFFFFFFFFFF')
                        keyboard
                        %error('SHOULDNT BE POSSIBLE')
                        n = length(sol.I); 
                        del = zeros(n, 1);
                        sig = zeros(n, 1);


                    end 

                    disp('....')
                    disp('max gm mu error')

                    % TODO -- remove these, not helpful 

                    vel_at_error = obj.omega(mgme_idx);
                    comp_current_at_error = I_comp(mgme_idx);

                    gm_bad = gm(mgme_idx);
                    mu_bad = mu(mgme_idx);
                    I_bad = sol.I(mgme_idx);
                    Isq_bad = Isq(mgme_idx);
                    s_bad = s(mgme_idx);


                    del_bad = del(mgme_idx);

                    display(del_bad);
                    display(gm_bad);
                    display(mu_bad);
                    %display(mgme_idx)
                    %display(max_gm_mu_error);
                    %display(vel_at_error);
                    %display(comp_current_at_error);
                    %keyboard

                end 


            end

            % TODO 
            % NOTE: May want to restructure so we only both fixing the 
            % problem based on torque error 
            % the error in torque is all we care about 
            bad_idxs = unique([tau_bad_idxs; gmmu_bad_idxs]); 
            directions = driving(bad_idxs); %TODO -- think on this


        end  % end if (well, no if true) debug 
        %} 

        if obj.settings.debug 
            % TODO -- add more feautures to solution
        end 

        % RESTRUCTURE -- SHOULD ONLY NEED TO RUN THESE CHECKS WHEN 
        % omega.*compensation torque goes negative (need to go through proof)


    end 



    function [y_sol, combo_cost] = socp_solve(obj, matrices, cutoff)
    %
    %
    %
    %
    %
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


        if strcmpi(obj.settings.solver, 'gurobi')

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
            % we call regardless of solver???? 

            % TODO -- logging? 
            % TODO -- a way for users to change solver params 
            % like tolerances (--- 'Advanced Users only ----')
            params.cutoff = cutoff; 
            params.outputflag = 0;      % TODO 

            %params.FeasibilityTol = 1e-8; % Default 1e-6
            %params.OptimalityTol = 1e-9; % TODO % default 1e-6 
            %params.BarConvTol = 1e-9; % defaulat 1e-8 
            %params.BarQCPConvTol = 1e-9; % default 1e-6

            result = gurobi(model, params); 

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
                    if result.objval <= cutoff

                        warning(['Solver returned suboptimal solution',...
                             ' may need to loosen tolerances']);

                        display(result)
                        display(combo_cost)
                        keyboard
                    
                    else 
                        y_sol = nan(dim_y, 1);
                        combo_cost = inf; % returned "suboptimal" but really cutoff 
                    end 

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
function str = sciprint(num, specifier)
    fstr = sprintf(['%',specifier,'f'], num);
    estr = sprintf(['%',specifier,'e'], num);
    if length(fstr) <= length(estr) && ((num > 1e-2) || num == 0)
        str = fstr;
    else 
        str = estr; 
    end 
end 



