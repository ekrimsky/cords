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

    % ----- Problem Data -- May want more dscriptive names?
    omega
    Q
    c
    M   % can encode SOCP  
    r
    beta % beta 
    H
    b
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
    direct_drive        % dummy for no gearbox 

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
end 


methods (Access = public)

    function obj = MotorSelection(varargin) % we will parse the inputs directly 
    %**********************************************************************
    % Constructor for the class. 
    %
    % Required Inputs: 
    %  
    %
    % Optional Inputs ('name' - value pairs):
    %
    %
    % Returns:
    %    MotorSelection object
    %********************************************************************    

        % NOTE settings as an input or no? 

        % For now -- emptpy 
        % Can call the update problem later if want to instantiate with 
        % problem 



        % Assign the default motor and gear tables from some directory? 
        % https://www.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html
        % FOR NOW --- hard coding in location but when package
        % this up its fine (see line)
        % NOTE: could have default directory to look for path 
        % and the option to change that path to load in from elswhere 

        load('/home/erez/Documents/MotorSelection/database/maxon_motor_gb_data.mat',...
                    'motor_table', 'gear_table'); 

        obj.motors = motor_table;
        obj.gearboxes = gear_table;

        % Load in from database too 
        % also here will need to play around with matlab packaging 
        load('/home/erez/Documents/MotorSelection/database/test_vals.mat',...
                    'test_motor', 'test_gearbox'); 

        obj.test_motor = test_motor;
        obj.test_gearbox = test_gearbox; 


        % TODO - validation on motor/gear set 


        % Default settings -- NOTE: may want to to be able to instantiate with settings 
        %settings.solver = 'ecos' ; % TODO -- implement other solver 
        settings.solver = 'gurobi' ; % TODO -- implement other solver 

        settings.rho = 1e-6; 
        settings.verbose = 1; % 0 nothing, 1 some stuff, 2 a lot 
        settings.qcvx_reltol = 1e-3;  % bound tolerance for convergence 
        settings.qcvx_abstol = 1e-5;  
        obj.settings = settings; 

           % Creating a 'direct drive' gearbox 
        direct_drive.inertia = 0;
        direct_drive.eta = 1; 
        direct_drive.alpha = 1; 
        direct_drive.mass = 0; 
        direct_drive.Max_torque = inf; % Or some other potnetial criteria or this 
        obj.direct_drive = direct_drive; 

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
        
        % Variable input only for qcvx ojective 
 
        % bounds: ub, lb 
        % call this input - "factional"
        % Needs to have all 4 fields 
        % CHECK IF EXTRA ARG NO EMPTY

        % If quasi -- the cost objectives
        % need to be empty -- put this in as a check
        % if not quasi -- nan out the quasi terms 



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

        n = length(omega); % Need to 
        m = numel(Q) - 1; 


        % NOTE -- if its faster to have NO depedndeance on inpuits
        % might check sensitity and then use null
        % inputs to speed up alot 

        % Check that others are the right size too
        if ~isa(H, 'function_handle');  H = @(motor, gearbox) H;      end 
        if ~isa(b, 'function_handle');  b = @(motor, gearbox) b(:);   end 
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

        I_u_test = I_u(test_motor, test_gearbox);
        assert(length(I_u_test) == n, 'I_u incorrect length');
        assert(~isinf(max(I_u_test)), 'I_u must be finite'); 
        assert(min(I_u_test) > 0, 'I_u must be strictly positive'); 

        
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


            assert(size(rj, 1) == w, 'update_problem: r_%d incorrect length', j);
            assert(size(cj, 1) == n, 'update_problem: c_%d incorrect length', j);
            assert(size(rj, 2) == 1, 'update_problem: r_%d must be col vector', j);
            assert(size(cj, 2) == 1, 'update_problem: c_%d must be col vector', j);


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
        obj.b = b; 
        obj.T = T; 
        obj.d = d; 
        obj.omega = omega;
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
        if isempty(varargin) || isempty(varargin{1})
            num_return = 1;
        else 
            num_return = varargin{1};
        end 

        

        % Probably assume from setup 
        % maybe some prints to help 


        % TODO -- this will need to account for hints/rankings 
        % should incorporate hints/rankings into 1 thing


        % Get combinations from filtered list 
        combos = obj.get_combinations(); 

        % NOTE these operations might be slow 
        % could do this once in the constructor 
        motors = table2struct(obj.motors);
        gears = table2struct(obj.gearboxes);


        % update ranking somewhere 

     

        num_init_combos = size(combos, 1); 
        obj.vprintf(1, 'Found %d initial feasible combinations\n', num_init_combos);

        % Starting at end of combo list and remove based on velocty 
        max_output_vel = max(abs(obj.omega)); 

        for j = length(combos):-1:1
            motor_idx = combos(j, 1); 
            gear_idx = combos(j, 2); 
            if  gear_idx ~= 0 
                max_motor_vel = max_output_vel * gears(gear_idx).alpha;  
            else 
                max_motor_vel = max_output_vel;
            end 

            if max_motor_vel > motors(motor_idx).omega_max
                combos(j, :) = []; % clear it out 
            end 
        end 

        num_updated_combos = size(combos, 1); 
        obj.vprintf(1, 'Filtering by max velocity removed: %d combinations\n',...
                 num_init_combos - num_updated_combos);
        

        combos = combos(1:min(length(combos), 200), :); 
        warning('remember to comment this debug thing out limiting combos to 200')
    

        %%%% Inputs needed beyond this point (for if splitting )
        % 


        %
        %  in conclusion -- not hard to split into standard solve 
        %  loop and quasi/fractional solve loop 
        % 

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
                       disp_txt = sprintf('%d of %d, Best Cost: %0.4f',...
                         j, num_combinations, mincost);
                obj.vprintf(1, disp_txt);

                motor_idx = combos(j, 1);
                gear_idx = combos(j, 2); 

                % Create the combination 
                motor = motors(motor_idx); 
                if gear_idx > 0
                    gearbox = gears(gear_idx);
                else 
                    gearbox = obj.direct_drive; 
                end 

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




        % ..... these are just numbers in a list so how to link back
        % up with all acutal possible combos for next solution??
        % ....
        %
        % one way to specify a combo hint would be 
        % [motor_idx, gear_idx] in the table 
        % so, rankings will be N x 3 array
        % first col gives cost 
        % next col gives motor idx 
        % next col gives gear idx 

        % then use this to presort 

        [~, rank_idxs] = sort(cost_list);   % Need to index this back in 
        tmp_table = [cost_list, combos];
        obj.rankings = tmp_table(rank_idxs, :); 

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
            % Direct drive options 
            combo_list{end + 1, 1} = [motor_idx, 0]; % direct drive option 
            for j = 1:numel(compatible_gears)
                % TODO: if 'filtered' this combo may be excluded 
                gear_idx = compatible_gears(j); 
                combo_list{end + 1, 1} = [motor_idx, gear_idx]; 
            end 
        end 
        % convert cell to N x 2 array 
        combo_list = cell2mat(combo_list); 
        nc = length(combo_list);

        % maybe add number of distinct motors and gearboxes too 
        %if obj.settings.verbose; fprintf('%d total combinations'); end 

    end 
    


    % NOTE: some plotting methods could be good 
    % Selection Filtering?
    % Private methods? 



    % Function to CLEAR FILTERS on motor/gb -- go back to original 
    % ...this would probably clear any motor/gb that was manually added
    % no great way around this 

    % cant think of need to copy 
    %function actuatorCopy = ActCopy(obj)
    %    actuatorCopy = obj.copy();
    %end
end % end public methods 


methods (Access = private)


    function [combo_cost, sol, solve_time] = combo_solve(obj, motor, gearbox,...
                                                 bound)


        % rename cutoff to "bound" and then check problem type 
        if strcmp(obj.problem_type, 'standard')
            cutoff = bound; 
        elseif strcmp(obj.problem_type, 'fractional')
            cutoff = inf; 
        else 
            error('Invalid problem type');
        end 

        % How we actually "solve" will depend on solver
        % Gurobi OR Sedumi OR ECOS 
        % 
        % SOCP implicit for Gurobi solve (looks like a stanrd QCQP)
        omega = obj.omega; 

        H = obj.H(motor, gearbox);
        b = obj.b(motor, gearbox);

        T = obj.T(motor, gearbox);
        d = obj.d(motor, gearbox);

        eta = gearbox.eta; 
        alph = gearbox.alpha; 
        k_t = motor.k_t; 
        I_u = obj.I_u(motor, gearbox);  % symmetric bounds, dont need I_l

        n = obj.n; 
        w = obj.w;
        p = obj.p;
        m = obj.m; 

        % TODO -- this allows us to use differnt friction models 
        % note no sign flip here??????????????????
        %I_nl = obj.no_load_torque(omega);
        % TODO -- option for other modoel to not take I_nl directly 
        I_nl = motor.I_nl * ones(n, 1);

        % TODO -- decide how best to handle zero velocities 
        so = sign(omega + eps); 

        % Index Map for Optimization 
        I_start_idx = 1;
        I_end_idx = n; 

        % For code reuse, keep +/- decomp even if eta == 1
        Ip_start_idx = n + 1; 
        Ip_end_idx = 2*n; 

        In_start_idx = 2*n + 1;
        In_end_idx = 3*n; 

        if eta < 1 
            dim_y = w + 5*n;  % total dimension of opt vector 
        else 
            dim_y = 3*n + w; 
        end 

        lb = -inf(dim_y, 1); 
        ub = inf(dim_y, 1); 

        %  Ip - In = I 
        A_eq_tau1 = zeros(n, dim_y);    % sparse? 
        A_eq_tau_1(:, Ip_start_idx:Ip_end_idx) = eye(n);
        A_eq_tau_1(:, In_start_idx:In_end_idx) = -eye(n);
        A_eq_tau_1(:, I_start_idx:I_end_idx) = -eye(n);
        b_eq_tau1 = zeros(n, 1); 

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
            A_eq_tau2(:, I_start_idx:I_end_idx) = -eye(n);
            b_eq_tau2 = so .* I_nl; 

            % Tx + d = motor/gearbox torque 
            A_eq_tau_x = zeros(n, dim_y);    % sparse? 
            A_eq_tau_x(:, x_start_idx:x_end_idx) = -T; 
            A_eq_tau_x(:, gm_start_idx:gm_end_idx) = alph*eta*k_t*eye(n);
            A_eq_tau_x(:, mu_start_idx:mu_end_idx) = (alph*k_t/eta)*eye(n);
            b_eq_tau_x = d + alph*k_t*(eta + 1/eta)*so.*I_nl; 

            A_eq_tau = [A_eq_tau1; A_eq_tau2; A_eq_tau_x]; 
            b_eq_tau = [b_eq_tau1; b_eq_tau2; b_eq_tau_x]; 


            % Ineqaulity constraints containting multiple variables 
            % (if one variable, more computationally efficient to handle with bound
            % constraints )

            % sign(omega)*(gamma- mu) - I_nl - Ip - In <= 0 
            A_ineq_tau = zeros(n, dim_y); 
            A_ineq_tau(:, gm_start_idx:gm_end_idx) = diag(so);
            A_ineq_tau(:, mu_start_idx:mu_end_idx) = -diag(so);
            A_ineq_tau(:, Ip_start_idx:Ip_end_idx) = -eye(n);
            A_ineq_tau(:, In_start_idx:In_end_idx) = -eye(n); 
            b_ineq_tau = I_nl; 

            % Bounds on individual variables (ie. box constraints)
            % If sign(omega) = 1, lower bound on gm is I_nl
            % and upper bound I_u + I_nl 
            % If sign(omega) = -1, upper bound on gm is -I_nl 
            % lower bound is -I_u - I_nl 
            lb(gm_start_idx:gm_end_idx) = min(so.*I_nl,  so.*(I_nl + I_u));
            ub(gm_start_idx:gm_end_idx) = max(so.*I_nl,  so.*(I_nl + I_u));

            % If sign(omega) = 1, upper bound on mu is I_nl
            % and lower bound -I_u + I_nl 
            % If sign(omega) = -1, lower bound on mu is -I_nl 
            % upper bound is I_u - I_nl      
            % TODO -- double check 
            lb(mu_start_idx:mu_end_idx) = min(so.*I_nl,  so.*(I_nl - I_u));
            ub(mu_start_idx:mu_end_idx) = max(so.*I_nl,  so.*(I_nl - I_u));

        else   % Direct drive or fully efficient gearbox 
            x_start_idx = 3*n + 1; 
            x_end_idx = x_start_idx + w - 1; 

            A_eq_tau_x = zeros(n, dim_y);    % sparse? 
            A_eq_tau_x(:, x_start_idx:x_end_idx) = -T; 
            A_eq_tau_x(:, I_start_idx:I_end_idx) = alph*k_t*eye(n);
            b_eq_tau_x = d + alph*k_t*so.*I_nl; 

            A_eq_tau = [A_eq_tau1; A_eq_tau_x]; 
            b_eq_tau = [b_eq_tau1; b_eq_tau_x]; 

            A_ineq_tau = [];
            b_ineq_tau = []; 
        end 

        % Fill in variable bounds on I, Ip, In 
        lb(I_start_idx:I_end_idx) = -I_u; 
        ub(I_start_idx:I_end_idx) = I_u; 

        lb(Ip_start_idx:Ip_end_idx) = 0;    % non-negativity 
        ub(Ip_start_idx:Ip_end_idx) = I_u; 

        lb(In_start_idx:In_end_idx) = 0; 
        ub(In_start_idx:In_end_idx) = I_u; 


        % First index torque constraints into it
        if ~isempty(H)
            A_eq_h = zeros(size(H, 1), dim_y);
            A_eq_h(:, x_start_idx:x_end_idx) = -H;   
        else 
            A_eq_h = [];
        end

        A_eq = [A_eq_tau; A_eq_h]; 
        b_eq = [b_eq_tau; b]; 

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
            q_tmp(I_start_idx:I_end_idx) = c_tmp; 
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
            % augment H with equality 
            A_aug = zeros(2, dim_y); 
            A_aug(1, x_start_idx:x_end_idx) = obj.r_num -bound*obj.r_den;
            % Constrain denominator to be positive  
            A_aug(2, x_start_idx:x_end_idx) = -obj.r_den; % nonnegativitiy -- need to generalize 
            b_aug = [bound*obj.beta_den - obj.beta_num; obj.beta_den];
        elseif strcmp(obj.problem_type, 'standard')
            A_aug = []; 
            b_aug = []; 
        else 
            error('Invalid Problem type ')
        end 

        A_ineq = [A_ineq_tau; A_ineq_other; A_aug];
        b_ineq = [b_ineq_tau; b_ineq_other; b_aug];


        num_eq = length(b_eq);  
        num_ineq = length(b_ineq); 
        sense = [repmat('=', num_eq, 1); repmat('<', num_ineq, 1)]; 

        A_all = sparse([A_eq; A_ineq]); 
        b_all = [b_eq; b_ineq]; 


        %% ----- Incorporate Actual Cost --------------
        %Q_cost = zeros(dim_y, dim_y); % TODO -- sparse 
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
        c_cost_init(I_start_idx:I_end_idx) = c0; 
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
            params.outputflag = 0;
            result = gurobi(model, params); 
            if strcmp(result.status, 'OPTIMAL')
                y_sol = result.x; 
                % subtract off cost augmentation 
                combo_cost = result.objval - c_rho_aug'*y_sol; 
            else 
                y_sol = nan(dim_y, 1); 
                combo_cost = inf; 
            end 
        elseif strcmpi(obj.settings.solver, 'ecos') 
            result = ecos_solve(Q_cost, c_cost, beta0, A_eq, b_eq,...
                            A_ineq, b_ineq, quadcon, lb, ub,  cutoff);

            if ~isinf(result.objval)
                y_sol = result.x; 
                combo_cost = result.objval - c_rho_aug'*y_sol; 
            else
                y_sol = nan(dim_y, 1); 
                combo_cost = inf;  
            end 

        elseif strcmpi(obj.settings.solver, 'sedumi')
            error('havent writen sedumi solving yet')
            % make sure to account for bound/LP cone constraints
        else
            error('invalid solver')
        end 

        %% Parse solutons 

        sol.I = y_sol(I_start_idx:I_end_idx);
        sol.x = y_sol(x_start_idx:x_end_idx); 

        solve_time = 100; % TODO 
    end 




    function [cost_list, sol_structs] = optimize_fractional(obj, num_return, combos)
    %
    %
    %
    %
    %

        % TODO -- incorporate rankings 


        % Would be cool animation to see all the
                    % individual lower and upper bounds change 
                    % would need some logging 

        motors = table2struct(obj.motors);
        gears = table2struct(obj.gearboxes);
        mincost = inf; % no feasible solution yet 

        global_ub = obj.cost_ub;   % global upper bound for solution POOL 
        global_lb = obj.cost_lb;
        num_combinations = size(combos, 1); 

        lb_list = global_lb * ones(num_combinations, 1);  
        ub_list = global_ub * ones(num_combinations, 1); 

        % storing solutions separately so dont need to always recompute
        % if bounds already convered for a given design 
        sols = cell(num_combinations, 1); 

        max_abs_bnd_diff = inf;
        max_rel_bnd_diff = inf; 

        outer_loop_iter = 1; 
        while (max_abs_bnd_diff > obj.settings.qcvx_abstol) && ...
             (max_rel_bnd_diff > obj.settings.qcvx_reltol)

            pq = PQ(true); % max priority queue -- new every loop 

            obj.vprintf(1, '\nOuter iteration %d, Combination ', outer_loop_iter); 
            disp_txt = []; 

            for j = 1:num_combinations % Our loop thoufg   
                motor_idx = combos(j, 1);
                gear_idx = combos(j, 2); 

                % Create the combination 
                motor = motors(motor_idx); 
                if gear_idx > 0
                    gearbox = gears(gear_idx);
                else 
                    gearbox = obj.direct_drive; 
                end 

                % while not converged on this + break condition below

                % Need each upper and lower bound to approach each other every iter

                init_flag = true;  
                while ((ub_list(j) - lb_list(j)) > obj.settings.qcvx_abstol) && ...
                     ((ub_list(j) - lb_list(j)) > obj.settings.qcvx_reltol*max(min(ub_list(j), global_ub), abs(lb_list(j)))) &&...
                     (lb_list(j) <= global_ub) 
                   
                    if (outer_loop_iter == 1) && init_flag 
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
                if ub_list(j) < global_ub   % ie. this goes into sol pool
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
                        % then update the cutoff      
                        [global_ub, ~] = pq.peek(); % update the cutoff 
                    end 

                    % For the actual minimum cost 
                    if  ub_list(j) < mincost
                        mincost = bound; 
                    end                             
                end 

                if lb_list(j) > global_ub  
                    lb_list(j) = ub_list(j); % so we stop wasting time 
                end 

                % Prints 
                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
                disp_txt = sprintf(['%d of %d, LB %0.4f,',...
                                  ' Best Cost %0.4f'],...
                               j, num_combinations, global_lb, mincost);
                obj.vprintf(1, disp_txt);
            end 



            bnd_diff = ub_list - lb_list; 
            rel_bound_diff = bnd_diff./max(ub_list, abs(lb_list));
            max_abs_bnd_diff = max(bnd_diff);   % inf - inf = nan
            max_rel_bnd_diff = max(rel_bound_diff); 


            % Update global lower bounds 
            global_lb = min(lb_list); % no design can beat this 
            outer_loop_iter = outer_loop_iter + 1; 

        end 

        % TODO -- format outputs 
        num_sols = pq.size(); % may not have found num_return feas sols 
        for i = num_sols:-1:1
            [~, sol_struct] = pq.pop(); 
            sol_structs(i) = sol_struct;
        end 
        cost_list = ub_list; % NOTE: no pseudocost incorporated here 

        % ALSO -- Return some exit criteria info 
        % no feasible 
        % rel diff met 
        % abs diff met 

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
