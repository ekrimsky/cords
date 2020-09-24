classdef cords < handle 
% CORDS (Convex Optimal Robot Drive Selection)    
%      
%   CORDS is a tool for selecting motors and gearboxes for robotic applications.
%   The optimization is performed over a motor and gearbox database that needs
%   to be in the CORDS search path. 
%
%   prob = CORDS() creates a new CORDS object with default settings
%   
%   CORDS can be instantiated with a number of key value pairs for example:
%
%   prob = CORDS('settings', settings) creates a CORDS object with custom 
%   settings specified in a settings struct 
%
%   prob = CORDS('mgdb_path', 'Docuements/foo'), sets a non-default path to 
%   search for the mgdb (Motor Gearbox DataBase). 
%
%   prob = CORDS('reuse_db', false) will cause CORDS to create new MGDB from the
%   motor and gearbox files in the folder named MGDB on the search path 
%
%   prob = CORDS('reuse_db', true) will load the saved database from a previous
%   optimization run. This reorders the motor/gearbox combination using the 
%   results from the previous optimization run to improve speed.
%
%   prob = CORDS('mgdb_file_path', new_file_path) will overwrite the default 
%   location to look for motor and gearbox files to build a new database from. 
%
%   prob = CORDS('mgdb_mat_path', new_mat_path) will overwrite the default 
%   directory on where to search for a file called 'mg_database.mat' that has 
%   saved results from a previous optimization run. 
%   
%   
%   Solving a motor/gearbox optimization with CORDS looks like the following:
%    >> prob = CORDS()      % create a CORDS object with default settings 
%    >> my_data = data_struct (see  'update_problem' in the CORDS methods below)
%    >> prob.update_problem(my_data)        % add your custom data 
%    >> solutions = prob.optimize(100)      %  calculate the 100 best solutions
%
% CORDS Methods:
%       update_problem    - updates the data for the problem 
%       optimize          - runs the optimization using the loaded data 
%       update_settings   - updates what solver to use and whats printed out
%       update_physics    - updates physics settings (eg. friction and inertia)
%       update_filters    - updates additional criteria for motors gearboxes
%       update_tolerances - updates numerical tolerances in solving problems
%
% CORDS Properties:
%     settings     - see update_settings in CORDS methods      
%     physics      - see update_physics in CORDS methods 
%     filters      - see update_filters in CORDS methods 
%     tolerances   - see update_tolerances in CORDS methods
%     problem_data - struct, the data for the problem we are solving 
%     problem_type - 'standard' (convex), or 'fractional' (quasi-convex)
%     mg_database  - mgdb (Motor Gearbox DataBase) object with motor/gearbox data
%
%
%
%
%   What kind of problems can optimize solve with CORDS?
%     see our documentation but in short.....
%       see documnetion for update problelm for details on valud problem inputs 
%
%   minimize I'*Q0*I + c0'*I + x'*M0*x + r0'*x + beta0
%   subject to: 
%         T*x + tau_c = motor/gearbox output torque 
%            x_lb <= x <= x_ub
%           G*x + h = 0
%           G_ineq*x + h_ineq <= 0
%     I'*Qj*I + cj'*I + x'*Mj*x + rj'*x + betaj <= 0, for j = 1...m 
%
%   where -- the requirements on the inputs 
%
%   A reference to the github:
%   A reference to the eventual paper 
%   
%   Something about the license (GPL or MIT or Apache)
%   Copyright something something
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 7/27/20
%       Stanford University, Biomechatronics Lab 
%
%
%   Several interfaces (TODO) for CORDS simple problems inlcudeing 
%
%   See also MGDB, MIN_POWER_CONSUMPTION, MIN_MASS
properties (GetAccess = public, SetAccess = private)
    settings        %  for solver, verbose, print frequency
    tolerances  % numerical tolerances and the like  
    physics     % physics settings 
    filters         % struct of selection criteria 
    problem_data    % data for the problem we want to solve 
    problem_type    % char array, 'standard' (SOCP) or 'fractional'
    mg_database     % motor gearbox database object 
end 


methods (Access = public)

    function obj = cords(varargin) % we will parse the inputs directly 
    % Constructor for the CORDS class. 
    %
 	% 	Optional Inputs (string/value pairs)
    %       'reuse_db'      - true (default)/false
    %       'mgdb_mat_path' - where to search for mg_database.mat to load where 
    %                       to load a save mgdb object from. Also the path to 
    %                       where the mg_database.mat file frrom this 
    %                        opitimization run will be writen to
    %       'mgdb_file_path'- the folder to seach in for motor/gearbox files to 
    %                         build a new database from (is there isnt one yet)
    %       'settings'      - see update_settings
    %       'tolerances'    - see update_tolerances
    %       'physics'       - see update_physics
    %
        validate_dependencies();  % make sure everything is installed 
        % first look for a matfile called mg_database.mat that would
        % have a saved mgdb object from a previous optimization run 
        default_mat_path = fullfile('**', 'mg_database.mat');  
        default_file_path = find_dirs('MGDB');  % look for a folder called mgdb  

        ip = inputParser; 
        addParameter(ip, 'mgdb_mat_path', default_mat_path);               % Soooo, these is a difference between the path that has the files that webuild the database from AND
        addParameter(ip, 'mgdb_file_path', default_file_path{1});               % Soooo, these is a difference between the path that has the files that webuild the database from AND
        addParameter(ip, 'settings', []);                           % the path where we save 
        addParameter(ip, 'tolerances', []);                             % SO -- we will have 2 seperate things -- db_mat_path () -- and db_file_path
        addParameter(ip, 'physics', []);
        addParameter(ip, 'reuse_db', true, @(x)islogical(x));


        parse(ip, varargin{:}); 

        reuse_db = ip.Results.reuse_db; 
        db_mat_path = ip.Results.mgdb_mat_path;
        db_file_path = ip.Results.mgdb_file_path; 
        obj.settings = default_settings();
        obj.tolerances = default_tolerances();
        obj.physics = default_phyics(); 

        obj.update_settings(ip.Results.settings);
        obj.update_tolerances(ip.Results.tolerances);
        obj.update_physics(ip.Results.physics);

      
        % Look for a saved file called motor_database.mat (start looking in pwd?)
        % If cannot find a file create one and then save it
        % Database file will get overwritten after finishing optimizatino to 
        % to reflect rankings 
        old_db_files = dir(db_mat_path); 
        no_database = true; 
        if (numel(old_db_files) >= 1) && reuse_db
            if numel(old_db_files) > 1
                warning('multple motor/gearbox database files found');
            end 
            old_db_file = fullfile(old_db_files(1).folder,old_db_files(1).name); 
            try
                load(old_db_file, 'mg_database');
                obj.vprintf(1, 'Loaded database from \n\t%s\n', old_db_file);
                no_database = false; % because there is one
            catch ME
                warning(sprintf(['No mdgb object named "mg_database" in: ',...
                                                        '\n\t%s'], old_db_file));
            end 
        end 

        if no_database % no database file -- create new one and save it 
            mg_database = mgdb(db_file_path);
            db_file = fullfile(pwd, 'mg_database.mat');
            save(db_file, 'mg_database');
            obj.vprintf(1, ['New motor/gearbox database created, saved in: ',...
                                                       '\n\t%s\n'], db_file);
        end 
        
        obj.mg_database = mg_database; 
        mg_settings.verbose = obj.settings.verbose; % link verbosity
        obj.mg_database.update_settings(mg_settings); 
        obj.filters = obj.mg_database.get_filters(); % start with the defaults 
        obj.problem_type = []; % none yet 
    end %end constructor 

    

    function update_problem(obj, input_data)
    % update_problem update the problem data
    %
    %   There are 2 main types of problems that can be solved with cords:
    %       1) Second Order Cone Programs (SOCPs)
    %       2) Linear Fractional Programs with SOC constraints
    %
    %   See examples/example2 for an example of a linear fractional 
    %   problem to maximize a robot's runtime.
    %
    %
    %   For type 1 (SOCP) problems, cords solves
    %               minimize    I'*Q0*I + c0'*I + x'*M0*x + r0'*x + beta0
    %
    %   For type 2 (linear-fractional) problems, cords solves
    %             minimize    (r_num'*x + beta_num)/(r_den'*x + beta_den)
    %
    %   subject to (for both problem types): 
    %                   T*x + tau_c = motor/gearbox output torque 
    %                            G*x + h = 0
    %                 G_ineq*x + h_ineq <= 0
    %     I'*Qj*I + cj'*I + x'*Mj*x + rj'*x + betaj <= 0, for j = 1...m 
    %                         x_lb <= x <= x_ub
    % 
    %   where I is an (n x 1) vector of motor currents and x is a (w x 1)
    %   vector of user defined optimization variables. 
    %
    %   Inputs that rely on the motor/gearbox are given as function handle that
    %   take the motor and gearbox as inputs. See MGDB for a list of valid
    %   motor and gearbox properties
    %   For example:
    %        T = @(motor, gearbox) motor.mass * ones(n, w); % depends on motor 
    %   If the input does not depend on the motor and gearbox it can be given
    %   as a constant (eg. T = eye(n));
    %       
    %   The struct 'input_data' has the following fields:
    %     
    %   Required fields for either problem type:
    %       T     - (n x w) matrix relating x variables to torques 
    %       tau_c - (n x 1) vector of 'constant' torques  
    %       omega -  (n x 1) vector of the joint velocity (NOTE: if using a 
    %               gearbox this is not the same as the motor velocity) in rad/s
    %       I_max - maximum permissible current (in Amps)
    %       V_max - maximum available drive voltage
    %
    %   Optional fields optional input for either problem type:
    %       omega_dot - (n x 1) vector of joint accelerations (rad/s^2). If 
    %                       empty no inertial compenstation is performed  
    %       Q         - (m x 1) CELL array where each cell contains an (n x 1)
    %                    vector of non-negative values on the diagonal of Q. 
    %       c         - (m x 1) CELL array where each cell contains an (n x 1)
    %                    vector satisfying (gearbox.direction)*c.*omega >= 0
    %       M         - (m x 1) CELL array where each cell contains a symmetric 
    %                   sparse (w x w) matrix. Each M{i} (for i = 1:m) can take
    %                   on 3 possible forms:
    %                   1) Standard Quadratic - in this case M{i} is PSD 
    %                   2) Standard Second Order Cone - in this case M{i} 
    %                      is made up of a PSD block and a -1 on its diagonal.
    %                      The corresponding Q{i}, c{i}, r{i} and beta{i} must 
    %                      all be empty. 
    %                      For example:
    %                           M{i} = [a   0   0
    %                                   0   b   0
    %                                   0   0  -1],
    %                      encodes a*x1^2 + b*x2^2 <= x3^2 (an SOC Constraint),
    %                      where x3 is implicitly constrained to be non-negative
    %                   3) Rotated Second Order Cone - in this case M{i} is made 
    %                      up of a PSD block and a row column/pair that each 
    %                      contain one non-zero on an off diagonal.
    %                      The corresponding Q{i}, c{i}, r{i} and beta{i} must 
    %                      all be empty. 
    %                      For example:
    %                          M{i} = [a    0      0
    %                                  0    0   -0.5
    %                                  0  -0.5     0],
    %                      encodes a*x1^2 <= x2*x3
    %                      where x2 and x3 are implicitly constrained to be 
    %                      non-negative
    %       r         - (m x 1) CELL array, each cell contains a (w x 1) vector  
    %       beta      - (m x 1) CELL array, each cell contains one scalar       
    %       x_lb      - (w x 1) vector of lower bounds on x (can be -inf) 
    %       x_ub      - (w x 1) vector of upper bounds on x (can be inf)
    %       G         - (? x w) equality constraint matrix on x 
    %       h         - (? x 1) vector for equality constraints on x 
    %       G_ineq    - (? x w) inequality constraint matrix on x 
    %       h_ineq    - (? x 1) vector for inequality constraints on x
    %       
    %
    %
    %   Required fields for SOCP problems (NOTE, each can be empty):
    %       Q0    -  (n x 1) vector of non-negative values on the diagonal of Q0       
    %       c0    -  (n x 1) vector satisfying (gearbox.direction)*c.*omega >= 0
    %       M0    -  sparse PSD (w x w) matrix 
    %       r0    -  (w x 1) vector 
    %       beta0 -  constant cost scalar 
    %
    %
    %   For linear-fractional problems, the fields Q0, c0, M0, r0, beta0 should
    %   all be left empty. 
    %   
    %   Required fields for linear fractional problems:
    %       cost_ub  - a finite upper bound on the optimal objective
    %       cost_lb  - a finite lower bound on the optimal objective
    %       r_num    - (w x 1) vector 
    %       r_den    - (w x 1) vector 
    %       beta_num - a scalar 
    %       beta_den - a scalar 
    %
    %
    %   See the example1 code for an example on defining function handles 
    %
        [problem_data, problem_type] = obj.validate_problem_data(input_data);
        obj.problem_data = problem_data; 
        obj.problem_type = problem_type; 
    end 

    
    function [sol_structs] = optimize(obj, varargin)
    %   Runs the actual optimization
    %
    %   optimize()      - will return only the best solution (fastest)
    %   optimize(k)     - will return the k-best solutions
    %   optimize(inf)   - will return ALL feasible solutions
    %
    %   Returns:
    %       sol_structs - a (k x 1) struct array with fields:
    %           'cost' - the objective
    %           'sol' - a struct with fields I and x (current and x vector) 
    %                   among others
    %           'motor' - the motors
    %           'gearbox' - the gearboxes
        sol_structs = struct([]); % empty 

        if isempty(obj.problem_type)
            warning('Cannot optimize before adding problem data');
            return;
        end
        
        if isempty(varargin) || isempty(varargin{1})
            num_return = 1;
        else 
            num_return = varargin{1};
        end 

        %
        %       Get Valod Motor/Gearbox Combinations 
        % Filtering out some options by max velocity  
        % If our specific filters are tighter than the others, use those 
        obj.filters.omega_max = max(obj.filters.omega_max,...
                                              max(abs(obj.problem_data.omega)));
        %% get bounds on torque 
        tmp_filters = obj.filters;

        [tau_peak_min, tau_rms_min] = obj.compute_torque_bounds();
        
        % NOTE: will want to add a way to turn these off -- could 
        % have presetting a filter as negative inf as flag for that 
        tmp_filters.tau_max = max(tmp_filters.tau_max, tau_peak_min);
        tmp_filters.tau_rms = max(tmp_filters.tau_rms, tau_rms_min);
        obj.mg_database.update_filters(tmp_filters);

        [motor_keys, gearbox_keys] = obj.mg_database.get_combinations();

        % Convert from database map keys to structs 
        for i = 1:length(motor_keys)  % slightly faster than cell fun
            motors(i) = obj.mg_database.motors(motor_keys{i});
            gearboxes(i) = obj.mg_database.gearboxes(gearbox_keys{i});
        end 
    
        %       
        %          Call the Optimizer for the Given Problem Type  
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
            warning('No feasible solutions found');
        end 

        % Update the rankings using the cost list to speed up the next solve 

        obj.mg_database.update_rankings(motor_keys, gearbox_keys, cost_list);
        mg_database = obj.mg_database;
        db_file = fullfile(pwd, 'mg_database.mat');
        save(db_file, 'mg_database');
        obj.vprintf(1, '\nSaved motor/gearbox database:\n\t\n%s updated\n', db_file);

    end % end optimize 


    function update_settings(obj, new_settings)
    %   updates the settings for the problem. 
    %
    %   new_settings - a struct with (any) of the following fields:
    %       solver - which solver to use, 'gurobi'(default, fastest) or 'ecos'
    %       verbose - how much to print
    %           verbose = 1 -- default
    %           verbose = 2 -- use for debugging will output data on every solve
    %           verbose = 0 -- supresses all outputs (except warnings)
    %       print_freq - how often to update prints on screen (default 1)
    %       line_freq - how often to print new line (default 500)
    %
    %
        if isempty(new_settings)
            return;
        end 
        settings = obj.settings; % the current settings 
        % Loop through the fields
        fn = fieldnames(new_settings);
        for ii = 1:length(fn)
            field = fn{ii};
            if ~isfield(settings, field)
                error('update_settings: invalid field %s', field);
            end 
        end 
        valid_solvers = {'gurobi', 'ecos'};
        has_gurobi = ~isempty(which('gurobi.m'));
        has_ecos = ~isempty(which('ecos.m'));
        if ~any([has_gurobi, has_ecos])
            error('validate_settings: no valid solvers found in path');
        end 
        if isfield(new_settings, 'solver')
            settings.solver = new_settings.solver; 
        end
        if strcmpi(settings.solver, 'gurobi')
            if ~has_gurobi
                if has_ecos
                    has_ecos; settings.solver = 'ecos'; 
                end 
                warning('Gurobi solver not found, using %s', settings.solver);                
            end 
        end 
        if strcmpi(settings.solver, 'ecos')
            if ~has_ecos
                if has_gurobi
                    settings.solver = 'gurobi'; 
                end 
                warning('ECOS solver not found, using %s', settings.solver);
            end 
        end 
        if isfield(new_settings, 'verbose')
            settings.verbose = new_settings.verbose;         
        end
        if isfield(new_settings, 'print_freq')
            settings.print_freq = new_settings.print_freq; 
        end
        if isfield(new_settings, 'line_freq')
            settings.line_freq = new_settings.line_freq; 
        end
       

    end 



    function update_physics(obj, new_physics)
    %   updates physics used in solving 
    %
    %   new_physics - a struct with (any) of the following fields:
    %       inertia - true/false, do inertial compensation (default true)
    %       f_damping - true/false, use viscous damping when specified in 
    %                        motor.viscous_friction OR for any BLDC (default true)
    %       f_couloumb - true/false, use coulomb friction when specified in 
    %                       motor.coulomb_fricrion OR for any DC motor 
    %       f_static - true/false use static friction bounds whenever omega = 0
    %                   (default false). NOTE: turning static friction on can
    %                   lead to numerical issues and is not physically accurate
    %                   for many scenarios 
    %
        if isempty(new_physics)
            return;
        end 
        np = new_physics;
        ip = obj.physics; % initial physics 
        if isfield(np, 'inertia'); ip.inertia = np.inertia; end
        if isfield(np, 'f_damping'); ip.f_damping = np.f_damping; end 
        if isfield(np, 'f_coulomb'); ip.f_coulomb = np.f_coulomb; end 
        if isfield(np, 'f_static'); ip.f_static = np.f_static; end 
        obj.physics = ip; 
    end 

    function update_filters(obj, new_filters)
    %   updates selection filters for which motor/gearbox combinations to 
    %   consider in optimization.
    % 
    %   See 'update filters' in MGDB for usage
    %
        obj.mg_database.update_filters(new_filters); % hashtag YES filter
    end 


    function update_tolerances(obj, new_tolerances)
    %   updates numerical tolerances. For ADVANCED USERS ONLY. Tuning
    %   may help if numerical issues aries in solving. 
    %
    %   new_tolerances - a struct with (any) of the following fields:
    %       rho - multiplier for l1 regularizer on gamma/mu decomposition
    %             default (1e-4)
    %       gmmu_tol - tolerance on acceptable gamma/mu decompostion in Amps.
    %              default 1e-3 (ie. 1 mA)
    %       feastol - feasibility tolerance in SOCP solve (default 1e-5)
    %       reltol - relative tolerance for accepting repeated solutions 
    %               default 1e-2 (corresponding to 1% optimality)
    %       abstol - absolute tolerance on solution (default 1e-3)
    %       qcvx_reltol - relative tolerance for convergence of linear 
    %                    fractional problems (default 1e-4)
    %       qcvx_abstol - absolute toleranec for convergence of linear 
    %                     fractional problems (default 1e-6)
    %
        if isempty(new_tolerances)
            return;
        end 
        nss = new_tolerances; % jnew solve settings
        iss = obj.tolerances;  % initial  
        % TODO -- add validation on ranges 
        if isfield(nss, 'rho'); iss.rho = nss.rho; end 
        if isfield(nss, 'gmmu_tol'); iss.gmmu_tol = nss.gmmu_tol; end 
        if isfield(nss, 'feastol'); iss.feastol = nss.feastol; end 
        if isfield(nss, 'reltol'); iss.reltol = nss.reltol; end 
        if isfield(nss, 'abstol'); iss.abstol = nss.abstol; end 
        if isfield(nss, 'qcvx_reltol'); iss.qcvx_reltol = nss.qcvx_reltol; end 
        if isfield(nss, 'qcvx_abstol'); iss.qcvx_abstol = nss.qcvx_abstol; end
        obj.tolerances = iss; 
    end 


end % end public methods 



methods (Access = private)

    function [problem_data, problem_type] = ...
                                        validate_problem_data(obj, input_data)
    %
    %
    %
    %
    %

        [test_motor, test_gearbox] = test_motor_gearbox(); 


        % These need to be present no matter what 
        assert(isfield(input_data, 'omega'), 'Missing omega');
        assert(isfield(input_data, 'T'), 'Missing T');
        assert(isfield(input_data, 'tau_c'), 'Missing tau_c');

        assert(isfield(input_data, 'I_max'), 'Missing I_max');
        assert(isfield(input_data, 'V_max'), 'Missing V_max');
        assert(isnumeric(input_data.I_max) && input_data.I_max > 0,...
                                                     'I_max must be positive');
        assert(isnumeric(input_data.V_max) && input_data.V_max > 0,...
                                                     'V_max must be positive');

        omega = input_data.omega(:);
        T = input_data.T; 
        tau_c = input_data.tau_c; 
        I_max = input_data.I_max;
        V_max = input_data.V_max; 
        n = length(omega);  



        if ~isa(T, 'function_handle');  T = @(motor, gearbox) T;      end 
        if ~isa(tau_c, 'function_handle');  tau_c = @(motor, gearbox) tau_c(:);  end 
        T_test = T(test_motor, test_gearbox);
        w = size(T_test, 2); 
        tau_c_test = tau_c(test_motor, test_gearbox); 
        assert(size(T_test, 1) == n, 'Incorrect number of rows in T');
        assert(numel(tau_c_test) == n || isempty(tau_c_test), 'Incorrect d length');

        if isfield(input_data, 'Q') || isfield(input_data, 'c') || ...
            isfield(input_data, 'M') || isfield(input_data, 'r') || ...
            isfield(input_data, 'b')

            % if specified any of these, need to specify all of them 
            assert(isfield(input_data, 'Q'), 'Missing Q');
            assert(isfield(input_data, 'c'), 'Missing c');
            assert(isfield(input_data, 'M'), 'Missing M');
            assert(isfield(input_data, 'r'), 'Missing r');
            assert(isfield(input_data, 'beta'), 'Missing beta');

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

        Q_empty = zeros(n, 1);  % the diagonal 
        M_empty = sparse(w, w); 
        c_empty = zeros(n, 1);    
        r_empty = zeros(w, 1);   



        m = numel(Q); 


        %
        %
        %         Validating inputs for linear fractional problems 
        %
        %
        if (isfield(input_data, 'cost_ub') && ~isempty(input_data.cost_ub)) ||...
           (isfield(input_data, 'cost_lb') && ~isempty(input_data.cost_lb)) ||...
           (isfield(input_data, 'r_num') && ~isempty(input_data.r_num)) ||...
           (isfield(input_data, 'r_den') && ~isempty(input_data.r_den))  || ...
           (isfield(input_data, 'beta_num') && ~isempty(input_data.beta_num)) ||...
           (isfield(input_data, 'beta_den') && ~isempty(input_data.beta_den)) 


           % If it has any one of those field and no empty 
           % need to ensure it has ALL of them 
            assert(isfield(input_data, 'cost_ub') &&...
                 ~isempty(input_data.cost_ub),...
                 'Cost upper bound required for fractional problems'); 
            assert(isfield(input_data, 'cost_lb') &&...
                 ~isempty(input_data.cost_lb),...
                 'Cost lower bound required for fractional problems'); 
            assert(isfield(input_data, 'r_num') &&...
                 ~isempty(input_data.r_den),...
                 'Numerator vector required for fractional problems'); 
            assert(isfield(input_data, 'r_den') &&...
                 ~isempty(input_data.r_num),...
                 'Denominator vector required for fractional problems'); 
            assert(isfield(input_data, 'beta_num') &&...
                 ~isempty(input_data.beta_num),...
                 'Numerator offset required for fractional problems'); 
            assert(isfield(input_data, 'beta_den') &&...
                 ~isempty(input_data.beta_den),...
                 'Denominator offset required for fractional problems'); 

            assert(isnumeric(input_data.cost_ub), 'Costs must be numeric');
            assert(isnumeric(input_data.cost_lb), 'Costs must be numeric');
            assert(input_data.cost_ub - input_data.cost_lb > 0,...
                    'Cost upper bound must be greater than lower bound'); 

            % make col vec regardless of input 
            input_data.r_num = input_data.r_num(:); 
            input_data.r_den = input_data.r_den(:); 

            % Size + Range Check on inputs 
            assert(isscalar(input_data.cost_ub), 'Cost upper bound must be scalar');
            assert(~isinf(input_data.cost_ub), 'Cost upper bound must be finite');
            assert(isscalar(input_data.cost_lb), 'Cost lower bound must be scalar');
            assert(~isinf(input_data.cost_lb), 'Cost lower bound must be finite');
            assert(isscalar(input_data.beta_num), 'beta_num must be scalar');
            assert(~isinf(input_data.beta_num), 'beta_num must be finite');
            assert(isscalar(input_data.beta_den), 'beta_den must be scalar');
            assert(~isinf(input_data.beta_den), 'beta_den must be finite');
            assert(length(input_data.r_num) == w, 'r_num must be length %d', w);
            assert(length(input_data.r_den) == w, 'r_den must be length %d', w);

            % If linear fractional - cost terms must all be zero/empty 
            error_txt = 'Cost inputs must be zero or empty for fractional problem';


            assert(~isfield(input_data, 'Q0') || isempty(input_data.Q0),...
                                                                     error_txt);
            assert(~isfield(input_data, 'c0') || isempty(input_data.c0),...
                                                                     error_txt);
            assert(~isfield(input_data, 'M0') || isempty(input_data.M0),...
                                                                     error_txt);
            assert(~isfield(input_data, 'r0') || isempty(input_data.r0),...
                                                                     error_txt);
            assert(~isfield(input_data, 'beta0') || isempty(input_data.beta0),...
                                                                     error_txt);
            problem_type = 'fractional'; 


            Q0 = @(~, ~) Q_empty;
            c0 = c_empty;
            M0 = @(~, ~) M_empty;
            r0 = r_empty;
            beta0 = 0;


        else 
            problem_type = 'standard'; 

            assert(isfield(input_data, 'Q0'), 'Missing Q0');
            assert(isfield(input_data, 'c0'), 'Missing c0');
            assert(isfield(input_data, 'M0'), 'Missing M0');
            assert(isfield(input_data, 'r0'), 'Missing r0');
            assert(isfield(input_data, 'beta0'), 'Missing beta0');

            Q0 = input_data.Q0;
            c0 = input_data.c0; 
            M0 = input_data.M0; 
            r0 = input_data.r0;
            beta0 = input_data.beta0; 

                    %%
            %
            %        validate objective terms 
            %
            %
            if ~isa(Q0, 'function_handle'); Q0 = @(~, ~) Q0; end 
            if ~isa(M0, 'function_handle'); M0 = @(~, ~) M0; end 
            Q0_test = Q0(test_motor, test_gearbox);
            if isa(c0, 'function_handle')
                c0_test = c0(test_motor, test_gearbox);
            else 
                c0_test = c0; 
            end 
            M0_test = M0(test_motor, test_gearbox);
            if isa(r0, 'function_handle')
                r0_test = r0(test_motor, test_gearbox);
            else 
                r0_test = r0; 
            end 
            if isa(beta0, 'function_handle')
                beta0_test = beta0(test_motor, test_gearbox);
            else 
                beta0_test = beta0; 
            end 
            % TODO -- if not empty need to validate all
            % a little cumbersome but writing 'validate functions' could make this section of code much more managable
            % the validate function would return an appropriately sized empty term if emtpy and the other valudation
            % as neccearyy -- then would need nearly as much repeated code 
            if isempty(Q0_test)     
                Q0 = @(~, ~) Q_empty; 
            end % else validate 
            if isempty(c0_test)
                c0 = c_empty; 
                c0_test = c_empty;
            end 
            if isempty(M0_test)
                M0 = @(~, ~) M_empty;
            end 
            if isempty(r0_test)
                r0 = r_empty;
                r0_test = r_empty;
            end 
            if isempty(beta0)
                beta0 = 0;
            end

            validate_c(c0_test, omega, n, 0);
            validate_r(r0_test, w, 0);
        end 


        % Get dummy motor and dummy gearbox for input validation 





        % Optional Inputs G, h, G_ineq, h_ineq
        % and Gx + h = 0
        % where G_ineq x + h_ineq <=  0
        G = [];
        h = [];
        if isfield(input_data, 'G') || isfield(input_data, 'h') 
            assert(isfield(input_data, 'G'),...
                                          'Need to supply G AND h');
            assert(isfield(input_data, 'h'),...
                                          'Need to supply G AND h');
            G = input_data.G;
            h = input_data.h; 
        end 

        G_ineq = [];    
        h_ineq = [];
        if isfield(input_data, 'G_ineq') || isfield(input_data, 'h_ineq') 
            assert(isfield(input_data, 'G_ineq'),...
                                          'Need to supply G_ineq AND h_ineq');
            assert(isfield(input_data, 'h_ineq'),...
                                          'Need to supply G_ineq AND h_ineq');
            G_ineq = input_data.G_ineq;
            h_ineq = input_data.h_ineq; 
        end 

        % Direct bounds on x-variable 
        x_lb = -inf(w, 1);
        if isfield(input_data, 'x_lb'); x_lb = input_data.x_lb; end 
        x_ub = inf(w, 1); 
        if isfield(input_data, 'x_ub'); x_ub = input_data.x_ub; end 


        % NOTE -- if its faster to have NO depedndeance on inpuits
        % might check sensitity and then use null
        % inputs to speed up alot 

        % Check that others are the right size too
        if ~isa(G, 'function_handle');  G = @(motor, gearbox) G;      end 
        if ~isa(G_ineq, 'function_handle')
                        G_ineq = @(motor, gearbox) G_ineq;            end 
        if ~isa(h, 'function_handle');  h = @(motor, gearbox) h(:);   end 
        if ~isa(h_ineq, 'function_handle')
                        h_ineq = @(motor, gearbox) h_ineq(:);         end 
        if ~isa(x_ub, 'function_handle');  x_ub = @(motor, gearbox) x_ub(:); end 
        if ~isa(x_lb, 'function_handle');  x_lb = @(motor, gearbox) x_lb(:); end 

        

        % Validate Remainin Inputs (G, h, G_ineq, h_ineq, x_lb, x_ub)
        G_test = G(test_motor, test_gearbox); 
        h_test = h(test_motor, test_gearbox); 
        assert(size(G_test, 2) == w || isempty(G_test),...
                                         'Incorrect number of columns in H');
        assert(size(G_test, 1) == length(h_test), 'Size G and h incompatible')
        p = length(h_test); 

        G_ineq_test = G_ineq(test_motor, test_gearbox); 
        h_ineq_test = h_ineq(test_motor, test_gearbox); 
        assert(size(G_ineq_test, 2) == w || isempty(G_ineq_test),...
                            'Incorrect number of columns in G_ineq');
        assert(size(G_ineq_test, 1) == length(h_ineq_test),...
                         'Size G_ineq and h_ineq incompatible')
        x_lb_test = x_lb(test_motor, test_gearbox);
        x_ub_test = x_ub(test_motor, test_gearbox);
        assert(length(x_lb_test) == w, 'Incorrect x lower bound size');
        assert(length(x_ub_test) == w, 'Incorrect x upper bound size');
        assert(all(x_ub_test >= x_lb_test), ['Upper bounds must be ',...
                                        'greater than lower bounds']);

  

        %% Check for optional inputs -- Acceleration 
        nd = 8; % number of decimals to keep on omega, omega_dot
        if isfield(input_data, 'omega_dot') && (obj.physics.inertia)
            omega_dot = input_data.omega_dot(:); 
            assert(length(omega_dot) == n,...
                    'omega and omega_dot must be same size'); 
            omega_dot = round(omega_dot, nd);
        else 
            omega_dot = zeros(n, 1); % treat as zero
        end 






        % Same for the rest -- switch to other seperate objective/contraints 
        %{
        for j = 1:m + 1
            % Check if input is function handle, if not make it so 
            % NOTE: may want to change these too 
            if ~isa(Q{j}, 'function_handle'); Q{j} = @(~, ~) Q{j}; end 
            if ~isa(M{j}, 'function_handle'); M{j} = @(~, ~) M{j}; end 
        end 
        %} 
        for j = 1:m  % TODO -- dont force to be function handle (like r and c)
            % Check if input is function handle, if not make it so 
            % NOTE: may want to change these too 
            if ~isa(Q{j}, 'function_handle'); Q{j} = @(~, ~) Q{j}; end 
            if ~isa(M{j}, 'function_handle'); M{j} = @(~, ~) M{j}; end 
        end 

        % when f_j is encoding linear inequality -- its more  
        % efficient to explicitly encode these as linear constrainrts
        % intead of SOC constraints with empty matrices 
        % We will convert Q to vectors containing the diagonal 



        % Letting c,r,beta be fixed instead of function handles can 
        % greatly speed up code 
        %for j = 1:m + 1   % NOTE: swticehd for seperate -- need to still validate objective though 
        lin_ineq = zeros(m, 1);   % Indicator (1 = linear)
        
        for j = 1:m 

            Qj = Q{j}(test_motor, test_gearbox); % to test validity 
            Mj = M{j}(test_motor, test_gearbox); % to test validity 

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

            validate_c(cj, omega, n, j)
            validate_r(rj, w, j);

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

            %if (nnz(Mj) == 0) && (j > 1)  % because using linear on Q   
            if (nnz(Mj) == 0)  % because using linear on Q  -- because split up objective/constraints
                %lin_ineq(j - 1) = 1; 
                lin_ineq(j) = 1; 
            end

            if nnz(Mj) > 0
                validate_soc(Mj); 
            end 
        end 

       


        %
        %
        %    Now all inputs validated  :) 
        %
        %
        if strcmp(problem_type, 'fractional')
            problem_data.cost_ub = input_data.cost_ub;
            problem_data.cost_lb = input_data.cost_lb;
            problem_data.r_num = input_data.r_num;
            problem_data.r_den = input_data.r_den; 
            problem_data.beta_num = input_data.beta_num;
            problem_data.beta_den = input_data.beta_den; 
        end 

        problem_data.Q0 = Q0;
        problem_data.r0 = r0;
        problem_data.M0 = M0;
        problem_data.c0 = c0; 
        problem_data.beta0 = beta0; 

        problem_data.T = T; 
        problem_data.tau_c = tau_c; 
        problem_data.omega = omega;
        problem_data.omega_dot = omega_dot; 

        problem_data.Q = Q;   % etc 
        problem_data.c = c; 
        problem_data.M = M; 
        problem_data.r = r; 
        problem_data.beta = bet; 


        problem_data.G = G; 
        problem_data.G_ineq = G_ineq; 
        problem_data.h = h; 
        problem_data.h_ineq = h_ineq; 
        problem_data.x_lb = x_lb; 
        problem_data.x_ub = x_ub; 


        problem_data.zero_vel_idxs = find(omega == 0); 
        problem_data.lin_ineq = lin_ineq; 
        problem_data.I_max = I_max; 
        problem_data.V_max = V_max;

        % add problem_data.objective for parsing and assigning outputs???? 
        % not a bad idea....

        % update relevant dimensions 
        problem_data.n = n;
        problem_data.w = w;
        problem_data.m = m;
        problem_data.p = p; 


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
                ((mod(j - 1, obj.settings.line_freq) == 0) && j > 1)
                % Print a new line 
                obj.vprintf(1, '\n'); 
            else 
                % update the current line (by first removing)
                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
            end 
            disp_txt = sprintf('%13d%12d%12d%13.3e%13.3e%11.1f',...
                                    j, num_rep_solves, num_mi_solves,...
                                            cutoff, mincost, toc(start_tic)); 
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
        while (max_abs_bnd_diff > obj.tolerances.qcvx_abstol) && ...
              (max_rel_bnd_diff > obj.tolerances.qcvx_reltol) && ...
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
                      &&  (rel_diff_j > obj.tolerances.qcvx_reltol)  && ...        
                          (abs_diff_j > obj.tolerances.qcvx_abstol) || init_flag 

                   
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

                    %
                    %        Solve 
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

                            if (cost_diff < obj.tolerances.qcvx_abstol) || ...
                                    (cost_diff < obj.tolerances.qcvx_reltol*...
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


                obj.vprintf(1, repmat('\b', 1, length(disp_txt))); 
                disp_txt = sprintf('%6d%12d%13.3e%13.3e%13.3e%8.4f%7.1f',...
                        outer_loop_iter, j, global_lb, global_ub, mincost,... 
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
            disp_txt = sprintf('%6d%12d%13.3e%13.3e%13.3e%8.4f%7.1f',...
                        outer_loop_iter, j, global_lb, global_ub, mincost,...
                                            max_rel_bnd_diff, toc(start_tic));                
            obj.vprintf(1, disp_txt);

            % If no solution found at end of first outer loop, break 
            if (outer_loop_iter == 1) && isinf(mincost)
                obj.vprintf(1, ['\nNo solutions found.',...
                                            ' Try increasing the upper bound']);
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
        
        if (max_abs_bnd_diff < obj.tolerances.qcvx_abstol) 
            exitflag = 1; 
        elseif (max_rel_bnd_diff < obj.tolerances.qcvx_reltol) 
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
                                                        motor, gearbox, bound)
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

        % TODO -- add small coefficient check in build matrices
        % or round vels and other inputs when put it --- this is a better idea 
        %if strcmp(obj.settings.solver, 'gurobi')
        %    matrices = presolve(matrices, false(length(matrices.lb), 1));
        %end

        % TODO -- put cutoffs back in  -- debugging like this 
        % because code needs to work without cutoffs 
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
            
            if ~isempty(bad_idxs_clean) || isnan(y_sol(1))
                obj.mixed_int_solve(matrices, index_map.del, cutoff);
                exitflag = 2; 
            end

        end 
        solve_time = toc(start_tic);
    end 




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
            assert(~isinf(bound),'Bound must be finite for fractional problems');
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

        G = pd.G(motor, gearbox);
        h = pd.h(motor, gearbox);
        G_ineq = pd.G_ineq(motor, gearbox);
        h_ineq = pd.h_ineq(motor, gearbox); 
         % TODO -- remove confusion between A/b in solving socp and H/b in problem setuo
         % probablyt keep A/b and rename the 'b' in the problem setup context 

        T = pd.T(motor, gearbox);
        tau_c = pd.tau_c(motor, gearbox); 

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

        % Index Map for Optimization 
        I_idxs = 1:n;
        Isq_idxs = I_idxs(end) + (1:n);	 % equivalent to I squared 
        tau_idxs = Isq_idxs(end) + (1:n); 

        % potenial condition this on the physics too 
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
            %   Rows 1:n        Tx + tau_c = tau_out  (delivered to robot)
            %   Rows n+1:2n     tau_out + tau_gb_inertia = motor/gb torque  
			%
            % sign of motor/gb torque term determines driving/driven 		
            num_vals = nnz(T) + 4*n + nzvi; 
            A_eq_tau_x = sparse([], [], [], 2*n, dim_y, num_vals);
            b_eq_tau_x = zeros(2*n, 1);     % to be filled in 

            A_eq_tau_x(1:n, tau_idxs) = eye(n);
            A_eq_tau_x(1:n, x_idxs) = -T; 
            b_eq_tau_x(1:n) = tau_c; 

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
            b_eq_tau_x(1:n) = tau_c; 

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

        lb(x_idxs) = pd.x_lb(motor, gearbox);
        ub(x_idxs) = pd.x_ub(motor, gearbox);


        %
        %
        %		Account for Static Friction Cone
        %		for points with zero velocity
        %
        %		Adjust A_eq_tau_x and b_eq_tau_x accordingly
        %
        if (nzvi > 0) && (tau_static_friction > 0)
        	% think through a little more (but then just ratio on next line)
            % add the static frinction value (to be computed) into the torque equlity 
            % for the motor itslef
            A_eq_tau_x(n + zero_vel_idxs, sf_idxs) = ratio*eye(nzvi); 

            % Overwrite the other zero vel points, treat gb as fully eff 
            if eta < 1
            	% Replace ratio*kt*eta with ratio*kt on those rows 
            	A_eq_tau_x(n + zero_vel_idxs, gm_idxs(1) + zero_vel_idxs - 1)...
                                                         = ratio*k_t*eye(nzvi);
            	A_eq_tau_x(n + zero_vel_idxs, mu_idxs) = 0;  % dont consider it driven at this step
            end 

            % Static Friction Bounded 
            ub(sf_idxs) = tau_static_friction;
            lb(sf_idxs) = -tau_static_friction; 
        end 

        if ~isempty(G)
            A_eq_G = sparse([], [], [], size(G, 1), dim_y, nnz(G));
            A_eq_G(:, x_idxs) = -G;   % note signs
            b_eq_G = h; 
        else 
            A_eq_G = [];
            b_eq_G = [];
        end

        % very slow 
        A_eq = [A_eq_tau_x; A_eq_other; A_eq_G]; 
        b_eq = [b_eq_tau_x; b_eq_other; b_eq_G]; % NOTE signs 

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

        Q = pd.Q; 
        % conveninet but actually slower than writing loop 
        %Q_tmp = cellfun(@(x) x(motor, gearbox), Q, 'UniformOutput', false); 

        for j = 1:m    % incorporate the quadratic constraints + other linear ineqs 
            %cj = pd.c{j + 1};  % +1 bc 1 indexing 
            cj = pd.c{j};  % +1 bc 1 indexing 

            if isa(cj, 'function_handle')
                c_tmp = cj(motor, gearbox); % slow 
            else 
                c_tmp = cj;
            end 

            %rj = pd.r{j + 1};   % WHY SO SLOW ? - NOT EMPTY       
            rj = pd.r{j};   % WHY SO SLOW ? - NOT EMPTY             

            if isa(rj, 'function_handle')
                r_tmp = rj(motor, gearbox); % slow 
            else 
                r_tmp = rj; 
            end 
            
            %bet = pd.beta{j + 1}; 
            bet = pd.beta{j}; 

            if isa(bet, 'function_handle')
                beta_tmp = bet(motor, gearbox); % slow 
            else 
                beta_tmp = bet; 
            end 

            q_tmp = zeros(dim_y, 1); 
            q_tmp(I_idxs) = c_tmp; 
            %q_tmp(Isq_idxs) =  pd.Q{j + 1}(motor, gearbox); 
            q_tmp(Isq_idxs) =  Q{j}(motor, gearbox);  % slow
            %q_tmp(Isq_idxs) =  Q_tmp{j};  % slow


            q_tmp(x_idxs) = r_tmp; 

            if pd.lin_ineq(j)  % if linear 
                lin_ineq_idx = lin_ineq_idx + 1; 
                A_ineq_extra(lin_ineq_idx, :) = q_tmp'; 
                b_ineq_extra(lin_ineq_idx) = -beta_tmp;
            else  % quad constraints 
                qc_idx = qc_idx + 1; 

                %Mj_tmp = pd.M{j + 1}(motor, gearbox); 
                Mj_tmp = pd.M{j}(motor, gearbox); 


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

        % Add the linear inequalities in G_ineq, h_ineq
        if ~isempty(G_ineq)
        	% Add these to A_ineq other 
        	A_ineq_G = sparse([], [], [], size(G_ineq, 1), dim_y, nnz(G_ineq));
        	A_ineq_G(:, x_idxs) = G_ineq; 
        	b_ineq_G = h_ineq; 

        	A_ineq_extra = [A_ineq_extra; A_ineq_G];
        	b_ineq_extra = [b_ineq_extra;  -b_ineq_G]; % NOTE signs 
        end  

        %% ----- Incorporate Actual Cost --------------
        %[M0_row, M0_col, M0_val] = find(pd.M{1}(motor, gearbox)); 
        [M0_row, M0_col, M0_val] = find(pd.M0(motor, gearbox)); 


        Q_cost = sparse(x_idxs(1) + M0_row - 1, x_idxs(1) + M0_col - 1,...
        									 M0_val, dim_y, dim_y); 

        %r0_tmp = pd.r{1};
        r0_tmp = pd.r0; 

        if isa(r0_tmp, 'function_handle')
            r0 = r0_tmp(motor, gearbox);
        else 
            r0 = r0_tmp;
        end 
        
        %beta0_tmp = pd.beta{1}; 
        beta0_tmp = pd.beta0; 

        if isa(beta0_tmp, 'function_handle')
            beta0 = beta0_tmp(motor, gearbox);
        else 
            beta0 = beta0_tmp;
        end 
        
        %Q0 = pd.Q{1}(motor, gearbox); % diag vector 
        %c0_tmp = pd.c{1};
        Q0 = pd.Q0(motor, gearbox); % diag vector -- NOTE: should we do the function handle test here too? 
        c0_tmp = pd.c0;

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
        	c_rho_aug(gm_idxs) = so.* obj.tolerances.rho; 
        	c_rho_aug(mu_idxs) = -so .* obj.tolerances.rho; 
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

    end % end build_matrices



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
        ratio = gearbox.direction .* gearbox.ratio;  % NOTE: accounts for dir
        eta = gearbox.efficiency; 

        omega_motor = omega*ratio; 

        I_comp = comp_torques.I_comp; 

        sol.I = y_sol(index_map.I);
        sol.V = sol.I*motor.R + motor.k_t*omega_motor; % Voltage  
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
            gm = y_sol(index_map.gm); 
            mu = y_sol(index_map.mu);
        else 
            sol.tau = ratio*k_t*(I_shift) ...
                    + ratio*static_motor_friction - tau_gearbox_inertia;
        end 



        % calculate compensation torque 
        % TODO -- make sure to account for zero vel frictoi cone in outputs 
        
        % For debug 
        tau_comp = sol.tau + tau_gearbox_inertia;  % tau delivered to + torque eateb up by gb inetita 
       
        % in this context, strictly greater 
        % TODO double check with rever direction gearboxes
        driving = [tau_comp .* sign(gearbox.direction * omega) > 0];  % note OR equals 
        driven = [tau_comp .* sign(gearbox.direction * omega) < 0];

        
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


        if isfield(index_map, 'binary')
            % NOTE: may decide to check on all indices
            min_gmmu_all = min(abs(gm), abs(mu));
   
            % Only pick out the indices we care about 
            gm_bin = gm(index_map.binary);
            mu_bin = mu(index_map.binary); 
            min_gmmu = min_gmmu_all(index_map.binary);
            init_directions = driving(index_map.binary); 

            sol.del = y_sol(index_map.del);


            % Only really concerned with where decomp may be bad (aug_idxs)            
            gmmu_bad_idxs = find(min_gmmu > obj.tolerances.gmmu_tol); 
            bad_vals = min_gmmu(gmmu_bad_idxs);
            bad_bin_idxs = gmmu_bad_idxs; 
            
          
            [~, reindex] = sort(bad_vals, 'descend'); 

            bad_bin_idxs = bad_bin_idxs(reindex); % now sorted 
            directions = init_directions(reindex); 

            tau_check = sol.tau - (sol.tau_friction + sol.tau_inertia);
            tau_error = abs(tau_check - sol.tau_em); 

            % index out to return 
            tau_error = tau_error(index_map.binary(bad_bin_idxs));
            tau_error = tau_error(reindex);
        end 

        

    end % end parse_solution

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


        % find initial bad indices
        I_init = y_sol(index_map.I);
        gm_init = y_sol(index_map.gm);
        mu_init = y_sol(index_map.mu); 
        min_gmmu = min(abs(gm_init), abs(mu_init));
        bad_idxs = find(min_gmmu > obj.tolerances.gmmu_tol);        % absolute indexing 


        % and maybe dont alter others that are already close 


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
        s = (gm - mu).^2; 
        %gmsq = gm(index_map.binary).^2;
        del = driving(index_map.binary); 


        %fix_idxs = false(length(I), 1); 
        %fix_idxs(bad_idxs) = true; % dont both 'cleaning' where already good enough 
        %fix_idxs = and(fix_idxs, moving);  % dont clean where static 

        %% Check if making this adjustment might lead to numerica tolerancing 
        % issue on current at indices where the initial gm/mu decomposition 
        % was good enough in the first place 

        fix_idxs = moving; 

        I_ub_bad = [I > matrices.ub(index_map.I) + obj.tolerances.feastol];  
        I_lb_bad = [I < matrices.lb(index_map.I) - obj.tolerances.feastol]; 
        I_bad = or(I_ub_bad, I_lb_bad);
        gmmu_good = [min_gmmu < obj.tolerances.gmmu_tol]; 
        ignore = and(I_bad, gmmu_good); 
        fix_idxs = and(moving, not(ignore));

        %% 
        tmp_sol(index_map.I(fix_idxs)) = I(fix_idxs);
        tmp_sol(index_map.gm(fix_idxs)) = gm(fix_idxs);
        tmp_sol(index_map.tau(fix_idxs)) = tau(fix_idxs); 
        % 
        %fix_idxs_shift = ismember(index_map.binary, find(fix_idxs));
        %tmp_sol(index_map.del(fix_idxs_shift)) = del(fix_idxs_shift);       % shifted 

        % removing redundancies helps numerical issues 
        %tmp_sol(index_map.s(fix_idxs)) = s(fix_idxs); 
        %tmp_sol(index_map.mu) = mu; 
        %tmp_sol(index_map.sig) = sig;
        %tmp_sol(index_map.Isq) = Isq;
        %tmp_sol(index_map.musq) = musq; 
        %tmp_sol(index_map.gmsq(fix_idxs_shift)) = gmsq(fix_idxs_shift);     % shifted

        not_nan_idxs = find(~isnan(tmp_sol)); 
        clean_matrices = matrices; 

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


        % Want to fix all of the values we decided on above 

        % Check for direct upper bound issues 


        % TODO Something to consider is if x-vals are up against bounds....
        % work through this and maybe try to construct an example where we run into this
        % 
        % by keeping torque the same dont need to worry about x vars that are 
        % only coupled into torque 

        %{

        ub_bad_idxs = find(tmp_sol > matrices.ub + obj.tolerances.feastol);  
        lb_bad_idxs = find(tmp_sol < matrices.lb - obj.tolerances.feastol); 

        
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
                    disp('tolerancing error on tau')
                else 
                    disp('aaaa ---------------------- ')
                end 
            end 

            % now should be cleanable without violating bounds however 
            % may create new bad idxs 

            % case or recusrion limtis 
            disp('fiixxxxxxx')
            [new_sol, new_cost, result] = obj.socp_solve(clean_matrices, inf);

            
            disp('about to make recursive clean')
            keyboard

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
        %} 
        ff = 1e-3; % fudge factor on specified bounds to help with numerics
        % Anything that was fixed to zero we want to keep numerically zero
        % for the rest we will loosen the bounds a little bit to help with numerical issues
        tmp_lb = max(matrices.lb, tmp_sol - [tmp_sol ~= 0]*ff);
        tmp_ub = min(matrices.ub, tmp_sol + [tmp_sol ~= 0]*ff);


        clean_matrices.lb(not_nan_idxs) = tmp_lb(not_nan_idxs);
        clean_matrices.ub(not_nan_idxs) = tmp_ub(not_nan_idxs);


        [new_sol, new_cost, result] = obj.socp_solve(clean_matrices, inf);

        cost_diff = new_cost - init_cost;
        cost_diff_rel = cost_diff/min(abs([new_cost, init_cost]));

        % TODO using 0.01 here calling this parameter a reltol might not 
        % be the most accurate -- should still be setable and 1% is fine 
        % may just need to add another parameter entirely
        if (cost_diff > obj.tolerances.abstol) && ...
                                (cost_diff_rel > obj.tolerances.reltol)
            new_sol = nan;
            new_cost = inf; 
        end
         
    end 


    function [tau_peak_min, tau_rms_min] = compute_torque_bounds(obj)
    % compute_torque_bounds 
    %
    %   If T, and tau_c do not depend directly on the motor and gearbox, we can
    %   sometimes establish lower bounds on the peak (abs) output torque and  
    %   the rms torque. We can use these bounds to exclude some motor/gearbox
    %   combinations from our search off the bat 
        tau_peak_min = 0;  % the lower bound if cant presolve anything   
        tau_rms_min = 0;      % the lower bound if cant presolve anything 

        try % check if there is depedendance 
            T = obj.problem_data.T([],[]);
            tau_c = obj.problem_data.tau_c([], []); 
        catch ME 
            return; % if T and tau_c depend on motor/gearbox cant do anything 
        end 

        try % check if there is depedendance 
            G = obj.problem_data.G([], []);
            h = obj.problem_data.h([], []); 
        catch ME % TODO -- add check on error type, otherwise rethrow
            G = []; 
            h = [];
        end 

        try % check if there is depedendance 
            G_ineq = obj.problem_data.G_ineq([],[]);
            h_ineq = obj.problem_data.h_ineq([],[]); 
        catch ME % TODO - should add check on error type 
            G_ineq = []; 
            h_ineq = [];
        end 

        % If G,h, G_ineq, h_ineq also dont rely on motor/gearbox can use these
        % constraints as well for a tighter problem 

        %
        %       Tx + tau_c = tau 
        %     

        % full optimization vector y = [x, tau, tau_peak_abs]
        % formulate as Ax = b constraint
        n = obj.problem_data.n; % number of time points
        w = obj.problem_data.w; % dimensionality of x 
        dim_y =  w + n + 1; 

        % torque equality -- same equality constraints for both problems 
        A_tau = zeros(n, dim_y); 
        A_tau(:, 1:w) = T; 
        A_tau(:, w + (1:n)) = -eye(n);
        b_tau = -tau_c; 
        lb = -inf(dim_y, 1);
        ub = inf(dim_y, 1);

        if ~isempty(G)
            A_G = zeros(length(h), dim_y);
            A_G(:, 1:w) = G; 
            b_G = -h; 
        else 
            A_G = []; b_G = [];
        end 
        A_eq = [A_tau; A_G];
        b_eq = [b_tau; b_G]; 

        %% Solve the problem of maximizing peak absolute torque 
        if ~isempty(G_ineq)
            A_G_ineq = zeros(length(h_ineq), dim_y);
            A_G_ineq(:, 1:w) = G_ineq; 
            b_G_ineq = -h_ineq; 
        else 
            A_G_ineq = []; b_G_ineq = [];
        end 

        % Inequality  any +/-tau <=tau_peak_abs, 
        A_ineq_tau = zeros(2*n, dim_y);
        A_ineq_tau(1:n,  w + (1:n)) = eye(n);
        A_ineq_tau(n + (1:n),  w + (1:n)) = -eye(n);
        A_ineq_tau(:, w + n + 1) = -1; 
        b_ineq_tau = zeros(2*n, 1); 

        c_cost = zeros(dim_y, 1); 
        c_cost(end) = 1; % mimimize peak (then know peak is at least this)

        ps_matrices.c_cost = c_cost;  
        ps_matrices.Q_cost = sparse(dim_y, dim_y); % empty 

        ps_matrices.ub = ub;
        ps_matrices.lb = lb;
        ps_matrices.A_eq = A_eq;
        ps_matrices.b_eq = b_eq; 
        ps_matrices.A_ineq = [A_ineq_tau; A_G_ineq];
        ps_matrices.b_ineq = [b_ineq_tau; b_G_ineq];
        ps_matrices.beta0 = 0;
        ps_matrices.quadcon = struct([]);


        [tmp_sol1, tau_peak_min] = obj.socp_solve(ps_matrices, inf); 


        %% Solve the problem of minimizing rms torque 
        % -- will overwrite inequality constraints and add a quad constraint
        % full optimization vector y = [x, tau, tau_rms]
        % NOTE: may want option to turn this off if uneven time vector and this
        % is not truly RMS -- todo 
        ps_matrices.A_ineq = A_G_ineq;
        ps_matrices.b_ineq = b_G_ineq; 

        % add constraint (1/n^2) tau' * tau <= tau_rms^2  
        quadcon = struct();
        Qc = sparse([], [], [], dim_y, dim_y, n);
        Qc(w + (1:n), w + (1:n)) = (1/n^2) * eye(n); 
        quadcon.Qc = Qc; 
        qc = zeros(dim_y, 1); 
        qc(end) = -1;
        quadcon.q = qc;
        quadcon.rhs = 0; 
        quadcon.sense = '<'; 
        ps_matrices.quadcon = quadcon; 

        [tmp_sol2, tau_rms_sq_min] = obj.socp_solve(ps_matrices, inf); 
        tau_rms_min = sqrt(tau_rms_sq_min); 
    end 
    

    function [tau_motor_friction, tau_static_friction] = ...
                                            friction_model(obj, motor, gearbox)
    %
    %   TODO - thorough explanation
    %
    %
    %
    %
        omega = obj.problem_data.omega;
        sign_omega = sign(omega); 


        if strcmp(motor.type, 'DC')

            if ~isnan(motor.coulomb_friction)
                tau_static_friction = motor.coulomb_friction;
                coulomb_friction = sign_omega*motor.coulomb_friction;
            else 
                if isnan(motor.I_nl)
                    I_nl = (motor.V - motor.k_t*motor.omega_nl)/motor.R; 
                else 
                    I_nl = motor.I_nl;
                end 
                coulomb_friction = sign_omega*motor.k_t*I_nl;
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
            
        % Use Settings  
        if ~obj.physics.f_damping
            viscous_friction = 0*viscous_friction;
        end 

        if ~obj.physics.f_coulomb
            coulomb_friction = 0*coulomb_friction;
        end 

        if ~obj.physics.f_static
            tau_static_friction = 0; 
        end 

        tau_motor_friction = coulomb_friction + viscous_friction; 
    end 


    function [y_sol, combo_cost, result] = socp_solve(obj, matrices, cutoff)
    %
    %
    %   TODO -- this all needs cleaning -- might want to add a seperate 
    %   'gurobi solve' file for consistency
    %
        model = [];
        Q_cost = matrices.Q_cost;
        c_cost = matrices.c_cost;
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
            [result, model] = gurobi_solve(Q_cost, c_cost, beta0, A_eq, b_eq,...
                                             A_ineq, b_ineq, quadcon, lb, ub,...
                                                    cutoff, obj.tolerances);
        elseif strcmpi(obj.settings.solver, 'ecos') 
            if strcmpi(obj.settings.solver, 'ecos')
                [c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc, G_rsoc, h_rsoc] = ...
                             prep_socp(Q_cost, c_cost, beta0, A_eq, b_eq,...
                        A_ineq, b_ineq, quadcon, lb, ub,  cutoff, 'ecos');
                [result, data] = ecos_solve(c, A_eq, b_eq, A_lp, b_lp,...
                            G_soc, h_soc, G_rsoc, h_rsoc, obj.tolerances);
            end
        else
            error('invalid solver'); % shouldnt be possible 
        end 

                    % ...maybe copy outside if for reuse? 
        if ~isinf(result.objval)
            y_sol = result.x; 
            combo_cost = result.objval; 
        else
            y_sol = nan(length(lb), 1); 
            combo_cost = inf;  
        end 

    end % socp_solve

    function [y_sol, combo_cost, result] = ...
                                mixed_int_solve(obj, matrices, bin_idxs, cutoff)
    %
    %
    %   TODO -- this all needs cleaning -- might want to add a seperate 
    %   'gurobi solve' file for consistency
    %
        model = [];
        Q_cost = matrices.Q_cost;
        c_cost = matrices.c_cost;
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
            result = gurobi_solve(Q_cost, c_cost, beta0, A_eq, b_eq,...
                                             A_ineq, b_ineq, quadcon, lb, ub,...
                                        cutoff, obj.tolerances, bin_idxs);
        elseif strcmpi(obj.settings.solver, 'ecos') 
            error('erez still needs to write the branch and bound module')
        else
            error('invalid solver'); % shouldnt be possible 
        end 

                    % ...maybe copy outside if for reuse? 
        if ~isinf(result.objval)
            y_sol = result.x; 
            combo_cost = result.objval; 
        else
            y_sol = nan(length(lb), 1); 
            combo_cost = inf;  
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

%
%       Input validation functions 
%
%{
function Q = validate_Q()
end 

function M = validate_M()
end 

function M0 = validate_M0()
end 
%} 

function dirs = find_dirs(tofind)
% copied verbatim from https://www.mathworks.com/matlabcentral/answers/347892-get-full-path-of-directory-that-is-on-matlab-search-path
% shoutout to Walter Robinson
    esctofind = regexptranslate('escape', tofind);   %in case it has special characters
    dirs = regexp(path, pathsep,'split');          %cell of all individual paths
    temp = unique(cellfun(@(P) strjoin(P(1:find(strcmp(esctofind, P),1,...
                'last')),filesep), regexp(dirs,filesep,'split'), 'uniform', 0));    
    dirs = temp(~cellfun(@isempty,temp));     %non-empty results only
    % remove any .git paths, complicates things submodules 
    dirs = dirs(~cellfun(@(x)contains(x, '.git'), dirs)); 
end 


function validate_c(cj, omega, input_size, input_num)
    j = input_num;
    n = input_size; 
    assert(size(cj, 1) == n, ['update_problem: c_%d incorrect length, ',...
                    'expected %d, got %d'], j, n, size(cj, 1) );
    assert(size(cj, 2) == 1, 'update_problem: c_%d must be col vector', j-1);

    assert(min(cj.*omega) >= 0, ['update_problem: c_%d'' must ',...
                                    'satisfy c .* omega >= 0'], j-1); 
end 

function validate_r(rj, input_size, input_num)
    j = input_num;
    w = input_size;
    assert(size(rj, 2) == 1, 'update_problem: r_%d must be col vector', j-1);
    assert(size(rj, 1) == w, ['update_problem: r_%d incorrect length, ',...
                            'expected %d, got %d'], j, w, size(rj, 1) );
end

           


%%%% Non Class Methods 
function validate_dependencies()
% Make sure we have everything we need installed 
    if isempty(which('mgdb.m'))
        error('mgdb.m (Motor-Gearbox DataBase) not found');
    end 
    if isempty(which('gurobi.m')) && isempty(which('ecos.m'))
        error('no valid solvers installed (Gurobi or ECOS)');
    end 
end 


function is_valid = valid_objective(objective)

    is_valid = false; % TODO 
end 

function validate_soc(Mj)
    % TODO -- simplify 
    [row, col, val] = find(Mj)
    %unique_col = unique(col); 
    unique_col = find(any(Mj));  % faster  

    M_small = Mj(:, unique_col)
    M_small = full(M_small(unique_col, :)); 
    [V, D] = eig(M_small); 

    % TODO -- can remove this eigen decmop code -- see prep_socp


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

function def_settings = default_settings()
    def_settings = struct('solver', 'gurobi',...
                           'verbose', 1,...
                           'line_freq',500,... % how often to print new line 
                           'print_freq', 1);  % how often to update line 
end 

function def_tolerances = default_tolerances()
    def_tolerances = struct('rho', 1e-4,...
                            'gmmu_tol', 1e-3, ... % 1 ma 
                            'feastol', 1e-5,...    % 1e-5
                            'reltol', 1e-2,... % 1 percent optimality
                            'abstol', 1e-3,...  % if tighter, sometimes issues
                            'qcvx_reltol', 1e-3,...
                            'qcvx_abstol', 1e-7);
end 

function def_physics = default_phyics()
    %
    %  TODO -- exaplain all these and defaults 
    %
    def_physics = struct('inertia', true,...
                            'f_damping', true,...
                            'f_coulomb', true,...
                            'f_static', false); 
end 

function [test_motor, test_gearbox] = test_motor_gearbox()

    test_motor = struct('key', 'none',...
                        'manufacturer', 'none',...
                        'ID', 'none',...
                        'type', 'DC',...       
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
                            'type', 'planetary',...        
                            'stages', 1,...
                            'ratio', 2,...
                            'mass', 0.1,...
                            'inertia', 1e-6,...
                            'efficiency', 0.9,...
                            'direction', 1,...
                            'max_int_torque', inf,...
                            'max_cont_torque', inf); 
end 

