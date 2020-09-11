classdef mgdb < matlab.mixin.Copyable 
%    Summary goes here  
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
%       Erez Krimsky, ekrimsky@stanford.edu, 9/9/20
%       Stanford University, Biomechatronics Lab 
%
properties (GetAccess = public, SetAccess = private)


    motors          % containers.Map
    gearboxes       % containers.Map
    compatibility 

    verbosity

    %
    %
    %       Filter Cutoffs -- NOTE: allow direct access or no? 
    %
    %
    filters 

    % Setting up as toolbox/package might make specifying file paths easier? 
end 


methods (Access = public)


    function obj = mgdb(varargin) %

        % TODO --- accouint for input redirecting us to a new search path 
        % If no args look in a a default location
        
        % Could look in other places too i guess
        % cell array of paths 
        database_paths = {'~/Documents/MotorDatabase'};           % we will look recursively in this path 

        % motor files called "*_motors.csv"
        % gearbox files called "*_gearboxes.csv"
        % compatibility filed called "*_compatibility.csv"
        % see somewhere else (TODO) for formatting 
        for i = 1:length(database_paths)
            pth = database_paths{i};
            motor_file_tmp = dir(sprintf('%s/**/*_motors.csv', pth));
            gearbox_file_tmp = dir(sprintf('%s/**/*_gearboxes.csv', pth));
            compat_file_tmp = dir(sprintf('%s/**/*_compatibility.csv', pth));

            m_tmp = cellfun(@(x) fullfile(x.folder, x.name), num2cell(motor_file_tmp), 'UniformOutput', false);
            g_tmp = cellfun(@(x) fullfile(x.folder, x.name), num2cell(gearbox_file_tmp), 'UniformOutput', false);
            c_tmp = cellfun(@(x) fullfile(x.folder, x.name), num2cell(compat_file_tmp), 'UniformOutput', false);
            if i == 1
                motor_files = m_tmp;
                gearbox_files = g_tmp;
                compat_files = c_tmp; 
            else 
                motor_files = [motor_files, m_tmp];
                gearbox_files = [gearbox_files, g_tmp];
                compat_files = [compat_files, c_tmp];
            end 
        end 

        % Read in motor csvs, NOTE move to own function takinfg in list 
        [motor_keys, motor_values] = add_motors(motor_files); 
        % Read in Gearbox csvs --NOTE: may move to its own function taking in list 
        [gb_keys, gb_values] = add_gearboxes(gearbox_files); 

        % NOTE: A motor might appear mulitpe times in multiple compatibility 
        % files (could be compatible with multiple manufactureres for example)
        obj.motors = containers.Map(motor_keys, motor_values);
        obj.gearboxes = containers.Map(gb_keys, gb_values);
        obj.compatibility = containers.Map('KeyType', 'char', 'ValueType', 'any'); % value type will be other maps 
        % Read in compatibility csvs 
        add_compatibility(obj, compat_files); % TODO -- all files 

        % TODO -- add direct drive 'gearbox'




        %
        %
        %      Initialize Filters 
        %
        %  filters is a struct where each field is different criteria
        %  initialized to not filter out any possible choices 
        % 

    end 



    function [motor_combo_keys, gearbox_combo_keys] = get_combinations(obj)

        % get motor, gearbox key tuples 
        motor_keys = obj.motors.keys; 

        motor_combo_keys = {}; % will be a cell array of motor keys 
        gearbox_combo_key = {}; % will be a cell array of gb keys 

        % NOTE: may want to allocate pretty big then trim 
        cost_list = {}; % will be used to reindex and sort outputs 

        init_combos = 0;
        % For each motor, get all compatible gearboxes 
        for m_idx = 1:length(motor_keys)
            motor_key = motor_keys{m_idx}; 
            tmp_map = obj.compatibility(motor_key)
            gb_keys = tmp_map.keys; 
            for gb_idx = 1:lengh(gb_keys) 
                gb_key = gb_keys{gb_idx};
                % Check if combo is valid given the filters
                if obj.valid_combo(motor_key, gb_key)
                    motor_combo_keys{end + 1} = motor_key;
                    gearbox_combo_key{end + 1} = gb_key; 
                    cost_list{end + 1} = tmp_map(gb_key);
                end 
                init_combos = init_combos + 1; % to keep track of how many removed 
            end 
        end 

        num_combos = length(motor_combo_keys);
        num_removed = init_combos - num_combos;
        % TODO - verb
        fprintf('Found %d initial combinations, removed %d\n', init_combos, num_removed);

        % TODO -- some indicator of how many combos filtering removed
        % will want verbosity settings --> pass in from caller
        costs = cell2mat(cost_list); 
        [~, reindex] = sort(costs);

        % Sort by rankings 
        motor_combo_keys = motor_combo_keys(reindex);
        gearbox_combo_keys = gearbox_combo_keys(reindex); 
    end 

    function update_rankings(obj, motor_keys, gearbox_keys, new_costs)

        % check that motor_keys, gearbox_keys, new_costs are same length 

        % support string arrays seperate from cell arrays of chars?? 
        for i = 1:length(motor_keys)
            motor_key = motor_keys{i};
            if obj.compatibility.isKey(motor_key)
                tmp_map = obj.compatibility(motor_key);  % by reference 
                tmp_map(gearbox_keys(i)) = new_costs(i);
            else 
                error('Invalid motor key');
            end 
        end 
    end 

    function clear_filter(obj, varargin) 

        % with no extra args, clears all filters 

        % with args -- only clears an individual filter 

        % TODO -- come up with filter list 

    end 


    function update_filter(obj, criteria, value)

    end 

end 


methods (Access = private)


    function add_compatibility(obj, compat_files)

        % could combine all compat files together to make simpler 
        % assume this is done somewhere above this line of code 

        dd_key = 'DD';


        gearbox_keys = obj.gearboxes.keys; % WILL RETURN THE KEYS SORTED ALPHABETICALLY 

        % TODO -- check that compat files is cell array even if only 1 file 

        for kk = 1:length(compat_files)
            compat_file = compat_files{kk}; 
            compat_fid = fopen(compat_file);

            tline = fgetl(compat_fid);
            while ischar(tline)     % go through whole file
                line_cells = strsplit(tline, ',');  % might be no compat 
                motor_key = line_cells{1};

                % First check that the motor key is actually present 

                % For each motor add direct drive compatibility 

                % Now get gearboxes 

                % Loop through gearbox keys in the row
                % As a O(nlogn) compromise heres the plan
                % get the full list of gearbox keys O(n) 
                % Sort the list of gearbox keys lexocigraphically O(nlogn)
                % If test does NOT contain a wildcard, index directly into map
                % If test key DOES contain a wildcard, find the corresponding keys 
                % Since sorted, just need to find the first (lexicographicaly) wildcard match 
                % and the last wildcard match and then everything in between is a match 
                for i = 2:length(line_cells)
                    gb_key = line_cells{i}; % may be partial key 
                    if obj.gearboxes.isKey(gb_key)
                        new_gb_match_keys = {gb_key};  % assumed cell array 
                    elseif endsWith(gb_key, '*')        % partial key
                        new_gb_match_keys = find_matches(gearbox_keys, gb_key(1:end-1)); % TODO -- can rewrite a faster subroutine 
                    else % error 
                        error('Gearbox not found in database');
                    end 

                    % TODO -- if gb_match_keys is isempty? 
                    if i == 2
                        gb_match_keys = new_gb_match_keys;
                    else % append 
                        % could be more efficient to grow cell
                        gb_match_keys = [gb_match_keys, new_gb_match_keys];
                    end 

                    if isempty(gb_match_keys)
                        error('No gearbox matching key found in database');
                    end 
                end

                if obj.compatibility.isKey(motor_key)
                    % already have things added, append 
                    for j = 1:length(gb_match_keys)
                        gb_key = gb_match_keys{j};
                        tmp_map = obj.compatibility(motor_key);     % should be copy by ref
                        tmp_map(gb_key) = inf; % initialize all combination to have infinite cost 
                        % Could check if already key and issue warning if redundant
                    end 
                else 
                    % Add all the new ones together AND a direct drive option
                    inf_cell = num2cell(inf(1, length(gb_match_keys) + 1)); 
                    tmp_map = containers.Map(['DD', gb_match_keys], inf_cell); 
                    obj.compatibility(motor_key) = tmp_map;
                end 


                tline = fgetl(compat_fid);
            end % finish reading through lines of this file 
            fclose(compat_fid); 
        end % end looping through files 
    end 




    function [valid] = valid_combo(obj, motor_key, gearbox_key)

        % Check this combination of motor and gearbox against the filters 
        motor = obj.motors(motor_key);
        gearbox = obj.gearboxes(gearbox_key); 

        % TODO -- actually check against the filters 

        % total mass 
        % effective inertia (through gb ratio)
        % max_int_torque (check gearbox then motor)
        % max_vel (gearbox and motor) 
        % max_cont_power
        valid = true;
    end 

end 

%=================== END CLASS DEF =========================
end 

function [motor_keys, motor_values] = add_motors(motor_files)
    for i = 1:length(motor_files)
        [motor_keys_tmp, motor_cells_tmp] = read_motor_file(motor_files{i}); 
        if i == 1
            motor_keys = motor_keys_tmp;
            motor_values = motor_cells_tmp;
        else % append 
            % NOTE may want a faster append method 
            motor_keys = [motor_keys, motor_keys_tmp];
            motor_values = [motor_values, motor_cells_tmp]; 
        end 
    end 
end 

function [gb_keys, gb_values] = add_gearboxes(gearbox_files)
    for i = 1:length(gearbox_files)
        [gb_keys_tmp, gb_cells_tmp] = read_gearbox_file(gearbox_files{i}); 
        if i == 1
            gb_keys = gb_keys_tmp;
            gb_values = gb_cells_tmp;
        else % append 
            % NOTE may want a faster append method 
            gb_keys = [gb_keys, gb_keys_tmp];
            gb_values = [gb_values, gb_cells_tmp]; 
        end 
    end 
end 


function [motor_keys, motor_cells] = read_motor_file(motor_file)
    T = readtable(motor_file);    % NOTE: read table can be slow - may be able to speed up by specifing delims and other things
    motor_struct = table2struct(T);
    motor_keys = {motor_struct(:).Key};
    motor_cells = num2cell(motor_struct); 
end 

function [gearbox_keys, gearbox_cells] = read_gearbox_file(gearbox_file)
    T = readtable(gearbox_file);
    gearbox_struct = table2struct(T);
    gearbox_keys = {gearbox_struct(:).Key};
    gearbox_cells = num2cell(gearbox_struct); 
end 


function matches = find_matches(key_list, key)
    match_idxs = startsWith(key_list, key); 
    matches = key_list(match_idxs);
end 


%{
% TODO -- move to utility 
function idx = find_in_sorted()

    % TODO -- actually write this, for now, lazy :) 


end 
%} 