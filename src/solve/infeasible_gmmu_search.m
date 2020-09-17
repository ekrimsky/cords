
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