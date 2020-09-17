function [c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc, G_rsoc, h_rsoc] = ...
                                     prep_socp(Q, obj, objcon, A_eq, b_eq,...
                                    A_ineq, b_ineq, quadcon, lb, ub,  cutoff)
%*******************************************************************************
%   Function:
%        
%
%   Description:
%      
%
%   Outputs:
%       
%
%   Major Revisions:
%       
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 8/11/20 
%       Stanford University, Biomechatronic Lab 
%*******************************************************************************


% NOTE: Dont even need to "convert" to ecos -- absolutely no need to return 
% a model to the caller == Just solve it here -- makes more sense 
% and much less overhead 


% TODO -- in a loop finding eqs and ineqs will be very slow 
% we should force the caller to input seperate A_eq, b_eq
% and A_ineq, b_ineq -- For the purpose of this project 
% we dont need a general purpose converter -- we need something fast 
% NOTE: ECOS does NOT support constant objective term -- need to carry over
% TODO: if there is a quadratic term, we need introduce a new variable 
% TODO: check that model sense is min (flip signs on obj if not )
% TODO -- cutoffs 

% ASSUME ALL MATRICES ARE SYMMETRIC (CAN ALSO REPRESENT LIKE SO)


% objective is just 1 variable - cost 
% this lets us use cutoffs 

dim_y = length(obj) + 1; % cost augmentation 

c = zeros(dim_y, 1); 
c(end) = 1; % linear objective term just on single cost variable 
num_quadcon = numel(quadcon); 

A_eq = sparse([A_eq, zeros(size(A_eq, 1), 1)]); 
A_ineq = sparse([A_ineq, zeros(size(A_ineq, 1), 1)]); 


if nnz(Q) > 0    % Objective is quadratic 
    % Incorporate objective < cost_var as quadratic constraint 
    [row, col, val] = find(Q); 
    quadcon(num_quadcon + 1).Qc = sparse(row, col, val, dim_y, dim_y); 

    % TODO - double check -- gets divided by 2 below 
    quadcon(num_quadcon + 1).q = [obj; -2]; % augmented with cost var 
    quadcon(num_quadcon + 1).rhs = -objcon;   % TODO -- double check 
    num_quadcon = num_quadcon + 1; 
else 
    % objective is linear 
    % Add constraint to relate cost to c'x 
    A_cost = [obj', -1];
    A_eq = [A_eq; A_cost]; 
    b_eq = [b_eq; 0];
end 


%% Define Bound Constraints from Upper and Lower Bounds (ub, lb)
% use inf or -inf to denote NO explicit bound required 
% assume all inequalities structured as less than 
lb_idxs = find(lb ~= -inf);
lb_vals = lb(lb_idxs);
num_lb = length(lb_idxs);

ub_idxs = find(ub ~= inf);
ub_vals = ub(ub_idxs); 
num_ub = length(ub_idxs);

num_b = num_ub + num_lb;    % num bounds 
A_bound = sparse(1:num_b, [lb_idxs(:); ub_idxs(:)],...
                            [-ones(1, num_lb), ones(1, num_ub)], num_b, dim_y); 
b_bound = [-lb_vals; ub_vals]; 

if ~isinf(cutoff)
    A_cutoff = sparse(1, dim_y, 1, 1, dim_y); 
    b_cutoff = cutoff; 
else 
    A_cutoff = [];
    b_cutoff = []; 
end 


%% Define the simple linear inequality constraints for ecos (in the LP cone)
A_lp = [A_bound; A_ineq; A_bound; A_cutoff];
h_lp = [b_bound; b_ineq; b_bound; b_cutoff];
b_lp = h_lp;


%% Go through SOCP Constraints 
% will define cell array and then concat all the matrices or something like that
G_soc = {};  % standard socs 
h_soc = {};
G_rsoc = {};
h_rsoc = {}; 

% options: 1) standard quadratic, 2) standard SOC 3) Rot. SOC 

for j = 1:num_quadcon
    rhs = quadcon(j).rhs;
    Qj = quadcon(j).Qc;
    qc = quadcon(j).q/2;  % NO FACTOR IN GUROBI SOLVE -- MAY NEED TO CHECK DOCS 

    % Because we are callign from motor select, will assume
    % our inputs are valid (trust caller to validate inputs)

    % x^T Q x + 2qx < rhs   
    %qc = quadcon(i).q;




    [rows, cols, vals] = find(Qj);


    %%% detemine if SOC/ROTSOC or Standard PSD 
    %
    %
    %  If standard soc (ie. x^2 <= y^2) there will be a negative value on the diagonal
    %  If roated soc (ie x^2 <= yz) if assume symmetric -- 2 row/col entries
    %      where there is only one value in row/col, the value is negative
    %       and has a symetric entry about the diagonal - do O(n) search 

    ctype = 1; % flag for standard PSD constraint 
    for i = 1:length(vals)
        val = vals(i);
        if val < 0
            col = cols(i);
            row = rows(i); 
            if row == col
                % negative value on the diagonal - standard soc
                ctype = 2; 
                neg_val = val;
                neg_col = col;
                neg_row = row;
                break; 
            elseif nnz(Qj(row, :)) == 1 % singular row  
                ctype = 3;
                neg_val = val;
                neg_col = col;
                neg_row = row; 
                break; 
            end 
        end 

    end 


    % 
    %   Consider [1, -1;
    %            -1, 1]; % positivie SEMIdef  -- quadractic (x - y)^2
    % 
    unique_cols = find(any(Qj)); 
     

    switch ctype 
        % Type 1 -- Standard PSD Quadratic Constraint 
        case 1  % standard psd 

            % TODO -- facotr in RHS 

            % For all cases except cost itself
            if length(qc) < dim_y   
                qc = [qc; 0];
            end 
            qc_half = qc'/sqrt(2); 

            Q_short = Qj(unique_cols, :);  % cut empty rows 
            Q_small = full(Q_short(:, unique_cols)); 

           

            %W = sparse([], [], [], length(unique_cols), dim_y, nnz(Q_small)); 

            %% Cholesky decomposition has a lot of overhead and we frequently
            % single quadratic terms (1 x 1 matrice) so check for that explicitly
            % so we will check if its diagonl in which case do elementwise sqrt
           
            
            %W(:, unique_cols) = fast_chol(Q_small);

            % TODO should reference ECOS paper or some other documentation 
            % relalted to some code in ECOSQP 
            % See note: 6/3/20 
            % NOTE: can speed up a lot of ths spart indexing in this block 
            G = sparse([], [], [], length(unique_cols) + 2, dim_y,...
                             nnz(Q_small) + 2*numel(qc_half)); 
            G(1, :) = qc_half;
            G(2:end-1, unique_cols) = -fast_chol(Q_small); % expensive 
            G(end, :) = -qc_half; 

            h  = [1/sqrt(2);  zeros(size(Q_small,1),1); 1/sqrt(2)]; 
            %G = [c_half', -1/sqrt(2);
            %      -W,  zeros(size(W,1),1);
            %      -c_half', +1/sqrt(2)];
            %h  = [1/sqrt(2);  zeros(size(W,1),1); 1/sqrt(2)]; 
            G_soc{end + 1} = G;
            h_soc{end + 1} = h;
        case 2 % Standard SOC  
            c_tmp = zeros(1, dim_y); 
            c_tmp(neg_col) = sqrt(-neg_val); 

            %psd_cols = setdiff(unique_cols, neg_col); 
            psd_cols = fsd(unique_cols, neg_col); 

            Q_short = Qj(psd_cols, :);  % cut empty rows 
            Q_small = full(Q_short(:, psd_cols)); 
            Qhalf_psd = fast_chol(Q_small); % NOTE: fail on semidef? 

            W = sparse([], [], [], length(psd_cols), dim_y, nnz(Qhalf_psd));
            W(:, psd_cols) = Qhalf_psd; 

            num_rows =  length(psd_cols) + 1;             

            G = [-c_tmp; -W]
            h = zeros(num_rows, 1);
            G_soc{end + 1} = G;
            h_soc{end + 1} = h;
        case 3 % rotated SOC 
            exclude_cols = [neg_col, neg_row];

            c_tmp1 = zeros(1, dim_y); 
            c_tmp1(exclude_cols) = sqrt(-2*neg_val) * [-1, -1]; 
            c_tmp2 = zeros(1, dim_y); 
            c_tmp2(exclude_cols) = sqrt(-2*neg_val) * [1, -1];


            %psd_cols = setdiff(unique_cols, exclude_cols);   % cols in original
            psd_cols = fsd(unique_cols, exclude_cols);   % cols in original
            Q_short = Qj(psd_cols, :);  % cut empty rows 
            Q_small = full(Q_short(:, psd_cols)); 
            Qhalf_psd = fast_chol(Q_small); 

            num_rows =  length(psd_cols) + 2;     

            %W = sparse([], [], [], length(psd_cols), dim_y, nnz(Qhalf_psd));
            %W(:, psd_cols) = Qhalf_psd; 

            G = sparse([], [], [], length(psd_cols) + 2, dim_y,...
                             nnz(Qhalf_psd) + 2*nnz(c_tmp1)); 
            G(1, :) = c_tmp1;
            G(2:(length(psd_cols)+ 1), psd_cols) = -2*Qhalf_psd;
            G(length(psd_cols) + 2, :) = c_tmp2;


            h = zeros(num_rows, 1);
            G_rsoc{end + 1} = G;
            h_rsoc{end + 1} = h;
        otherwise
            error('Invalid quadaratic constraint type');
    end % end switch

end  % loop 

end % function 

% if works maybe move to onwn file 
%
% Fast set diff where speed up comes from assuming 
% both inputs are sorted and contain no repeated values 
% and both lists only have integer values 
% and out_list does NOT contain any values which are NOT in in_list
function c = fsd(in_list, out_list)

    c = zeros(length(in_list) - length(out_list), 1); 

    c_idx = 1; 
    o_idx = 1; 

    for i_idx = 1:length(in_list)
        val = in_list(i_idx); 

        if out_list(o_idx) > val
            c(c_idx) = val;
            c_idx = c_idx + 1; 
            if c_idx > length(c)
                break;
            end  
        else 
            o_idx = o_idx + 1;
        end
    end 
end 


function M = fast_chol(Q)

    if (size(Q, 1) == 1) || isdiag(Q)  % redundant but helps with diag overhead 
        M = sqrt(Q); 
    elseif size(Q, 1) == 2
        % common to have Q of the form
        %
        %   Q = [1, -1; 
        %       -1,  1]  which is only semidfefinite  
        if det(Q) == 0  
            M = (1/sqrt(trace(Q)))*Q; 
        else % strictly pos def, do explicit chol  
            % http://metamerist.blogspot.com/2008/03/googlaziness-cholesky-2x2.html#:~:text=All%20we're%20talking%20about,root%20of%20a%20square%20matrix.&text=The%20L%20matrix%20is%20lower,of%20the%20diagonal%20are%20zero.&text=Another%20thing%20that's%20clear%20is,to%20be%20symmetric%20for%20Cholesky.
            a = sqrt(Q(1,1));
            b = a/Q(1,2);  
            c =  sqrt(Q(2,2) - b^2); 
            M = [a, b; 0, c];
        end 
    else 
        try
            M = chol(Q);  % faster than sqrtm 
        catch ME   % for pos semi def case 
            [M, ~] = sqrtm(Q);
        end 
    end
end 