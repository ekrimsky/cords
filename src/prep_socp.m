function [c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc] = prep_socp(Q, obj, objcon, A_eq, b_eq,...
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

%% Go through SOCP Constraints 
% will define cell array and then concat all the matrices or something like that
G_cell = cell(num_quadcon, 1); 
h_cell = cell(num_quadcon, 1); 

% options: 1) standard quadratic, 2) standard SOC 3) Rot. SOC 

for j = 1:num_quadcon
    rhs = quadcon(j).rhs;
    Qj = quadcon(j).Qc;
    qc = quadcon(j).q/2;  % NO FACTOR IN GUROBI SOLVE -- MAY NEED TO CHECK DOCS 

    % Because we are callign from motor select, will assume
    % our inputs are valid 

    % x^T Q x + 2qx < rhs   
    %qc = quadcon(i).q;

    [row, col, ~] = find(Qj);
    %unique_cols = unique(col); 
    unique_cols = find(any(Qj)); 

    Q_short = Qj(unique_cols, :);  % cut empty rows 
    Q_small = full(Q_short(:, unique_cols)); 
    [V, D] = eig(Q_small); 
    v = V(:, 1); 
    min_eig = D(1,1); 

    % Type 1 -- Standard PSD Quadratic Constraint 
    if min_eig >= 0     % standard quad constraint


        % TODO -- facotr in RHS 

        if length(qc) < dim_y   % For all cases except cost itself
            qc = [qc; 0];
        end 

        qc_half = qc'/sqrt(2); 

        %[new_rows, ~, ~] = find(Q_short); 
        W = sparse([] , [], [], length(unique_cols), dim_y, numel(Q_small)); 
        W(:, col) = chol(Q_small);  % faster than sqrtm 


        % should reference ECOS paper or some other documentation 
        % relalted to some code in ECOSQP 
        % See note: 6/3/20 
        %G = [qc_half;  -W; -qc_half];
        % NOTE: can speed up a lot of ths spart indexing in this block 
        G = sparse([], [], [], length(unique_cols) + 2, dim_y, nnz(W) + 2*numel(qc_half)); 
        G(1, :) = qc_half;
        G(2:end-1, :) = -W; 
        G(end, :) = -qc_half; 

        h  = [1/sqrt(2);  zeros(size(W,1),1); 1/sqrt(2)]; 

        
        %G = [c_half', -1/sqrt(2);
        %      -W,  zeros(size(W,1),1);
        %      -c_half', +1/sqrt(2)];
        %h  = [1/sqrt(2);  zeros(size(W,1),1); 1/sqrt(2)]; 
    else % Type 2 -- SOC Constraint 
        if nnz(v) == 1 % standard SOC 
            % Get positive def submatrix 

            display('foooooooooooooooooooooo'); error('fff')

            v_idx = find(v); 
            exclude_col = unique_cols(v_idx); 
            psd_cols = setdiff(unique_cols, exclude_col); 
            Qhalf_psd = chol(Q_small(psd_cols, psd_cols)); 

            W = sparse([], [], [], length(psd_cols), dim_y, nnz(Qhalf_psd));
            W(:, psd_cols) = Qhalf_psd; 

            num_rows =  length(psd_cols) + 1;             
            c_tmp = zeros(1, dim_y); 
            c_tmp(exclude_col) = sqrt(-min_eig); 
            G = [-c_tmp; -W]
            h = zeros(num_rows, 1);

        elseif nnz(v) == 2 % rotated SOC 


            display('bar'); error('fffff')

            v_idxs = find(v); 
            exclude_cols = unique_cols(v_idxs); 
            psd_cols = setdiff(unique_cols, exclude_cols); 
            Qhalf_psd = chol(Q_small(psd_cols, psd_cols)); 
            W = sparse([], [], [], length(psd_cols), dim_y, nnz(Qhalf_psd));
            W(:, psd_cols) = Qhalf_psd; 

            num_rows =  length(psd_cols) + 2;             
            c_tmp1 = zeros(1, dim_y); 
            c_tmp1(exclude_cols) = sqrt(-min_eig) * [1, 1]; 
            c_tmp2 = zeros(1, dim_y); 
            c_tmp2(exclude_cols) = sqrt(-min_eig) * [-1, 1]; 
            
            G = [c_tmp1; -2*W; ctmp2]
            h = zeros(num_rows, 1);
        else
            error('invalid constraint matrix')
        end 
    end 


    G_cell{j} = G; 
    h_cell{j} = h; 

end 


% TODO -- go through and rename throughout for clarity 
b_lp = h_lp;

G_soc = G_cell; 
h_soc = h_cell; 



