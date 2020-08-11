function result = sedumi_solve(c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc)
%*******************************************************************************
%   Function:
%        
%
%   Description:
%      
%   Inputs:
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

% All initial variables are in "Free" cone
dims.f = length(c); 

% Add Slack Variables for Inequality Constraints 
num_ineq_slack = length(b_lp);
dims.l = num_ineq_slack; 

% Add Slack Variables For 
dims.q = zeros(numel(G_soc), 1);
for j = 1:numel(G_soc)
    dims.q(j) = size(G_soc{j}, 1);
end 

num_soc_slack = sum(dims.q); 


% Augment Equality Constraint matrix to incorporate 
% Cone Constraints (nonnegative + SOC)
% Concat 
G = cat(1, G_soc{:});
h = cat(1, h_soc{:}); 

dim_y = dims.f + num_ineq_slack + num_soc_slack; 

num_eq = length(b_eq) + length(b_lp) + length(h);

A_eq_aug = zeros(num_eq, dim_y);
A_eq_aug(1:length(b_eq), 1:dims.f) = A_eq; 
% Inequalities
lp_rows = length(b_eq) + (1:num_ineq_slack);
A_eq_aug(lp_rows, dims.f + (1:num_ineq_slack)) = eye(dims.l);
A_eq_aug(lp_rows, 1:dims.f) = A_lp;
% SOCs 
soc_rows = length(b_eq) + num_ineq_slack + (1:num_soc_slack);
A_eq_aug(soc_rows, dims.f + dims.l + (1:num_soc_slack)) = eye(num_soc_slack);
A_eq_aug(soc_rows, 1:dims.f) = G;

b_eq_aug = [b_eq; b_lp; h]; 


% Augment Objective Term to higher dimensions
c_aug = zeros(dim_y, 1);
c_aug(1:dims.f) = c; 


% TODO -- add solver settings 
pars.fid = 0; % no output to screen
[x, y, data] = sedumi(A_eq_aug, b_eq_aug, c_aug, dims, pars);


result.x = x(1:dims.f - 1);   % NOTE: some dual info could be good, think on how to integrate this better 

% Numerical issues? 
if (data.pinf == 1) || (data.dinf == 1) || data.numerr == 2% infease 
	% infeas

	result.objval = inf; 
else
	result.objval = c_aug'*x; 
end 

% TODO -- warning/checks for numerical issues (issue warnings)
result.runtime = data.cpusec;  
