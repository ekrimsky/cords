function [result, data] = sedumi_solve(c, A_eq, b_eq, A_lp, b_lp,...
                                     G_soc, h_soc, G_rsoc, h_rsoc, varargin)
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
%    https://github.com/sqlp/sedumi
%
%   NOTE: Unlike ECOS, sedumi specifies different inputs for rotated SOCs
%   and standard quadratic SOCS 
%
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 8/11/20 
%       Stanford University, Biomechatronic Lab 
%*******************************************************************************

% TODO -- this can be sped up a lot by switching to sparse ops 


if ~isempty(varargin{:})
    settings = varargin{1};
    pars =  struct('fid', 0);

    %pars = struct('fid', 0,...    % verbosity
    %                'eps', settings.abstol);       % accuracy , default 1e-8
                       
else % default settings 
    pars =  struct('fid', 0);
end 


% All initial variables are in "Free" cone
dims.f = length(c); 

% Add Slack Variables for Inequality Constraints 
num_ineq_slack = length(b_lp);
dims.l = num_ineq_slack; 

% Add Slack Variables For Standard SOCs
dims.q = zeros(numel(G_soc), 1);
for j = 1:numel(G_soc)
    dims.q(j) = size(G_soc{j}, 1);
end 
num_soc_slack = sum(dims.q); 

%% Add slack vars for Rotated SOCs 
dims.r = zeros(numel(G_rsoc), 1);
for j = 1:numel(G_rsoc)
    dims.r(j) = size(G_rsoc{j}, 1);
end 
num_rsoc_slack = sum(dims.r); 

dim_y = dims.f + num_ineq_slack + num_soc_slack + num_rsoc_slack; 

% Augment Equality Constraint matrix to incorporate 
% Cone Constraints (nonnegative + SOC)
% Concat 
Gs = cat(1, G_soc{:}); 
hs = cat(1, h_soc{:}); 

Gr = cat(1, G_rsoc{:});
hr = cat(1, h_rsoc{:});


num_eq = length(b_eq) + length(b_lp) + length(hs) + length(hr);

A_eq_aug = zeros(num_eq, dim_y);
A_eq_aug(1:length(b_eq), 1:dims.f) = A_eq; 

% Inequalities
lp_rows = length(b_eq) + (1:num_ineq_slack);
A_eq_aug(lp_rows, dims.f + (1:num_ineq_slack)) = eye(dims.l);
A_eq_aug(lp_rows, 1:dims.f) = A_lp;

% Standard SOCs 
soc_cols = dims.f + dims.l + (1:num_soc_slack);
soc_rows = lp_rows(end) + (1:num_soc_slack);
A_eq_aug(soc_rows, soc_cols) = eye(num_soc_slack);
A_eq_aug(soc_rows, 1:dims.f) = Gs;

% Rotated SOCs
rsoc_cols = soc_cols(end) + (1:num_rsoc_slack);
rsoc_rows = soc_rows(end) + (1:num_rsoc_slack);
A_eq_aug(rsoc_rows, rsoc_cols) = eye(num_rsoc_slack);
A_eq_aug(rsoc_rows, 1:dims.f) = Gr;


b_eq_aug = [b_eq; b_lp; hs; hr]; 


% Augment Objective Term to higher dimensions
c_aug = zeros(dim_y, 1);
c_aug(1:dims.f) = c; 


[x, y, data] = sedumi(A_eq_aug, b_eq_aug, c_aug, dims, pars);


result.x = x(1:dims.f - 1);   % NOTE: some dual info could be good, think on how to integrate this better 

% Numerical issues? 
if (data.pinf == 1) || (data.dinf == 1) || data.numerr == 2% infease 
	% infeas
	result.objval = inf; 

    keyboard
else
	result.objval = c_aug'*x; 
end 

% TODO -- warning/checks for numerical issues (issue warnings)
result.runtime = data.cpusec;  
