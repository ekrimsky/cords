function result = ecos_solve(c, A_eq, b_eq, A_lp, b_lp, G_soc, h_soc)
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


dims.l = size(A_lp, 1); 
dims.q = zeros(numel(G_soc), 1);
for j = 1:numel(G_soc)
    dims.q(j) = size(G_soc{j}, 1);
end 

% Concat 
G = [A_lp; cat(1, G_soc{:})];
h = [b_lp; cat(1, h_soc{:})]; 

% TODO -- incorporate more detailed settinsg into caller
% Potnetially link verbosity as well -- add as input 
opts =  ecosoptimset('VERBOSE', 0,...
                     'FEASTOL', 1e-4,...
                      'ABSTOL', 1e-4);

[x, y, data, s, z] = ecos(c, G, h, dims, A_eq, b_eq, opts); 




result.x = x(1:end-1);   % NOTE: some dual info could be good, think on how to integrate this better 

if data.exitflag == 0
    result.objval = data.pcost; 
else 
    %data.exitflag == 2
    result.objval = inf; 
end 
%else % TODO -- check 
%    keyboard%
%
%end 



result.runtime = data.timing.runtime; % TODO -- comaptibility 

