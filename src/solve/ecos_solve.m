function [result, data] = ecos_solve(c, A_eq, b_eq, A_lp, b_lp,...
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
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 8/11/20 
%       Stanford University, Biomechatronic Lab 
%*******************************************************************************

% ecos doenst require to distinquish between rotated and standard SOCs so 
% we can combine the, 

% TODO -- sparsify for speed 

G_soc = [G_soc, G_rsoc];  % combine the cells 
h_soc = [h_soc, h_rsoc];  % combine the cells 

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


if ~isempty(varargin{:})
    settings = varargin{1};
    %opts  = ecosoptimset('verbose', 0, 'feastol', 1e-5, 'abstol', 1e-2);
    
    
    opts = ecosoptimset('verbose', 0,...
                        'maxit', 100,...
                        'feastol', settings.feastol,...
                        'abstol', settings.abstol);
else % default settings 
    opts =  ecosoptimset('verbose', 0,...
                      'maxit', 100,... % default 50 
                       'feastol', 1e-5,...  % bigger can lead to issues
                        'reltol', 1e-3,...
                       'abstol', 5e-4);  % smaller can lead to issues

end 







[x, y, data, s, z] = ecos(c, G, h, dims, A_eq, b_eq, opts); 




result.x = x(1:end-1);   % NOTE: some dual info could be good, think on how to integrate this better 

% Exits Flags
%
% See Domahidi These, Sectinon 9.4.2, p. 169
% -3 : Multipliers leaving the cone 
% -1 : iteration limit 
% 0 : good sol
% 1 : Ceritificate of infeas
% 10 : Optimial solution found within reduced tolerance 

result.objval = inf;
if (data.exitflag == 0) 
    result.objval = data.pcost; 
elseif (data.exitflag == 10) 
    result.objval = data.pcost; 
elseif (data.exitflag == 1) %  % infeasibility proven 
elseif data.exitflag == -2 
    result.objval = data.pcost; 

    %warning(['ecos_solve:Numerical issues may have been encountered. ',...
    %                                 'Consider loosening ABSTOL']);
    %keyboard
else
    display(data)
end 
%else % TODO -- check 
%    keyboard%
%
%end 

% 1 - certi of primal infeasibility 


result.runtime = data.timing.runtime; % TODO -- comaptibility 

