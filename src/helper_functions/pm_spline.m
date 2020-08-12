%*********************************
%   Function:
%       pm_spline.m
%
%   Generates piecewise cubic spline curve from nodes 
%
%   p - value at spline points
%   m - slope at spline points
%   t_node - time at nodes
%   t_test - time vector for generating spline curve
%
%   Erez Krimsky, Stanford University Biomechatronics Lab
%   ekrimsky@stanford.edu
%
%
%*********************************


function y_out = pm_spline(p, m, t_node, t_test)
    % TODO -- could add checks on validity of inputs 


    h00 = @(x) 2*x.^3 - 3*x.^2 + 1; 
    h10 = @(x) x.^3 - 2*x.^2 + x; 
    h01 = @(x) -2*x.^3 + 3*x.^2; 
    h11 = @(x) x.^3 - x.^2; 

    % TODO -- add check for out of bound times 

    y_out = 0 * t_test; % initialize 

    ta = t_node(1); 
    tb = t_node(2); 
    idx = 1; 

    h = length(t_test); 

    for t_idx = 1:h

        t_cur = t_test(t_idx); 

        if (t_cur > tb)
            idx = idx + 1; 
            ta = tb;
            tb = t_node(idx + 1); 
        end 

        
        t_map = (t_cur - ta)/(tb - ta);

        y_out(t_idx) = h00(t_map)*p(idx) + h10(t_map)*(tb-ta)*m(idx) +...
                        h01(t_map)*p(idx+1) + h11(t_map)*(tb-ta)*m(idx+1); 

    end 

end  