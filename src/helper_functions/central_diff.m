
function v = central_diff(x, t) 

	if (size(t, 1) ~= size(x, 1)) && (size(t, 2) ~= size(x, 2))
		t = t';
	end 


    v = 0*x; 
    h = t(3:end) - t(1:end-2); 

    % do central diff 
    v(2:end-1) = (x(3:end) - x(1:end-2))./h;
    v(1) = (x(2) - x(1))/(t(2) - t(1));
    v(end) = (x(end) - x(end - 1))/(t(end) - t(end-1));  
end 