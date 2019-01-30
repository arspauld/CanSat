function[xr, fx, ea, i] = bisect(f, xL, xu, et)  
% BISECT: bisection root locator that finds roots between xL and xu
% Inputs:
% f - function to be evaluated
% xL, xu - the lower and upper bounds
% et - allowable error
% Outputs:
% xr - estimated root value
% fx - function value at estimated root location
% ea - magnitude of apporximate relative error
% i - iterations of program

% Created by: Patrick Smith
% January 24, 2019
    
    if nargin < 3, error('At least 3 input arguments required'), end
    test = f(xL)*f(xu);
    if test > 0, error('No sign change f(xL) and f(xu)'), end
    if nargin < 4 || isempty(et), et=0.0001; end
    xr = xL; ea = 100;
    i = 0;
    while (1)
        i = i + 1;
        xrold = xr;
        xr = (xL + xu)/2;
        sgnchng = f(xL)*f(xr);
        if sgnchng < 0
            xu = xr;
            ea = abs((xr-xrold)/xr)*100;
        elseif sgnchng > 0
            xL = xr; 
            ea = abs((xr-xrold)/xr)*100;
        else
            ea = 0;
        end
        if ea < et
            break
        end
    end
    fx = f(xr);
end