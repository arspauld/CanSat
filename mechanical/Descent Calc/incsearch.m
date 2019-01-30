function[nb, xb] = incsearch(func, xmin, xmax, ns)
% INCSEARCH: incremental search root loactor that finds roots between
% xmin and xmax evaluating ns points
% Inputs:
% func - function to be evaluated
% xmin, xmax - lower and upper bounds
% ns - number of points to evaluate
% Outputs:
% nb - number of roots found
% xb - list of the lower bound and upper bound that bracket the root

% Created by: Patrick Smith
% January 24, 2019

    if nargin < 3, error('Function, lower & upper bounds required'), end
    if nargin < 4, ns=50; end
    
    x = linspace(xmin, xmax, ns);
    f = func(x);
    
    nb = 0; xb = [];
    for i = 1:ns-1
        if sign(f(i)) ~= sign(f(i+1))
            nb = nb + 1;
            xb(nb,:) = [x(i) x(i+1)];
        end
    end
end