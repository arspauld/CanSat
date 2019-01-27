%% Importing and Fitting 
% First import and fit the data to a polynomial and create a function to
% use for obtain the coefficient of lift at every angle of attack

nums = importdata('nums.txt');
alpha = nums(:,1)';
naca6409 = nums(:,4)';

coef = polyfit(alpha, naca6409, 7);
f = @(t) coef(1)*t.^7+coef(2)*t.^6+coef(3)*t.^5+coef(4)*t.^4+coef(5)*t.^3+coef(6)*t.^2+coef(7)*t+coef(8);
x = linspace(-8, 16);

figure(1)
plot(alpha, naca6409); hold on
plot(x, f(x));
title('\alpha versus C_L')
xlabel('\alpha'); ylabel('C_L');
legend('Data', 'Fit', 'Location', 'northwest')

%% Incrementing Over Length
L = 8;
l = linspace(0,L);
a = @(h) 5*h/4 - 10;
c_lave = mean(f(a(l)))