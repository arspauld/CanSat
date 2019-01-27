%% Importing and Fitting 
% First import and fit the data to a polynomial and create a function to
% use for obtain the coefficient of lift at every angle of attack 

nums = importdata('nums.txt');
alpha = nums(:,1)';
c_lift = nums(:,2)';
c_drag = nums(:,3)';

coef = polyfit(alpha, c_lift, 7);
c_l = @(t) coef(1)*t.^7+coef(2)*t.^6+coef(3)*t.^5+coef(4)*t.^4+coef(5)*t.^3+coef(6)*t.^2+coef(7)*t+coef(8);
coef = polyfit(alpha, c_drag, 7);
c_d = @(t) coef(1)*t.^7+coef(2)*t.^6+coef(3)*t.^5+coef(4)*t.^4+coef(5)*t.^3+coef(6)*t.^2+coef(7)*t+coef(8);
x = linspace(-8, 16);

subplot(221);
plot(alpha, c_lift, x, c_l(x));
title('\alpha versus C_L')
xlabel('\alpha [\circ]'); ylabel('C_L');
legend('Data', 'Fit', 'Location', 'northwest')

subplot(222);
plot(alpha, c_drag, x, c_d(x));
title('\alpha versus C_D')
xlabel('\alpha [\circ]'); ylabel('C_D');
legend('Data', 'Fit', 'Location', 'northwest')

subplot(223);
plot(x, c_l(x), x, c_d(x));
title('\alpha versus C_L and C_D')
xlabel('\alpha [\circ]'); ylabel('C_L and C_D');
legend('C_L', 'C_D', 'Location', 'northwest')

subplot(224);
plot(c_d(x), c_l(x));
title('C_D versus C_L')
xlabel('C_D'); ylabel('C_L');
%% Incrementing Over Length
r = 62;         % mm
L = 207;        % mm
m = .4;         % kg
g = 9.81;       % m / s^2
rho = 1.16;     % kg / m^3
V_y = 15;       % m / s


phi = @(h) 10 * h / L - 10; % degrees
w_i = 50;
w_f = 30;
w = @(h) (w_f-w_i)*h/L + w_i;    % mm


dh = .01;
spin = 0:.1:50;
rads = 2 * pi * spin;
lift_comp(1, length(spin)) = 0;
drag_comp(1, length(spin)) = 0; 
for i = 0:dh:L
    V_x = rads * (i + r) * .001; % m / s
    theta = - atand(V_y ./ V_x); % degrees
    p = phi(i); % phi
    alpha = p - theta; % degrees
    dA = w(i) * dh * .001^2; % m^2
    V_sq = V_x.^2 + V_y^2;
    
    lift_comp(1, :) = lift_comp(1, :) + rho * dA / 2 * V_sq .* c_l(alpha) .* cosd(theta);
    drag_comp(1, :) = drag_comp(1, :) + rho * dA / 2 * V_sq .* c_d(alpha) * V_y .* cosd(theta) ./ V_x;
end

weight = m * g * ones(1,length(lift_comp));
net_vert = 4 * lift_comp + 4 * drag_comp - weight;

figure(2);
subplot(131);
plot(spin*60, lift_comp);
grid on;
xlabel('Spin Rate [rpm]'); ylabel('Lift [N]');

subplot(132);
plot(spin*60, drag_comp);
grid on;
xlabel('Spin Rate [rpm]'); ylabel('Drag [N]');

subplot(133);
plot(spin*60, net_vert);
grid on;
xlabel('Spin Rate [rpm]'); ylabel('Net Force [N]');