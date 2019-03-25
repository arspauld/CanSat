%% Importing and Fitting 
% First import and fit the data to a polynomial and create a function to
% use for obtain the coefficient of lift at every angle of attack 

% imports the data from a text file
nums = importdata('xfoil.txt');

alpha = nums(:,1)';
a = alpha <= 10;
b = alpha >= -10;
indices = logical(a .* b);
alpha_small = alpha(indices);

c_lift = nums(:,2)';
c_l_small = c_lift(indices);

c_drag = nums(:,3)';
c_d_small = c_drag(indices);


% fits the coefficient tables witha polynomial to graph them at all angles
% of attack
deg = 6;
coef = polyfit(alpha, c_lift, deg);
coef = coef(deg+1:-1:1);
c_l = @(t) coef(7)*t.^6+coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);

coef = polyfit(alpha_small, c_l_small, 5);
coef = coef(6:-1:1);
c_ls = @(t) coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);

coef = polyfit(alpha, c_drag, deg);
coef = coef(deg+1:-1:1);
c_d = @(t) coef(7)*t.^6+coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);

coef = polyfit(alpha_small, c_d_small, 5);
coef = coef(6:-1:1);
c_ds = @(t) coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);

x = linspace(-35, 35);
x_s = linspace(-10, 10);


% Plots the data from the fits
figure(1);
subplot(221);
plot(alpha, c_lift, '*', x, c_l(x), x_s, c_ls(x_s));
title('\alpha versus C_L')
xlabel('\alpha [\circ]'); ylabel('C_L');
legend('Data', 'Fit', 'Small Fit', 'Location', 'northwest')

subplot(222);
plot(alpha, c_drag, '*', x, c_d(x), x_s, c_ds(x_s));
title('\alpha versus C_D')
xlabel('\alpha [\circ]'); ylabel('C_D');
legend('Data', 'Fit', 'Small Fit', 'Location', 'northwest')

subplot(223);
plot(x, c_l(x), x, c_d(x));
title('\alpha versus C_L and C_D')
xlabel('\alpha [\circ]'); ylabel('C_L and C_D');
legend('C_L', 'C_D', 'Location', 'northwest')

subplot(224);
plot(c_d(x), c_l(x));
title('C_D versus C_L')
xlabel('C_D'); ylabel('C_L');



%% Incrementing Over Time
r = 62;         % mm
L = 207;        % mm
I = .0003;      % moment of inertia
m = .4;         % kg
g = 9.81;       % m / s^2
rho = 1.1226;   % kg / m^3 at Standard 900 m


a_i = 10;
twist = 10;
phi = @(h) twist * h / L  - a_i; % degrees

w_i = 50;
w_f = 30;
w = @(h) (w_f-w_i)*h/L + w_i;    % mm

dh = 1;
dt = .001;
V_y = -20; % starting velocity does not matter as it will go to same end value
omega = 0;
dat = 0;
T = 0;
F = 0;

% models the drop
dx = 0;
h = 0:dh:L;
for t = 0:dt:100
    dat = dat + 1; 
    vert(1, dat) = V_y;
    time(1, dat) = t+dt;
    rps(1, dat) = omega;
    force(1, dat) = F;
    torque(1,dat) = T;
    
    F = 0;
    T = 0;
    % gets net forces and torques
    V_x = omega .* (h+r) * .001;    % m / s
    theta = atand(V_y ./ V_x);      % angle of the aiflow
    p = phi(h);                     % angle of incidence
    alpha = p - theta;              % angle of attack, corrected for the coordinate system
    dA = w(h) * dh * .001^2;        % m^2
    V = sqrt(V_x.^2 + V_y^2);       % m / s
    
    % separates the smaller alpha values
    a = alpha <= 10;
    b = alpha >= -10;
    inds = logical(a .* b);
    
    tau = rho * dA .* V.^2 / 2;     % common coefficient of lift and drag calcs
    L = tau .* (c_l(alpha.*~inds) + c_ls(alpha.*inds));          % Lift
    D = tau .* (c_d(alpha.*~inds) + c_ds(alpha.*inds));          % Drag

    
    beta = 90 + theta;              % new angle set in the 1st quadrant
    L_x = L .* cosd(beta);          % x component of lift
    L_y = L .* sind(beta);          % y component of lift
    D_x = D .* cosd(beta + 90);     % x component of drag
    D_y = D .* sind(beta + 90);     % y component of drag
      
    T = T + sum((L_x + D_x) .* (h + r) * .001);     % net force of one blade
    F = F + sum(L_y + D_y);     % net torque of one blade

    D_payload = -sign(V_y) * 0.82 * rho * V_y^2 / 2 * .002353; % drag of the can body
    
    F = 4*F + D_payload - m*g;  % Adjusts for four blades on payload
    T = 2*T;                    % Adjusts for two blades on rotor
    omega = omega + T / I * dt; % updates angular velocity
    V_old = V_y;
    V_y = V_y + F / m * dt;     % updates velocity
 
    if abs(V_y-V_old) >= 0.1 * dt, dx = dx + V_old * dt + F/m/2*dt^2; end
end
fprintf('Fell:    %7.3f\nSpeed: %9.3f\nRPM:   %9.3f\n', dx, V_y, omega * 60 / 2 / pi);

figure(2);
subplot(221)
plot(time, vert, time, V_y * ones(1, length(time)));
grid on;
xlabel('Time [s]'); ylabel('Vertical Velocity [^m/_s]');

subplot(222)
plot(time, rps);
grid on;
xlabel('Time [s]'); ylabel('Angular Velocity [^{rads}/_s]');

subplot(223)
plot(time, force);
grid on;
xlabel('Time [s]'); ylabel('Force [N]');

subplot(224)
plot(time, torque);
grid on;
xlabel('Time [s]'); ylabel('Torque [N\bulletm]');