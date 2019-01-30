%% Importing and Fitting 
% First import and fit the data to a polynomial and create a function to
% use for obtain the coefficient of lift at every angle of attack 

nums = importdata('xfoil.txt');
alpha = nums(:,1)';
c_lift = nums(:,2)';
c_drag = nums(:,3)';

deg = 6;
coef = polyfit(alpha, c_lift, deg);
coef = coef(deg+1:-1:1);
c_l = @(t) coef(7)*t.^6+coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);

coef = polyfit(alpha, c_drag, deg);
coef = coef(deg+1:-1:1);
c_d = @(t) coef(7)*t.^6+coef(6)*t.^5+coef(5)*t.^4+coef(4)*t.^3+coef(3)*t.^2+coef(2)*t+coef(1);
x = linspace(-35, 35);

figure(1);
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
I = .0003;      % moment of inertia
m = .4;         % kg
g = 9.81;       % m / s^2
rho = 1.16;     % kg / m^3


phi = @(h) 10 * h / L - 10; % degrees
w_i = 50;
w_f = 30;
w = @(h) (w_f-w_i)*h/L + w_i;    % mm

dh = 1;
dt = .01;
V_y = -20;
omega = 0;
dat = 0;
% models the drop
for t = 0:dt:1
    dat = dat + 1; 
    F = 0;
    T = 0;
    % gets net forces and torques
    for h = 0:dh:L
        fprintf('Time: %g\tH: %g\n', t, h);
        V_x = omega * (h + r) * .001;
        theta = atand(V_y ./ V_x); % degrees
        p = phi(h); % phi
        alpha = p - theta; % degrees
        dA = w(h) * dh * .001^2; % m^2
        V = sqrt(V_x.^2 + V_y^2); 
        
        tau = rho * dA * V^2 / 2;
        L = tau * c_l(alpha);
        D = tau * c_d(alpha);
        
        beta = 90 + theta;
        L_x = L * cosd(beta);
        L_y = L * sind(beta);
        D_x = D * cosd(beta + 90);
        D_y = D * sind(beta + 90);
        
        T = T + (L_x + D_x) * (h + r) * .001;
        F = F + L_y + D_y;
    end
    D_payload = -sign(V_y) * 0.81 * rho * V_y^2 / 2 * .002353;
    F = 4*F + D_payload - m*g; % Adjusts for four blades
    T = 2*T; % adjusts for two blades on rotor
    omega = omega + T / I * dt; 
    V_y = V_y + F / m * dt;
    
    vert(1, dat) = V_y;
    time(1, dat) = t+dt;
    rps(1, dat) = omega;
    
%     disp(V_y)
%     resp = input('Continue? (y/n): ', 's');
%     if isempty(resp), resp = 'y'; end
%     if resp ~= 'y'
%         break
%     end
end

figure(2);
plot(time, vert);
grid on;
xlabel('Time [s]'); ylabel('Vertical Velocity [^m/_s]');

figure(3);
plot(time, rps);
grid on;
xlabel('Time [s]'); ylabel('Angular Velocity [^{rads}/_s]');

%% Solving for Intersection
%rpm_coef