clear; close all; clc;

%% 1. Parameters Definition
R_circle = 2.0;
center_x = 0;
center_y = 0;
omega_r = 1.0;
traj_params = [R_circle, center_x, center_y, omega_r];
T_turn = 2 * pi / omega_r;

T_sim = 8;
tspan = [0 T_sim];

mu1 = 1.0; mu2 = 1.0; mu3 = 1.0; mu4 = 1.0;
Phi1 = 0.5; Phi2 = 0.5;
Theta1 = 1.5; Theta2 = 1.5;
kappa01 = 0.1; kappa1 = 0.1; kappa2 = 0.1;
kappa02 = 0.1; kappa3 = 0.1; kappa4 = 0.1;
a1 = 1.0; a2 = 1.0;
z1 = 0.1; z2 = 0.1;
epsilon = 0.05;
tiny = 1e-12;
disturb_amp = 1;
control_params = [mu1, mu2, mu3, mu4, Phi1, Phi2, Theta1, Theta2, ...
                  kappa01, kappa1, kappa2, kappa02, kappa3, kappa4, ...
                  a1, a2, z1, z2, epsilon, disturb_amp, tiny];

x0 = 1.1;
y0 = 0.2;
t0 = tspan(1);

[xd0, yd0, dxd0, dyd0, ~, ~] = trajectory_reference(t0, traj_params);

e0 = [x0 - xd0;
      y0 - yd0;
      0 - dxd0;
      0 - dyd0];

%% 2. RNFTSMC Simulation
[t, e_hist] = ode45(@(t, e) error_dynamics_RNFTSMC(t, e, control_params, traj_params), tspan, e0);

%% 3. Reconstruction, Sliding Surface and Plotting
%% Initialize
x_hist = zeros(length(t), 1);
y_hist = zeros(length(t), 1);
x_ref_hist = zeros(length(t),1);   % reference x
y_ref_hist = zeros(length(t),1);   % reference y
sigma_x_hist = zeros(length(t), 1);
sigma_y_hist = zeros(length(t), 1);
theta_hist = zeros(length(t),1);
theta_ref_hist = zeros(length(t),1);
v_x_hist = zeros(length(t),1);
v_y_hist = zeros(length(t),1);

for i = 1:length(t)
    [xd_ref, yd_ref, dxd_ref, dyd_ref, ddxd_ref, ddyd_ref] = trajectory_reference(t(i), traj_params);
    
    % Positions
    x_hist(i) = xd_ref + e_hist(i,1);
    y_hist(i) = yd_ref + e_hist(i,2);
    
    % Reference orientation
    theta_ref_hist(i) = atan2(dyd_ref, dxd_ref);
    
    % Actual orientation
    if i>1
        theta_hist(i) = atan2(y_hist(i)-y_hist(i-1), x_hist(i)-x_hist(i-1));
    else
        theta_hist(i) = theta_ref_hist(i);
    end
    
    % Sliding surfaces
    e1 = e_hist(i,1); e2 = e_hist(i,2); e3 = e_hist(i,3); e4 = e_hist(i,4);
    mu1 = control_params(1); mu2 = control_params(2); mu3 = control_params(3); mu4 = control_params(4);
    Phi1 = control_params(5); Phi2 = control_params(6);
    Theta1 = control_params(7); Theta2 = control_params(8);
    sigma_x_hist(i) = e1 + mu1*abs(e1)^Phi1*sign(e1) + mu2*abs(e3)^Theta1*sign(e3);
    sigma_y_hist(i) = e2 + mu3*abs(e2)^Phi2*sign(e2) + mu4*abs(e4)^Theta2*sign(e4);
    
    % Control signals (v_x, v_y)
    pow_e3 = abs(e3)^(2 - Theta1);
    factor_x = (1 + Phi1 * mu1 * (abs(e1) + 1e-12)^(Phi1 - 1));
    veqx = ddxd_ref - (pow_e3 * factor_x * sign(e3)) / (Theta1 * mu2);

    pow_e4 = abs(e4)^(2 - Theta2);
    factor_y = (1 + Phi2 * mu3 * (abs(e2) + 1e-12)^(Phi2 - 1));
    veqy = ddyd_ref - (pow_e4 * factor_y * sign(e4)) / (Theta2 * mu4);

    kappa01 = control_params(9); kappa1 = control_params(10); kappa2 = control_params(11);
    kappa02 = control_params(12); kappa3 = control_params(13); kappa4 = control_params(14);
    a1 = control_params(15); a2 = control_params(16);
    z1 = control_params(17); z2 = control_params(18);
    epsilon = control_params(19);

    bound_rho_x = kappa01 + kappa1*abs(e1) + kappa2*abs(e3) + z1;
    bound_rho_y = kappa02 + kappa3*abs(e2) + kappa4*abs(e4) + z2;

    vswx = - a1 * sigma_x_hist(i) - bound_rho_x .* tanh(sigma_x_hist(i) / epsilon);
    vswy = - a2 * sigma_y_hist(i) - bound_rho_y .* tanh(sigma_y_hist(i) / epsilon);

    v_x_hist(i) = veqx + vswx;
    v_y_hist(i) = veqy + vswy;
end

%% Position plot: robot vs reference
figure(1); hold on;
plot(x_hist, y_hist, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Trajectory');
plot(x_ref_hist, y_ref_hist, 'r--', 'LineWidth', 3, 'DisplayName', 'Reference Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on; legend show; title('Robot vs Reference Trajectory');

%% Error plot
figure(2); hold on;
plot(t, e_hist(:,1), 'r', 'LineWidth', 1.5);
plot(t, e_hist(:,2), 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (m)'); grid on;
legend('$e_x$','$e_y$','Interpreter','latex'); title('Position Errors');

%% Sliding surface plot
figure(3); hold on;
plot(t, sigma_x_hist, 'r', 'LineWidth', 1.5);
plot(t, sigma_y_hist, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\sigma'); grid on;
legend('\sigma_x','\sigma_y'); title('Sliding Surfaces vs Time');

%% Theta plot
figure(4); hold on;
plot(t, theta_hist, 'b', 'LineWidth', 1.5);
plot(t, theta_ref_hist, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (rad)'); grid on;
legend('Robot θ','Reference θ'); title('Orientation vs Reference');

figure(5); hold on;
plot(t, v_x_hist, 'b', 'LineWidth', 1.5);
plot(t, v_y_hist, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Control Input'); grid on;
legend('v_x','v_y'); title('Control Inputs vs Time');
