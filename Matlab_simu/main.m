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
disturb_amp = 0.2;
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

%% 3. Reconstruction and Plotting
if isempty(e_hist)
    disp('ERROR: RNFTSMC simulation failed');
    return;
end

x_hist = zeros(length(t), 1);
y_hist = zeros(length(t), 1);
for i = 1:length(t)
    [xd_ref, yd_ref, ~, ~, ~, ~] = trajectory_reference(t(i), traj_params);
    x_hist(i) = xd_ref + e_hist(i, 1);
    y_hist(i) = yd_ref + e_hist(i, 2);
end

figure(1); hold on;
title('RNFTSMC Simulation: Circular Trajectory Tracking with Disturbances');
xlabel('Position X (m)');
ylabel('Position Y (m)');
axis equal; grid on;

t_ref_one_turn = linspace(tspan(1), T_turn, 500); 
[x_ref, y_ref] = arrayfun(@(tau) trajectory_reference(tau, traj_params), t_ref_one_turn);
plot(x_ref, y_ref, 'r--', 'LineWidth', 3, 'DisplayName', 'Reference Trajectory');

plot(x_hist, y_hist, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');
plot(x0, y0, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Start');
legend show;

figure(2);
plot(t, e_hist(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, e_hist(:,2), 'g', 'LineWidth', 1.5);
title('Position Errors (e_x, e_y) - RNFTSMC');
xlabel('Time (s)'); ylabel('Error (m)');
legend('$e_x$','$e_y$','Interpreter','latex'); grid on;
