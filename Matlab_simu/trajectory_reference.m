function [xd, yd, dxd, dyd, ddxd, ddyd] = trajectory_reference(t, traj_params)
    R_circle = traj_params(1);
    center_x = traj_params(2);
    center_y = traj_params(3);
    omega_r = traj_params(4);

    xd = center_x + R_circle * cos(omega_r * t);
    yd = center_y + R_circle * sin(omega_r * t);

    dxd = -R_circle * omega_r * sin(omega_r * t);
    dyd = R_circle * omega_r * cos(omega_r * t);

    ddxd = -R_circle * (omega_r^2) * cos(omega_r * t);
    ddyd = -R_circle * (omega_r^2) * sin(omega_r * t);
end
