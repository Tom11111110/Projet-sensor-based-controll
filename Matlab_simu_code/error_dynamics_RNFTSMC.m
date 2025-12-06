function dedt = error_dynamics_RNFTSMC(t, e, control_params, traj_params)
    % e = [e1; e2; e3; e4] - error vector
    e1 = e(1); e2 = e(2); e3 = e(3); e4 = e(4);

    % Trajectory reference and derivatives (xd, dxd, ddxd)
    [~, ~, dxd, dyd, ddxd, ddyd] = trajectory_reference(t, traj_params);

    % Extract controller parameters (same order as in main)
    mu1 = control_params(1); mu2 = control_params(2); mu3 = control_params(3); mu4 = control_params(4);
    Phi1 = control_params(5); Phi2 = control_params(6);
    Theta1 = control_params(7); Theta2 = control_params(8);
    kappa01 = control_params(9); kappa1 = control_params(10); kappa2 = control_params(11);
    kappa02 = control_params(12); kappa3 = control_params(13); kappa4 = control_params(14);
    a1 = control_params(15); a2 = control_params(16);
    z1 = control_params(17); z2 = control_params(18);
    epsilon = control_params(19);
    disturb_amp = control_params(20);
    tiny = control_params(21);

    % Sliding surface (NFTSM) - Equations (23) and (24)
    sigma_x = e1 + mu1 * abs(e1)^Phi1 * sign(e1) + mu2 * abs(e3)^Theta1 * sign(e3);
    sigma_y = e2 + mu3 * abs(e2)^Phi2 * sign(e2) + mu4 * abs(e4)^Theta2 * sign(e4);

    % Equivalent control veq (Equations 27 and 28)
    pow_e3 = abs(e3)^(2 - Theta1);
    factor_x = (1 + Phi1 * mu1 * (abs(e1) + tiny)^(Phi1 - 1));
    sgn_e3 = sign(e3);
    veqx = ddxd - (pow_e3 * factor_x * sgn_e3) / (Theta1 * mu2);

    pow_e4 = abs(e4)^(2 - Theta2);
    factor_y = (1 + Phi2 * mu3 * (abs(e2) + tiny)^(Phi2 - 1));
    sgn_e4 = sign(e4);
    veqy = ddyd - (pow_e4 * factor_y * sgn_e4) / (Theta2 * mu4);

    % Switching/robustness term (vsw) - Equations (29) and (30)
    % Using tanh to reduce chattering
    bound_rho_x = kappa01 + kappa1 * abs(e1) + kappa2 * abs(e3) + z1;
    bound_rho_y = kappa02 + kappa3 * abs(e2) + kappa4 * abs(e4) + z2;

    vswx = - a1 * sigma_x - bound_rho_x .* tanh(sigma_x / epsilon);
    vswy = - a2 * sigma_y - bound_rho_y .* tanh(sigma_y / epsilon);

    % Total control (desired accelerations) - Equations (31) and (32)
    v_x_cmd = veqx + vswx;
    v_y_cmd = veqy + vswy;

    %  External disturbances
    rho_x = disturb_amp * sin(0.5 * t) + disturb_amp * cos(0.7 * t);
    rho_y = rho_x;

    % Error dynamics (based on linearized state equations)
    de1dt = e3;
    de2dt = e4;
    de3dt = v_x_cmd - ddxd + rho_x;
    de4dt = v_y_cmd - ddyd + rho_y;

    dedt = [de1dt; de2dt; de3dt; de4dt];
end
