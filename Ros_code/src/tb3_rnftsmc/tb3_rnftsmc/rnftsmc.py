# tb3_rnftsmc/rnftsmc.py
import numpy as np

def safe_power(x, p, max_val=1e6):
    """Calcul x**p en limitant la valeur maximale pour éviter les overflow."""
    try:
        val = np.sign(x) * min(abs(x)**p, max_val)
    except OverflowError:
        val = np.sign(x) * max_val
    return val



def sign(x):
    return np.sign(x) if np.abs(x) > 0 else 0.0

def error_dynamics_rnftsmc(t, e, control_params, traj_params):
    e1, e2, e3, e4 = float(e[0]), float(e[1]), float(e[2]), float(e[3])
    # unpack params (dict expected)
    mu1 = control_params['mu1']; mu2 = control_params['mu2']; mu3 = control_params['mu3']; mu4 = control_params['mu4']
    Phi1 = control_params['Phi1']; Phi2 = control_params['Phi2']
    Theta1 = control_params['Theta1']; Theta2 = control_params['Theta2']
    kappa01 = control_params['kappa01']; kappa1 = control_params['kappa1']; kappa2 = control_params['kappa2']
    kappa02 = control_params['kappa02']; kappa3 = control_params['kappa3']; kappa4 = control_params['kappa4']
    a1 = control_params['a1']; a2 = control_params['a2']
    z1 = control_params['z1']; z2 = control_params['z2']
    epsilon = control_params['epsilon']
    disturb_amp = control_params.get('disturb_amp', 0.2)
    tiny = control_params.get('tiny', 1e-12)

    # trajectory derivatives
    xd, yd, dxd, dyd, ddxd, ddyd = None, None, None, None, None, None
    xd, yd, dxd, dyd, ddxd, ddyd = __trajectory_wrapper(t, traj_params)

    # sliding surfaces
    sigma_x = e1 + mu1 * safe_power(e1, Phi1) + mu2 * safe_power(e3, Theta1)
    sigma_y = e2 + mu3 * safe_power(e2, Phi2) + mu4 * safe_power(e4, Theta2)

    # equivalent command accelerations
    pow_e3 = abs(e3)**(2 - Theta1) if Theta1 != 2 else 1.0
    factor_x = (1 + Phi1 * mu1 * (abs(e1) + tiny)**(Phi1 - 1))
    sgn_e3 = sign(e3)
    veqx = ddxd - (pow_e3 * factor_x * sgn_e3) / (Theta1 * mu2 + tiny)

    pow_e4 = abs(e4)**(2 - Theta2) if Theta2 != 2 else 1.0
    factor_y = (1 + Phi2 * mu3 * (abs(e2) + tiny)**(Phi2 - 1))
    sgn_e4 = sign(e4)
    veqy = ddyd - (pow_e4 * factor_y * sgn_e4) / (Theta2 * mu4 + tiny)

    # switching / robustness
    bound_rho_x = kappa01 + kappa1 * abs(e1) + kappa2 * abs(e3) + z1
    bound_rho_y = kappa02 + kappa3 * abs(e2) + kappa4 * abs(e4) + z2

    vswx = - a1 * sigma_x - bound_rho_x * np.tanh(sigma_x / (epsilon + tiny))
    vswy = - a2 * sigma_y - bound_rho_y * np.tanh(sigma_y / (epsilon + tiny))

    # total commanded accelerations (world frame)
    v_x_cmd = veqx + vswx
    v_y_cmd = veqy + vswy

    # disturbances
    rho_x = disturb_amp * np.sin(0.5 * t) + disturb_amp * np.cos(0.7 * t)
    rho_y = rho_x

    de1dt = e3
    de2dt = e4
    de3dt = v_x_cmd - ddxd + rho_x
    de4dt = v_y_cmd - ddyd + rho_y

    return np.array([de1dt, de2dt, de3dt, de4dt], dtype=float)

def return_command_accelerations(t, e, control_params, traj_params):
    # replicate the computation for v_x_cmd and v_y_cmd
    e1, e2, e3, e4 = float(e[0]), float(e[1]), float(e[2]), float(e[3])
    mu1 = control_params['mu1']; mu2 = control_params['mu2']; mu3 = control_params['mu3']; mu4 = control_params['mu4']
    Phi1 = control_params['Phi1']; Phi2 = control_params['Phi2']
    Theta1 = control_params['Theta1']; Theta2 = control_params['Theta2']
    kappa01 = control_params['kappa01']; kappa1 = control_params['kappa1']; kappa2 = control_params['kappa2']
    kappa02 = control_params['kappa02']; kappa3 = control_params['kappa3']; kappa4 = control_params['kappa4']
    a1 = control_params['a1']; a2 = control_params['a2']
    z1 = control_params['z1']; z2 = control_params['z2']
    epsilon = control_params['epsilon']
    tiny = control_params.get('tiny', 1e-12)
    disturb_amp = control_params.get('disturb_amp', 0.2)

    xd, yd, dxd, dyd, ddxd, ddyd = __trajectory_wrapper(t, traj_params)

    def sgn(u):
        return np.sign(u) if abs(u) > 0 else 0.0

    sigma_x = e1 + mu1 * (abs(e1)**Phi1) * sgn(e1) + mu2 * (abs(e3)**Theta1) * sgn(e3)
    sigma_y = e2 + mu3 * (abs(e2)**Phi2) * sgn(e2) + mu4 * (abs(e4)**Theta2) * sgn(e4)

    pow_e3 = abs(e3)**(2 - Theta1) if Theta1 != 2 else 1.0
    factor_x = (1 + Phi1 * mu1 * (abs(e1) + tiny)**(Phi1 - 1))
    sgn_e3 = sgn(e3)
    veqx = ddxd - (pow_e3 * factor_x * sgn_e3) / (Theta1 * mu2 + tiny)

    pow_e4 = abs(e4)**(2 - Theta2) if Theta2 != 2 else 1.0
    factor_y = (1 + Phi2 * mu3 * (abs(e2) + tiny)**(Phi2 - 1))
    sgn_e4 = sgn(e4)
    veqy = ddyd - (pow_e4 * factor_y * sgn_e4) / (Theta2 * mu4 + tiny)

    bound_rho_x = kappa01 + kappa1 * abs(e1) + kappa2 * abs(e3) + z1
    bound_rho_y = kappa02 + kappa3 * abs(e2) + kappa4 * abs(e4) + z2

    vswx = - a1 * sigma_x - bound_rho_x * np.tanh(sigma_x / (epsilon + tiny))
    vswy = - a2 * sigma_y - bound_rho_y * np.tanh(sigma_y / (epsilon + tiny))

    v_x_cmd = veqx + vswx
    v_y_cmd = veqy + vswy

    return float(v_x_cmd), float(v_y_cmd)

# small wrapper to compute trajectory
def __trajectory_wrapper(t, traj_params):
    R = traj_params['R_cercle']
    cx = traj_params['centre_x']
    cy = traj_params['centre_y']
    omega = traj_params['omega_r']
    xd = cx + R * np.cos(omega * t)
    yd = cy + R * np.sin(omega * t)
    dxd = -R * omega * np.sin(omega * t)
    dyd = R * omega * np.cos(omega * t)
    ddxd = -R * (omega**2) * np.cos(omega * t)
    ddyd = -R * (omega**2) * np.sin(omega * t)
    return xd, yd, dxd, dyd, ddxd, ddyd