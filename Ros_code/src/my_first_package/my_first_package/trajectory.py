import numpy as np

def trajectory_reference(t, traj_params):
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