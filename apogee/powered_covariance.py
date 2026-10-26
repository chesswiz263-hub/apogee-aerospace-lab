"""Approximate attitude-to-translation covariance injection for powered flight.

This is a conditional, locally linearized covariance augmentation, NOT a
joint (position, attitude) EKF. Over each hold interval an uncertain rotation
changes the thrust acceleration through delta_a = -[a]_x delta_theta.
Unmodeled temporal cross-correlation is not represented. Report this limitation.
"""
import numpy as np
from .precision_adcs import cross_matrix, symmetric_psd


def powered_attitude_discrete_cov(accel_km_s2, interval_s, attitude_3axis_sigma_deg):
    a=np.asarray(accel_km_s2,dtype=float)
    dt=float(interval_s);sig=float(attitude_3axis_sigma_deg)
    if a.shape!=(3,) or not np.isfinite(a).all() or not np.isfinite([dt,sig]).all() or dt<0 or sig<0:
        raise ValueError('Invalid attitude covariance inputs')
    # Conservative isotropic surrogate that matches reported RSS attitude sigma.
    sigma_axis=np.deg2rad(sig)/np.sqrt(3.)
    J=-cross_matrix(a)
    D=np.vstack((.5*dt*dt*J,dt*J))
    return symmetric_psd((sigma_axis*sigma_axis)*(D@D.T))
