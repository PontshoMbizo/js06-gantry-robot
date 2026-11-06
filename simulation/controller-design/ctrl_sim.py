"""Batch simulator of the firmware 2.2 axis controller on an identified plant.

All scenarios are simulated at once (numpy vectors), one scenario per element.

Controller (ESP32-C3, 5 ms update), as described in the firmware notes:
    e   = D - n*slot                      (position from the pulse count, mm)
    v^  = slot / max(last tick interval, time since last tick)
    u   = clamp(V0 + Kp*e - Kd*v^ + nudge, 0, Vlim)     (one-sided: never negative)
    stop (brake) as soon as e <= stop_band * slot
    u is slew limited, nudge grows while no tick arrives.

Plant (identified in report_method_analyses_new_tests):
    tau dv/dt = K*max(u(t-delay) - Vdb, 0) - v,  dx/dt = v   (with breakaway Vdb+dVb from rest)
    optional knee (Z): above Vk the speed gain per volt is K2 instead of K
    a pulse is emitted every `slot` mm of travel; braking decays v with tau*brake_factor.
"""
import numpy as np

DT = 1e-3          # plant step, s
TC = 5e-3          # controller period, s
NC = 5             # plant steps per control update


def simulate(P, t_max=20.0, record=0, slew=50.0, nudge_rate=2.0, nudge_after=0.15,
             stop_band=1.0, brake_factor=1.0, rest_mm=0.1):
    """P: dict of equally long 1-D arrays (K, tau, Vdb, dVb, delay_ms, D, frac0, slot,
    Kp, Kd, V0, Vlim; optional Vk, K2). Returns dict of result arrays (and trajectories if record>0)."""
    K = np.asarray(P['K'], float); N = K.size
    tau = np.asarray(P['tau'], float); Vdb = np.asarray(P['Vdb'], float)
    dVb = np.asarray(P['dVb'], float); D = np.asarray(P['D'], float)
    frac0 = np.asarray(P['frac0'], float); slot = np.asarray(P['slot'], float)
    Kp = np.asarray(P['Kp'], float); Kd = np.asarray(P['Kd'], float)
    V0 = np.asarray(P['V0'], float); Vlim = np.asarray(P['Vlim'], float)
    Vk = np.asarray(P['Vk'], float) if 'Vk' in P else np.full(N, np.inf)      # optional knee: above Vk the speed gain falls to K2
    K2 = np.asarray(P['K2'], float) if 'K2' in P else K
    dly = np.maximum(np.round(np.asarray(P['delay_ms'], float) / (DT * 1e3)).astype(int), 0)
    L = int(dly.max()) + 2
    hist = np.zeros((N, L)); ar = np.arange(N)

    x = np.zeros(N); v = np.zeros(N)
    n = np.zeros(N, int)
    t_last = np.zeros(N); dt_last = np.full(N, np.nan)
    u_cmd = np.zeros(N); nudge = np.zeros(N)
    stopped = np.zeros(N, bool); t_stop = np.full(N, np.nan)
    rest = np.zeros(N, bool); t_rest = np.full(N, np.nan)
    umax_app = np.zeros(N)
    ticks_t = []  # unused
    steps = int(round(t_max / DT))
    if record:
        nrec = steps // NC + 1
        R = {k: np.full((record, nrec), np.nan) for k in ('x', 'v', 'u', 'vhat', 'n')}
    a_brake = np.exp(-DT / (tau * brake_factor))
    a_run = np.exp(-DT / tau)
    k_rec = 0
    for s in range(steps):
        t = s * DT
        if s % NC == 0:
            tn = t
            elapsed = tn - t_last
            iv = np.where(np.isnan(dt_last), np.where(n > 0, elapsed, np.inf), np.maximum(dt_last, elapsed))
            vhat = np.where(np.isfinite(iv), slot / np.maximum(iv, 1e-6), 0.0)
            e = D - n * slot
            newstop = (~stopped) & (e <= stop_band * slot)
            t_stop = np.where(newstop, tn, t_stop)
            stopped |= newstop
            since = tn - t_last
            nudge = np.where(since > nudge_after, nudge + nudge_rate * TC, 0.0)
            raw = np.clip(V0 + Kp * e - Kd * vhat + nudge, 0.0, Vlim)
            u_new = np.clip(raw, u_cmd - slew * TC, u_cmd + slew * TC)
            u_cmd = np.where(stopped, 0.0, u_new)
            if record and k_rec < nrec:
                R['x'][:, k_rec] = x[:record]; R['v'][:, k_rec] = v[:record]
                R['u'][:, k_rec] = np.where(stopped, 0.0, u_cmd)[:record]
                R['vhat'][:, k_rec] = vhat[:record]; R['n'][:, k_rec] = n[:record]
                k_rec += 1
        hist[:, s % L] = u_cmd
        u_eff = hist[ar, (s - dly) % L] if s >= 1 else np.zeros(N)
        u_eff = np.where(s - dly < 0, 0.0, u_eff)
        u_eff = np.where(stopped, 0.0, u_eff)
        umax_app = np.maximum(umax_app, u_eff)
        enable = (v > 0.2) | (u_eff > Vdb + dVb)
        vss = np.where(stopped, 0.0, (K * np.maximum(np.minimum(u_eff, Vk) - Vdb, 0.0) + K2 * np.maximum(u_eff - Vk, 0.0)) * enable)
        a = np.where(stopped, a_brake, a_run)
        tau_e = np.where(stopped, tau * brake_factor, tau)
        v_new = vss + (v - vss) * a
        x_new = x + vss * DT + (v - vss) * tau_e * (1 - a)
        # tick detection (counts are credited in the driven direction, so only forward here)
        p_old = x + frac0 * slot; p_new = x_new + frac0 * slot
        n_new = np.floor(p_new / slot + 1e-12).astype(int)
        tick = n_new > n
        if tick.any():
            bound = n_new * slot
            frac_t = np.where(p_new > p_old, (bound - p_old) / np.maximum(p_new - p_old, 1e-12), 1.0)
            tt = t + DT * np.clip(frac_t, 0, 1)
            dt_last = np.where(tick, tt - t_last, dt_last)
            t_last = np.where(tick, tt, t_last)
            nudge = np.where(tick, 0.0, nudge)
        n = np.where(tick, n_new, n)
        x, v = x_new, v_new
        newrest = stopped & (~rest) & (v * tau * brake_factor < rest_mm)
        t_rest = np.where(newrest, t + DT, t_rest)
        rest |= newrest
        if rest.all():
            break
    # finish the (tiny) remaining coast analytically
    x_end = np.where(stopped, x + v * tau * brake_factor, x)
    n_end = np.floor((x_end + frac0 * slot) / slot + 1e-12).astype(int)
    out = dict(x_final=x_end, n_final=n_end, t_stop=t_stop, t_rest=t_rest, done=rest, u_max=umax_app,
               err_fb=D - n_end * slot,                    # error seen by the feedback (counts)
               over_fb=np.maximum(n_end * slot - D, 0.0),  # overshoot seen by the feedback
               err_true=D - (x_end + frac0 * slot),        # error of the true carriage position
               over_true=np.maximum(x_end + frac0 * slot - D, 0.0))
    if record:
        out['traj'] = R
        out['t_rec'] = np.arange(nrec) * TC
    return out
