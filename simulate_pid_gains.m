function out = simulate_pid_gains(p, g, t_end)
%SIMULATE_PID_GAINS Same PID loop as simulate_pid_baseline, but the 12
%   gains are an input so an optimiser can call it.
%
%   g = [Kp Ki Kd] for x, y, z, psi in that order (1x12 or 12x1).
%   Optional t_end shortens the run (used for the tuning/test split).
%
%   With g = baseline_gains() this reproduces the baseline result.

if nargin < 3 || isempty(t_end), t_end = p.Tf; end
g = g(:)';

t_vec = 0:p.Ts:t_end;
N = numel(t_vec);

x = [0; 0; -1; 0; 0; 0; 0; 0];
X_log = zeros(8, N);
U_log = zeros(4, N-1);
X_log(:, 1) = x;

Kx = g(1:3); Ky = g(4:6); Kz = g(7:9); Kpsi = g(10:12);
I_x = 0; I_y = 0; I_z = 0; I_psi = 0;
I_MAX = 5;   % same anti-windup clamp as the baseline

for k = 1:N-1
    t = t_vec(k);
    d_true = real_disturbance_profile(t, p);
    xref = reference_trajectory(t, p);

    ex = xref(1) - x(1);
    ey = xref(2) - x(2);
    ez = xref(3) - x(3);
    a = xref(7) - x(7);
    epsi = atan2(sin(a), cos(a));          % wrap to [-pi, pi]

    psi = x(7);
    ex_b =  cos(psi)*ex + sin(psi)*ey;
    ey_b = -sin(psi)*ex + cos(psi)*ey;

    I_x   = max(min(I_x   + ex_b*p.Ts, I_MAX), -I_MAX);
    I_y   = max(min(I_y   + ey_b*p.Ts, I_MAX), -I_MAX);
    I_z   = max(min(I_z   + ez*p.Ts,   I_MAX), -I_MAX);
    I_psi = max(min(I_psi + epsi*p.Ts, I_MAX), -I_MAX);

    X  = Kx(1)*ex_b + Kx(2)*I_x   - Kx(3)*x(4);
    Y  = Ky(1)*ey_b + Ky(2)*I_y   - Ky(3)*x(5);
    Z  = Kz(1)*ez   + Kz(2)*I_z   - Kz(3)*x(6);
    Mz = Kpsi(1)*epsi + Kpsi(2)*I_psi - Kpsi(3)*x(8);

    u_cmd = max(min([X; Y; Z; Mz], p.u_max), p.u_min);

    x = rk4_integrate(x, u_cmd, d_true, p, p.Ts);
    U_log(:, k) = u_cmd;
    X_log(:, k+1) = x;
end

Xref = reference_trajectory(t_vec, p);
out.t = t_vec;
out.X = X_log;
out.U = U_log;
out.pos_error = sqrt(sum((X_log(1:2, :) - Xref(1:2, :)).^2, 1));
out.rmse_pos = sqrt(mean(out.pos_error.^2));
out.ok = all(isfinite(X_log(:)));
end
