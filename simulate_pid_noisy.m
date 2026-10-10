function out = simulate_pid_noisy(p, g, opts)
%SIMULATE_PID_NOISY PID loop with realistic imperfections.
%   Same control law as simulate_pid_gains, but:
%     - the controller only sees a NOISY measurement of the state
%     - thruster commands go through a first-order lag
%   The vehicle's true motion and the reported error stay exact.
%
%   g    : 1x12 gains [Kp Ki Kd] for x, y, z, psi
%   opts : struct, all fields optional
%       .vel_noise  std on [u v w r]   (default 0.01, same as the MPC study)
%       .pos_noise  std on x, y, z [m] (default 0.02)
%       .yaw_noise  std on psi [rad]   (default 0.01)
%       .tau        thruster lag [s]   (default 0.3, an assumption)
%       .seed       RNG seed           (default 0)
%       .t_end      run length [s]     (default p.Tf)
%
%   Set vel_noise = pos_noise = yaw_noise = 0 and tau = 0 to get the
%   ideal-conditions result back.

if nargin < 3, opts = struct(); end
opts = fill_defaults(opts, p);
g = g(:)';

stream = RandStream('mt19937ar', 'Seed', opts.seed);

t_vec = 0:p.Ts:opts.t_end;
N = numel(t_vec);

x = [0; 0; -1; 0; 0; 0; 0; 0];          % true state
X_log = zeros(8, N);
U_log = zeros(4, N-1);
X_log(:, 1) = x;

Kx = g(1:3); Ky = g(4:6); Kz = g(7:9); Kpsi = g(10:12);
I_x = 0; I_y = 0; I_z = 0; I_psi = 0;
I_MAX = 5;

if opts.tau > 0
    a_lag = 1 - exp(-p.Ts / opts.tau);
else
    a_lag = 1;
end
u_act = zeros(4, 1);

sd = zeros(8, 1);
sd([1 2 3]) = opts.pos_noise;
sd(7)       = opts.yaw_noise;
sd([4 5 6 8]) = opts.vel_noise;

for k = 1:N-1
    t = t_vec(k);
    d_true = real_disturbance_profile(t, p);
    xref = reference_trajectory(t, p);

    % what the controller actually sees
    xm = x + sd .* randn(stream, 8, 1);

    ex = xref(1) - xm(1);
    ey = xref(2) - xm(2);
    ez = xref(3) - xm(3);
    a = xref(7) - xm(7);
    epsi = atan2(sin(a), cos(a));

    psi = xm(7);
    ex_b =  cos(psi)*ex + sin(psi)*ey;
    ey_b = -sin(psi)*ex + cos(psi)*ey;

    I_x   = max(min(I_x   + ex_b*p.Ts, I_MAX), -I_MAX);
    I_y   = max(min(I_y   + ey_b*p.Ts, I_MAX), -I_MAX);
    I_z   = max(min(I_z   + ez*p.Ts,   I_MAX), -I_MAX);
    I_psi = max(min(I_psi + epsi*p.Ts, I_MAX), -I_MAX);

    X  = Kx(1)*ex_b + Kx(2)*I_x   - Kx(3)*xm(4);
    Y  = Ky(1)*ey_b + Ky(2)*I_y   - Ky(3)*xm(5);
    Z  = Kz(1)*ez   + Kz(2)*I_z   - Kz(3)*xm(6);
    Mz = Kpsi(1)*epsi + Kpsi(2)*I_psi - Kpsi(3)*xm(8);

    u_cmd = max(min([X; Y; Z; Mz], p.u_max), p.u_min);

    u_act = u_act + a_lag * (u_cmd - u_act);      % thruster lag
    x = rk4_integrate(x, u_act, d_true, p, p.Ts);

    U_log(:, k) = u_cmd;
    X_log(:, k+1) = x;
end

Xref = reference_trajectory(t_vec, p);
out.t = t_vec;
out.X = X_log;
out.U = U_log;
out.pos_error = sqrt(sum((X_log(1:2, :) - Xref(1:2, :)).^2, 1));
out.rmse_pos = sqrt(mean(out.pos_error.^2));
out.chatter = mean(abs(diff(U_log, 1, 2)), 'all');   % command jitter
out.effort  = mean(abs(U_log), 'all');
out.ok = all(isfinite(X_log(:)));
end

function o = fill_defaults(o, p)
d = struct('vel_noise', 0.01, 'pos_noise', 0.02, 'yaw_noise', 0.01, ...
           'tau', 0.3, 'seed', 0, 't_end', p.Tf);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}) || isempty(o.(f{i}))
        o.(f{i}) = d.(f{i});
    end
end
end
