function g = baseline_gains()
%BASELINE_GAINS The hand-tuned gains used by simulate_pid_baseline.
%   Order: [Kp Ki Kd] for x, y, z, psi.
g = [8.0 0.15 4.0, ...    % x   (surge)
     8.0 0.15 4.0, ...    % y   (sway)
     10.0 0.20 5.0, ...   % z   (heave)
     6.0 0.10 2.5];       % psi (yaw)
end
