function [g_best, info] = tune_pid_noisy(p, train_seeds, w_chatter)
%TUNE_PID_NOISY Fix 1b: tune the gains under sensor noise and thruster lag.
%   Cost = mean position RMSE over the training seeds
%          + w_chatter * mean command jitter.
%   The jitter term is what stops the optimiser from using huge gains
%   that only work because the simulation was noise-free.
%
%   Seeds used here must differ from the seeds used for testing.

if nargin < 2 || isempty(train_seeds), train_seeds = [1 2 3]; end
if nargin < 3 || isempty(w_chatter),   w_chatter   = 0.02;    end

g0  = baseline_gains();
map = @(th) g0 .* exp(2*tanh(th));
obj = @(th) objective(map(th), p, train_seeds, w_chatter);

opts = optimset('Display', 'off', 'MaxFunEvals', 1500, 'MaxIter', 700, ...
                'TolX', 1e-3, 'TolFun', 1e-5, 'OutputFcn', @progress);
[th_best, f_best, ~, output] = fminsearch(obj, zeros(1, 12), opts);

g_best = map(th_best);
info.cost = f_best;
info.iterations = output.iterations;
info.evals = output.funcCount;
info.train_seeds = train_seeds;
info.w_chatter = w_chatter;
end

function f = objective(g, p, seeds, w)
r = zeros(size(seeds)); c = zeros(size(seeds));
for i = 1:numel(seeds)
    o = simulate_pid_noisy(p, g, struct('seed', seeds(i)));
    if ~o.ok
        f = 1e3; return;
    end
    r(i) = o.rmse_pos; c(i) = o.chatter;
end
f = mean(r) + w * mean(c);
end

function stop = progress(~, v, state)
stop = false;
if strcmp(state, 'iter') && mod(v.iteration, 25) == 0
    fprintf('  iter %4d   cost %.5f\n', v.iteration, v.fval);
end
end
