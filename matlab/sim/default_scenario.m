function scn = default_scenario(P, kind, seed, T_end)
%DEFAULT_SCENARIO  Closed-loop test scenarios (handover Section 14.2).
%   kind: 'performance' - random breath gaps, chanter reed drift ramp, one drone
%                         warming faster, chanter temperature step, bag temperature step
%         'worst'       - as 'performance' but with the worst-case gap sequence
%         'gaps_only'   - random breath gaps, no other disturbances
%         'quiet'       - no gaps and no disturbances (sanity check)
if nargin < 3, seed = 1; end
if nargin < 4, T_end = 600; end
scn.kind = kind;
scn.seed = seed;
scn.T_end = T_end;
scn.log_dt = 0.05;           % logging interval [s]
scn.metric_start = 10;       % ignore the first seconds in the metrics [s]
switch kind
    case 'performance'
        scn.gaps = breath_gaps(P, T_end, 'random', seed);
        scn.d_profile = @(t) perf_profile(t, P, T_end);
    case 'worst'
        scn.gaps = breath_gaps(P, T_end, 'worst');
        scn.d_profile = @(t) perf_profile(t, P, T_end);
    case 'gaps_only'
        scn.gaps = breath_gaps(P, T_end, 'random', seed);
        scn.d_profile = @(t) still_profile(t, P);
    case 'quiet'
        scn.gaps = zeros(0, 2);
        scn.d_profile = @(t) still_profile(t, P);
    otherwise
        error('default_scenario: unknown kind "%s"', kind);
end
end

function D = still_profile(t, P)
D = [repmat(P.Tbar, 4, numel(t)); zeros(1, numel(t))];
end

function D = perf_profile(t, P, T)
% Rates are fixed (PLACEHOLDER values - replace with the measured drift), event times
% scale with the run length, so short runs see the same physics as long ones.
rate_reed = 3.0/480;      % chanter reed drift [Hz/s]: +3 Hz in 8 min
rate_Td1  = 1.0/180;      % tenor 1 air column warming faster [K/s]: +1 K in 3 min
D = still_profile(t, P);
D(5, :) = rate_reed*max(t - 0.10*T, 0);                       % from 10% of the run
D(2, :) = D(2, :) + min(rate_Td1*max(t - 0.50*T, 0), 1.0);    % from 50%, up to +1 K
D(4, :) = D(4, :) + 0.1*(t >= 0.70*T);                        % chanter air step +0.1 K
D(1, :) = D(1, :) + 0.5*(t >= 0.83*T);                        % bag air step +0.5 K
end
