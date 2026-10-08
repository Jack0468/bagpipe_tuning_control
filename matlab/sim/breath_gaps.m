function gaps = breath_gaps(P, T_end, mode, seed)
%BREATH_GAPS  Breath-gap intervals [t_start t_end] (s) on [0, T_end].
%   'random': gap durations and blowing intervals drawn from truncated lognormal
%             distributions (P.gap, P.blow). Replace these with distributions fitted
%             to the player recordings (handover Section 10.5 item 7).
%   'worst' : longest gap followed by the shortest blowing interval, repeated.
%   'none'  : no gaps.
if nargin < 4, seed = 1; end
gaps = zeros(0, 2);
switch mode
    case 'none'
        return
    case 'random'
        s = RandStream('mt19937ar', 'Seed', seed);
        t = draw(P.blow, s);
        while t < T_end
            g = draw(P.gap, s);
            gaps(end+1, :) = [t, min(t + g, T_end)]; %#ok<AGROW>
            t = t + g + draw(P.blow, s);
        end
    case 'worst'
        t = P.blow.min;
        while t < T_end
            gaps(end+1, :) = [t, min(t + P.gap.max, T_end)]; %#ok<AGROW>
            t = t + P.gap.max + P.blow.min;
        end
    otherwise
        error('breath_gaps: unknown mode "%s"', mode);
end
end

function v = draw(D, s)
% Truncated lognormal with the given mean and coefficient of variation (rejection sampling).
sig2 = log(1 + D.cv^2);
mu = log(D.mean) - sig2/2;
v = Inf;
while v < D.min || v > D.max
    v = exp(mu + sqrt(sig2)*randn(s));
end
end
