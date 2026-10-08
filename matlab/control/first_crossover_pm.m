function [pm, wc, gm_dB] = first_crossover_pm(w, Lw)
%FIRST_CROSSOVER_PM  Phase margin at the first 0 dB crossover and the smallest gain margin.
%   w increasing [rad/s], Lw complex loop response (negative-feedback convention).
mag = abs(Lw);
ph = unwrap(angle(Lw))*180/pi;
i = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
if isempty(i)
    pm = Inf;  wc = NaN;
else
    r = log(mag(i))/(log(mag(i)) - log(mag(i+1)));     % interpolate in log magnitude
    wc = exp(log(w(i)) + r*(log(w(i+1)) - log(w(i))));
    phc = ph(i) + r*(ph(i+1) - ph(i));
    pm = mod(phc + 180 + 180, 360) - 180;              % wrap to (-180, 180]
end
% Gain margin: where the phase crosses -180 (mod 360), smallest 1/|L| above 1
k = find(diff(floor((ph + 180)/360)) ~= 0);
gm = 1./mag(k + 1);
gm = gm(gm > 1);
gm_dB = 20*log10(min([gm, Inf]));
end
