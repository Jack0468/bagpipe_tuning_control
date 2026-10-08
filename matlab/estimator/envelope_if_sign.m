function [sgn, r] = envelope_if_sign(z, fs_b)
%ENVELOPE_IF_SIGN  Passive sign of a two-tone beat (handover Section 13.2, Step 2).
%   For z = A*exp(j wa t) + B*exp(j wb t) with A > B, the instantaneous frequency swings
%   toward the weaker tone when the envelope is high and away from it when it is low.
%   So corr(envelope, instantaneous frequency) has the sign of (f_weaker - f_stronger).
%   If the chanter partial is the stronger one, sgn is the sign of the tuning error.
z = z(:);
env = abs(z(2:end));
ifr = diff(unwrap(angle(z)))*fs_b/(2*pi);
c = corrcoef(env, ifr);
r = c(1, 2);
sgn = sign(r);
end
