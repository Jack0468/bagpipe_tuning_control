function [z, fs_b] = band_baseband(y, fs, f0, fs_b_target)
%BAND_BASEBAND  Complex demodulation of the band around f0, low-pass and decimation.
%   z(t) contains the components of y near f0, shifted to f0 -> 0 Hz, sampled at fs_b.
%   Two tones beating in this band appear as two complex exponentials in z.
t = (0:numel(y) - 1).'/fs;
x = y(:).*exp(-1i*2*pi*f0*t);
R = round(fs/fs_b_target);
zr = real(x);  zi = imag(x);
for r = decimation_factors(R)                 % decimate() low-pass filters each stage
    zr = decimate(zr, r);
    zi = decimate(zi, r);
end
z = zr + 1i*zi;
fs_b = fs/R;
end

function f = decimation_factors(R)
% Split R into factors of at most 10 (decimate() recommends <= 13 per stage).
f = [];
while R > 1
    d = 10;
    while mod(R, d) ~= 0, d = d - 1; end
    if d == 1, error('band_baseband: cannot factor the decimation ratio %d', R); end
    f(end+1) = d; %#ok<AGROW>
    R = R/d;
end
end
