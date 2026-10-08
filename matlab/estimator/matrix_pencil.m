function [f, a, sv] = matrix_pencil(z, fs, M)
%MATRIX_PENCIL  Fit z(n) = sum_k a_k exp(j*2*pi*f_k*n/fs), k = 1..M (Hua & Sarkar, 1990).
%   Resolves tones closer than the FFT bin spacing 1/T when the SNR is high, which is
%   what the near-in-tune beats need. sv are the singular values of the Hankel matrix:
%   sv(M+1)/sv(1) near the noise floor means the model order M fits.
z = z(:);
N = numel(z);
Lp = round(N/3);                              % pencil parameter
Y = hankel(z(1:N - Lp), z(N - Lp:N));        % (N-Lp) x (Lp+1)
[~, S, V] = svd(Y, 'econ');
sv = diag(S);
% Rows of Y are combinations of [1 lam lam^2 ...], so its row space is spanned by
% conj(V) (Y = U*S*V'); the shift invariance holds there.
V = conj(V(:, 1:M));
lam = eig(pinv(V(1:end-1, :))*V(2:end, :));
f = angle(lam)*fs/(2*pi);
Z = lam.'.^((0:N - 1).');
a = Z\z;
[f, i] = sort(f);
a = a(i);
end
