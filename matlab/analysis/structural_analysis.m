function R = structural_analysis(sys, P, eq, verbose)
%STRUCTURAL_ANALYSIS  Reachability, observability, poles and channel zeros (Section 9).
%   Rank tests use the scaled model (Section 8.7); the bag-block formulas
%   det(O_bag) = a_m a_z^2 k / M and det(A_bag) = -g a_m k / M are cross-checked.
if nargin < 4, verbose = true; end
[sc, ~] = scale_model(sys, P, eq);

% The textbook test rank[B AB ... A^8 B] fails numerically here: the poles span
% ~0.08 to ~9800 1/s, so A^8 B is dominated by the motor-current pole (~1e32) and the
% computed rank collapses. Use (a) the PBH test, equivalent and well conditioned, and
% (b) the W-matrix rank of each decoupled block (bag, slide 1, slide 2).
[R.rank_ctrb, R.rank_obsv] = pbh_ranks(sc.A, sc.B, sc.C);
R.rank_ctrb_W_full = rank(ctrb(sc.A, sc.B));      % reported only to show the issue
blocks = {1:3, 4:6, 7:9};
R.rank_ctrb_W_blocks = cellfun(@(b) rank(ctrb(sc.A(b, b), sc.B(b, :))), blocks);
R.rank_obsv_W_blocks = cellfun(@(b) rank(obsv(sc.A(b, b), sc.C(:, b))), blocks);
sv = svd(obsv(sc.A, sc.C));
R.cond_obsv = sv(1)/sv(end);

% Bag block: observability from pressure alone, stability, per-input reachability
Ab = sys.A(1:3, 1:3);  Cb = sys.C(1, 1:3);
R.det_obsv_bag         = det(obsv(Ab, Cb));
R.det_obsv_bag_formula = eq.a_m*eq.a_z^2*P.k/P.M;
R.det_Abag             = det(Ab);
R.det_Abag_formula     = -eq.g*eq.a_m*P.k/P.M;
svb = svd(obsv(sc.A(1:3, 1:3), sc.C(1, 1:3)));
R.cond_obsv_bag = svb(1)/svb(end);
R.air_spring = P.A_c*eq.a_z;
R.rank_bag_from_qb = rank(ctrb(sc.A(1:3, 1:3), sc.B(1:3, 1)));
R.rank_bag_from_F  = rank(ctrb(sc.A(1:3, 1:3), sc.B(1:3, 2)));

R.poles = eig(sys.A);

% Channel transfer functions and zeros (scaled model is better conditioned)
Gs = ss(sc.A, sc.B, sc.C, sc.D);
R.zeros_p_qb = zero(minreal(Gs(1, 1), 1e-9, false));
R.zeros_p_F  = zero(minreal(Gs(1, 2), 1e-9, false));
R.zeros_e1_V1 = zero(minreal(Gs(2, 3), 1e-9, false));
G = ss(sys.A, sys.B, sys.C, sys.D);
R.dcgain_p_qb = dcgain(G(1, 1));
R.G = G;

if verbose
    fprintf('\n=== Structural properties (Section 9) ===\n');
    fprintf('PBH test: reachable rank %d of 9, observable rank %d of 9 (scaled model)\n', ...
        R.rank_ctrb, R.rank_obsv);
    fprintf('W-matrix rank per block [bag slide1 slide2]: reachability %s, observability %s\n', ...
        mat2str(R.rank_ctrb_W_blocks), mat2str(R.rank_obsv_W_blocks));
    fprintf('(full 9x36 W matrix gives numerical rank %d: the 1e5 pole spread swamps A^8 B)\n', ...
        R.rank_ctrb_W_full);
    fprintf('bag block from p: det(O) = %.4g (formula a_m a_z^2 k/M = %.4g), cond (scaled) = %.3g\n', ...
        R.det_obsv_bag, R.det_obsv_bag_formula, R.cond_obsv_bag);
    fprintf('  k = %.0f N/m vs air-spring stiffness A_c a_z = %.0f N/m\n', P.k, R.air_spring);
    fprintf('det(A_bag) = %.4g (formula -g a_m k/M = %.4g)\n', R.det_Abag, R.det_Abag_formula);
    fprintf('bag block reachable from q_b alone: rank %d of 3; from F alone: rank %d of 3\n', ...
        R.rank_bag_from_qb, R.rank_bag_from_F);
    fprintf('open-loop poles [1/s]:\n');  fprintf('  %s\n', num2str(R.poles.', '%10.4g'));
    fprintf('zeros p/q_b: %s\n', num2str(R.zeros_p_qb.', '%.4g '));
    fprintf('zeros p/F  : %s   (expect one at s = 0)\n', num2str(R.zeros_p_F.', '%.4g '));
    fprintf('DC gain p/q_b = %.4g Pa per kg/s (1/g = %.4g)\n', R.dcgain_p_qb, 1/eq.g);
end
end

function [rc, ro] = pbh_ranks(A, B, C)
% PBH: (A,B) reachable iff rank[lambda*I - A, B] = n for every eigenvalue lambda;
% (A,C) observable iff rank[lambda*I - A; C] = n. Returns the smallest such rank.
n = size(A, 1);
lam = eig(A);
rc = n;  ro = n;
for k = 1:numel(lam)
    Mc = [lam(k)*eye(n) - A, B];
    Mo = [lam(k)*eye(n) - A; C];
    rc = min(rc, rank(Mc, 1e-9*norm(Mc)));
    ro = min(ro, rank(Mo, 1e-9*norm(Mo)));
end
end
