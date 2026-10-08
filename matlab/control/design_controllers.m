function ctrl = design_controllers(P, eq, sys, opts)
%DESIGN_CONTROLLERS  Pressure (bag) controller and the two slide-mode controllers.
%   Structure (handover Section 15.2):
%   * Bag: state feedback on [dm dz dzd] plus integral of the pressure error,
%     observer from pressure alone, and a reference trajectory that squeezes the bag
%     at the onset of each breath gap and refills afterwards (2-DOF feedforward).
%   * Slides, in common (Sigma) and differential (Delta) coordinates, as a cascade:
%       inner: position servo, state feedback on the slide position and velocity
%              predicted by the motor model (pole placement);
%       outer: integral action on the microphone estimate of the tuning error, with
%              its gain set by loop shaping for a phase margin against the
%              moving-average delay of the estimator (Lecture 7);
%       plus feedforward of measured pressure to the slide reference.
%     The microphone cannot separate a slide offset from reed drift (both shift e by
%     a constant), so there is no observer of slide position from e: the outer loop
%     integrates the tuning error itself.
%   Poles are chosen in the s-plane and mapped to z = exp(s*Ts), and gains are placed
%   on ZOH-discretised models, because the controller runs digitally at Ts.
if nargin < 4, opts = struct(); end
opts = fill_defaults(opts);
Ts = opts.Ts;
ctrl.opts = opts;
ctrl.Ts = Ts;

%% Bag / pressure controller (scaled design, Section 8.7)
Ab = sys.A(1:3, 1:3);  Bb = sys.B(1:3, 1:2);  Cb = sys.C(1, 1:3);
Sx = diag([eq.qbar*P.gap.max, 0.04, 0.05]);  Su = diag([eq.qbar, 50]);  sy = 50;
dsc = c2d(ss(Sx\Ab*Sx, Sx\Bb*Su, Cb*Sx/sy, 0), Ts, 'zoh');
[Phi, Gam, Cs] = deal(dsc.A, dsc.B, dsc.C);
Aa = [Phi, zeros(3, 1); Ts*Cs, 1];          % xi[k+1] = xi[k] + Ts*dp[k]
Ba = [Gam; zeros(1, 2)];
switch opts.bag_method
    case 'place'
        Ka = place(Aa, Ba, exp(opts.bag_poles*Ts));
    case 'lqr'
        Ka = dlqr(Aa, Ba, opts.bag_Q, opts.bag_R);
    otherwise
        error('design_controllers: unknown bag_method "%s"', opts.bag_method);
end
bag.Kx = Su*Ka(:, 1:3)/Sx;                  % u_dev = -Kx*(xhat - x_ref) - Ki*xi
bag.Ki = Su*Ka(:, 4)/sy;                    % xi = integral of (p - pbar) [Pa s]
Ls = place(Phi', Cs', exp(opts.bag_obs_poles*Ts))';
PhiP = Sx*Phi/Sx;  GamP = Sx*Gam/Su;  CP = Cs*sy/Sx;  LP = Sx*Ls/sy;
bag.Ad = PhiP - LP*CP;                      % xhat+ = Ad*xhat + Bd*[u_dev; dp]
bag.Bd = [GamP, LP];
bag.Phi = PhiP;  bag.Gam = GamP;  bag.C = CP;  bag.L = LP;
bag.cl_poles = log(eig(Aa - Ba*Ka))/Ts;     % continuous-time equivalents
bag.obs_poles = log(eig(Phi - Ls*Cs))/Ts;
ctrl.bag = bag;

% Breath-gap reference trajectory (squeeze at the volume-outflow rate, then refill)
tr.v_gap   = eq.Qv_bar/P.A_c;                                    % [m/s]
tr.v_ret   = (opts.refill_fraction*P.qb_max - eq.qbar)/(eq.rho_bar*P.A_c);
tr.tau_ret = opts.tau_return;
tr.z_hi    = P.z_max - 0.005;
ctrl.traj = tr;

%% Slide controllers (common and differential modes)
ctrl.slideS = design_slide(P, eq, P.sens.Tw_sigma, opts);
ctrl.slideD = design_slide(P, eq, P.sens.Tw_delta, opts);
ctrl.beta_ff = [mean(eq.beta_e), P.beta_d1 - P.beta_d2];   % pressure feedforward gains
end

function K = design_slide(P, eq, Tw, opts)
Ts = opts.Ts;
M = slide_design_model(P, eq);
dm = c2d(ss(M.A, M.B, eye(2), 0), Ts, 'zoh');
K.Phi = dm.A;  K.Gam = dm.B;  K.M = M;  K.Tw = Tw;
% Inner position servo: V = -Kx*(xhat - [s_ref; 0])
K.Kx = place(K.Phi, K.Gam, exp(opts.servo_poles*Ts));
K.servo_poles = log(eig(K.Phi - K.Gam*K.Kx))/Ts;
% Outer integral gain kI [1/s]: s_ref += Ts*kI*e_m/h. Loop shaping for the target
% phase margin with the moving-average sensor: start from the pure-delay estimate
% wc = (90 - PM)/(Tw/2) and refine numerically on the actual loop.
pm_target = opts.tuning_pm_deg;
kI0 = (90 - pm_target)*pi/180/(Tw/2);
f = @(kI) tuning_loop_pm(K, kI, P.sens.f_mic, Ts) - pm_target;
K.kI = fzero(f, [0.2*kI0, 2*kI0]);
K.pm_deg = f(K.kI) + pm_target;
end

function pm = tuning_loop_pm(K, kI, f_mic, Ts)
% Phase margin of L = kI*Ts/(z-1) * T_servo(z) * MA(z) at its first crossover.
w = logspace(-3, 1, 2000);
Tservo = ss(K.Phi - K.Gam*K.Kx, K.Gam*K.Kx(1), [1 0], 0, Ts);   % s_ref -> s
z = exp(1i*w*Ts);
Lw = kI*Ts./(z - 1).*squeeze(freqresp(Tservo, w)).'.*moving_average_fr(K.Tw, f_mic, Ts, w);
pm = first_crossover_pm(w, Lw);
end

function o = fill_defaults(o)
d.Ts = 0.005;                                % controller / simulation step [s]
d.bag_method = 'place';
d.bag_poles = [-2, -5, -15 + 10i, -15 - 10i];       % s-plane [1/s]
d.bag_Q = diag([1, 1, 0.1, 10]);
d.bag_R = diag([1, 1]);
d.bag_obs_poles = [-50, -120 + 60i, -120 - 60i];    % fast observer: PM ~53 deg, GM ~17 dB
d.servo_poles = [-15, -200];                 % slide position servo [1/s]
d.tuning_pm_deg = 55;                        % phase margin of the microphone loop [deg]
d.refill_fraction = 0.8;                     % refill feedforward uses up to 80% of q_b,max
d.tau_return = 0.5;                          % [s]
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(o, f{k}), o.(f{k}) = d.(f{k}); end
end
end
