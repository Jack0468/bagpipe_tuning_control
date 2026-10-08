function out = simulate_closed_loop(P, eq, ctrl, scn, P_true)
%SIMULATE_CLOSED_LOOP  Fixed-step closed-loop simulation of the nonlinear plant.
%   P, eq, ctrl : nominal parameters, equilibrium and controllers (design_controllers)
%   scn         : scenario (default_scenario)
%   P_true      : parameters of the simulated plant and sensors (default P); use a
%                 perturbed copy for robustness tests.
%   The bag (nonlinear) is integrated with RK4 substeps; the slides (linear) use their
%   exact ZOH discretisation, including the motor current. The run stops with
%   out.failed = true if the bag pressure reaches zero (p > 0 assumption violated).
if nargin < 5, P_true = P; end
Ts = ctrl.Ts;
N = round(scn.T_end/Ts);
n_sub = 2;                                   % RK4 substeps for the bag per step
eqT = compute_equilibrium(P_true);
x = eqT.xbar;                                % plant starts at its own operating point

% Exact discretisation of each slide (truth parameters)
Phi = cell(1, 2);  Gam = cell(1, 2);
for j = 1:2
    sp = P_true.slide(j);
    Aj = [0, 1, 0; 0, -sp.b_s/sp.m_eff, sp.G*sp.K_t/sp.m_eff; 0, -sp.K_e*sp.G/sp.L, -sp.R_a/sp.L];
    dj = c2d(ss(Aj, [0; 0; 1/sp.L], eye(3), 0), Ts, 'zoh');
    Phi{j} = dj.A;  Gam{j} = dj.B;
end

Dp = scn.d_profile((0:N)*Ts);                % disturbances on the time grid
gaps = scn.gaps;  gi = 1;  ngap = size(gaps, 1);
rs = RandStream('mt19937ar', 'Seed', scn.seed + 1000);
S = mic_sensor_init(P_true.sens, Ts);

% Controller states and shorthand
bag = ctrl.bag;  tr = ctrl.traj;  KS = ctrl.slideS;  KD = ctrl.slideD;
h = KS.M.h;
xb = zeros(3, 1);  xi_p = 0;  z_ref = P.zbar;
xs = zeros(2, 1);  xd = zeros(2, 1);         % model-predicted [s; sd] of each mode
sIS = 0;  sID = 0;                            % integral (microphone) part of s_ref [m]
c_gas = (P.pbar + P.p_atm)/(P.R*P.Tbar);     % kg/m^3 at the operating point

% Logging
n_log = max(1, round(scn.log_dt/Ts));
NL = floor((N - 1)/n_log) + 1;
names = {'t', 'p', 'e1', 'e2', 'eS', 'eD', 'eSm', 'eDm', 'qb', 'F', 'V1', 'V2', ...
    'z', 's1', 's2', 'gap', 'z_ref'};
for k = 1:numel(names), lg.(names{k}) = nan(NL, 1); end
il = 0;
hits = struct('z', 0, 's', 0);
out.failed = false;  out.fail_msg = '';

try
    for k = 0:N-1
        t = k*Ts;
        d = Dp(:, k+1);
        while gi <= ngap && t >= gaps(gi, 2), gi = gi + 1; end
        in_gap = gi <= ngap && t >= gaps(gi, 1);

        % ---- measurements
        y = plant_outputs(x, d, P_true);
        p_meas = y(1) + P_true.sens.sigma_p*randn(rs);
        eS = mean(y(2:3));  eD = y(2) - y(3);
        [S, eSm, eDm] = mic_sensor_step(S, eS, eD, rs);
        dp = p_meas - P.pbar;

        % ---- bag controller: reference trajectory + state feedback + integral
        if in_gap
            zr_dot = tr.v_gap*(z_ref < tr.z_hi);
        elseif z_ref > P.zbar
            zr_dot = -min((z_ref - P.zbar)/tr.tau_ret, tr.v_ret);
        else
            zr_dot = 0;
        end
        mr_dot = -c_gas*P.A_c*zr_dot;
        x_ref = [c_gas*(P.V0 - P.A_c*z_ref) - eq.mbar; z_ref - P.zbar; zr_dot];
        u_ff  = [mr_dot; P.k*(z_ref - P.zbar) + P.b_a*zr_dot];
        u_dev = u_ff - bag.Kx*(xb - x_ref) - bag.Ki*xi_p;
        q_cmd = eq.qbar + u_dev(1);
        F_cmd = eq.Fbar + u_dev(2);
        qb = min(max(q_cmd, 0), P_true.qb_max*(~in_gap));
        F  = min(max(F_cmd, 0), P_true.F_max);
        if ~in_gap && q_cmd > 0 && q_cmd < P_true.qb_max   % conditional integration
            xi_p = xi_p + Ts*dp;
        end
        xb = bag.Ad*xb + bag.Bd*[qb - eq.qbar; F - eq.Fbar; dp];
        z_ref = z_ref + Ts*zr_dot;

        % ---- slide controllers (common / differential): position servo on the
        %      model-predicted slide state, reference = pressure feedforward +
        %      integral of the microphone tuning error
        VS = -KS.Kx*(xs - [ctrl.beta_ff(1)*dp/h + sIS; 0]);
        VD = -KD.Kx*(xd - [ctrl.beta_ff(2)*dp/h + sID; 0]);
        V = [VS + VD/2; VS - VD/2];
        Vs = min(max(V, -P_true.V_max), P_true.V_max);
        if all(Vs == V)                                   % anti-windup
            sIS = sIS + Ts*KS.kI*eSm/h;
            sID = sID + Ts*KD.kI*eDm/h;
        end
        xs = KS.Phi*xs + KS.Gam*mean(Vs);
        xd = KD.Phi*xd + KD.Gam*(Vs(1) - Vs(2));

        % ---- log (values at the start of this step)
        if mod(k, n_log) == 0
            il = il + 1;
            v = {t, y(1), y(2), y(3), eS, eD, eSm, eDm, qb, F, Vs(1), Vs(2), ...
                x(2), x(4), x(7), in_gap, z_ref - Ts*zr_dot};
            for q = 1:numel(names), lg.(names{q})(il) = v{q}; end
        end

        % ---- plant step
        xbag = x(1:3);
        hs = Ts/n_sub;
        for q = 1:n_sub
            k1 = bag_rhs(xbag, qb, F, d(1), P_true);
            k2 = bag_rhs(xbag + hs/2*k1, qb, F, d(1), P_true);
            k3 = bag_rhs(xbag + hs/2*k2, qb, F, d(1), P_true);
            k4 = bag_rhs(xbag + hs*k3, qb, F, d(1), P_true);
            xbag = xbag + hs/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        if xbag(2) > P_true.z_max || xbag(2) < 0          % squeeze travel stops
            xbag(2) = min(max(xbag(2), 0), P_true.z_max);
            xbag(3) = 0;  hits.z = hits.z + 1;
        end
        x(1:3) = xbag;
        x(4:6) = Phi{1}*x(4:6) + Gam{1}*Vs(1);
        x(7:9) = Phi{2}*x(7:9) + Gam{2}*Vs(2);
        for r = [4, 7]                                    % slide travel stops
            if x(r) > P_true.s_max || x(r) < P_true.s_min
                x(r) = min(max(x(r), P_true.s_min), P_true.s_max);
                x(r+1) = 0;  hits.s = hits.s + 1;
            end
        end
    end
catch err
    if strcmp(err.identifier, 'bagpipe:pressureNonPositive')
        out.failed = true;  out.fail_msg = err.message;
    else
        rethrow(err);
    end
end

for q = 1:numel(names), lg.(names{q}) = lg.(names{q})(1:il); end
out.log = lg;
out.hits = hits;
out.gaps = gaps;
out.metrics = closed_loop_metrics(lg, P_true, scn);
end

function f = bag_rhs(xb, qb, F, T_bag, P)
p = bag_pressure(xb(1), xb(2), T_bag, P);
if p <= 0
    error('bagpipe:pressureNonPositive', ...
        'Bag pressure %.1f Pa <= 0: the p > 0 assumption is violated.', p);
end
f = [qb - (P.K_d + P.K_c)*sqrt(p);
     xb(3);
     (F - P.k*xb(2) - P.b_a*xb(3) - P.A_c*p)/P.M];
end
