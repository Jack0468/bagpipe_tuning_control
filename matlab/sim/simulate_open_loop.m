function out = simulate_open_loop(P, eq, u_fun, d_fun, T_end, sys)
%SIMULATE_OPEN_LOOP  Open-loop response of the nonlinear model (ode15s) and the LTI model.
%   u_fun(t) -> 4x1 absolute inputs [q_b F V_m1 V_m2]; d_fun(t) -> 5x1 absolute
%   disturbances [T_bag T_d1 T_d2 T_c delta_reed]. Both start from the equilibrium.
%   ode15s events stop the run if p reaches 0 (the p > 0 assumption fails) and record
%   when p leaves the sounding window [p_min, p_max] and when z leaves [0, z_max].
%   out.nl.{t,x,y}, out.lin.{t,y}, out.events, out.failed.
if nargin < 6, sys = lti_analytic(P, eq); end
x0 = eq.xbar;
atol = [1e-9, 1e-8, 1e-8, 1e-9, 1e-8, 1e-6, 1e-9, 1e-8, 1e-6];
opts = odeset('RelTol', 1e-7, 'AbsTol', atol, 'MaxStep', 0.05, ...
    'Events', @(t, x) ol_events(t, x, d_fun, P));
out.failed = false;
try
    [t, x, te, ~, ie] = ode15s(@(t, x) plant_rhs(x, u_fun(t), d_fun(t), P), [0, T_end], x0, opts);
catch err
    if ~strcmp(err.identifier, 'bagpipe:pressureNonPositive'), rethrow(err); end
    out.failed = true;  t = 0;  x = x0.';  te = [];  ie = [];
end
y = zeros(numel(t), 3);
for k = 1:numel(t)
    y(k, :) = plant_outputs(x(k, :).', d_fun(t(k)), P).';
end
out.nl = struct('t', t, 'x', x, 'y', y);
names = {'p reached 0', 'p crossed p_min', 'p crossed p_max', 'z hit z_max', 'z hit 0'};
out.events = struct('t', {}, 'what', {});
for k = 1:numel(te)
    out.events(k) = struct('t', te(k), 'what', names{ie(k)});
end
if any(ie == 1), out.failed = true; end

% LTI model on a uniform grid (deviation variables)
tl = linspace(0, T_end, 4000).';
du = zeros(numel(tl), 4);  dd = zeros(numel(tl), 5);
for k = 1:numel(tl)
    du(k, :) = (u_fun(tl(k)) - eq.ubar).';
    dd(k, :) = (d_fun(tl(k)) - eq.dbar).';
end
G = ss(sys.A, [sys.B, sys.E], sys.C, [sys.D, sys.Dd]);
yl = lsim(G, [du, dd], tl);
out.lin = struct('t', tl, 'y', yl + eq.ybar.');
end

function [value, isterminal, direction] = ol_events(t, x, d_fun, P)
d = d_fun(t);
p = bag_pressure(x(1), x(2), d(1), P);
value = [p; p - P.p_min; P.p_max - p; P.z_max - x(2); x(2)];
isterminal = [1; 0; 0; 1; 1];
direction = [-1; 0; 0; -1; -1];
end
