function sys = lti_numeric(P, eq)
%LTI_NUMERIC  Central-difference Jacobians of the nonlinear model at the equilibrium.
%   Independent cross-check of lti_analytic (handover Section 8.8). The Symbolic
%   Math Toolbox is not installed here, so finite differences replace jacobian().
x0 = eq.xbar;  u0 = eq.ubar;  d0 = eq.dbar;
hx = [1e-7, 1e-6, 1e-6, 1e-7, 1e-6, 1e-6, 1e-7, 1e-6, 1e-6];  % step per state
hu = [1e-9, 1e-3, 1e-4, 1e-4];
hd = [1e-3, 1e-3, 1e-3, 1e-3, 1e-4];
f = @(x, u, d) plant_rhs(x, u, d, P);
h = @(x, d) plant_outputs(x, d, P);

A = zeros(9);  C = zeros(3, 9);
for k = 1:9
    dx = zeros(9, 1);  dx(k) = hx(k);
    A(:, k) = (f(x0 + dx, u0, d0) - f(x0 - dx, u0, d0))/(2*hx(k));
    C(:, k) = (h(x0 + dx, d0) - h(x0 - dx, d0))/(2*hx(k));
end
B = zeros(9, 4);
for k = 1:4
    du = zeros(4, 1);  du(k) = hu(k);
    B(:, k) = (f(x0, u0 + du, d0) - f(x0, u0 - du, d0))/(2*hu(k));
end
E = zeros(9, 5);  Dd = zeros(3, 5);
for k = 1:5
    dd = zeros(5, 1);  dd(k) = hd(k);
    E(:, k)  = (f(x0, u0, d0 + dd) - f(x0, u0, d0 - dd))/(2*hd(k));
    Dd(:, k) = (h(x0, d0 + dd) - h(x0, d0 - dd))/(2*hd(k));
end
sys = struct('A', A, 'B', B, 'C', C, 'D', zeros(3, 4), 'E', E, 'Dd', Dd);
end
