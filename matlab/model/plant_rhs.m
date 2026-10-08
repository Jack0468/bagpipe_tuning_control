function xdot = plant_rhs(x, u, d, P)
%PLANT_RHS  Nonlinear 9-state plant, handover Section 6.8.
%   x = [m z zd s1 sd1 i1 s2 sd2 i2]'
%   u = [q_b F V_m1 V_m2]'
%   d = [T_bag T_d1 T_d2 T_c delta_reed]'  (absolute temperatures [K], drift [Hz])
%   Errors if the bag pressure is not positive: the model assumes p > 0 for all
%   time (Section 4) and must not silently clamp it.
p = bag_pressure(x(1), x(2), d(1), P);
if p <= 0
    error('bagpipe:pressureNonPositive', ...
        'Bag pressure %.1f Pa <= 0: the p > 0 assumption is violated.', p);
end
xdot = zeros(9, 1);
xdot(1) = u(1) - (P.K_d + P.K_c)*sqrt(p);
xdot(2) = x(3);
xdot(3) = (u(2) - P.k*x(2) - P.b_a*x(3) - P.A_c*p)/P.M;
for j = 1:2
    sp = P.slide(j);
    r = 3*j;                       % slide j occupies states r+1..r+3
    xdot(r+1) = x(r+2);
    xdot(r+2) = (sp.G*sp.K_t*x(r+3) - sp.b_s*x(r+2))/sp.m_eff;
    xdot(r+3) = (u(2+j) - sp.R_a*x(r+3) - sp.K_e*sp.G*x(r+2))/sp.L;
end
end
