function sys = lti_analytic(P, eq)
%LTI_ANALYTIC  Hand-derived LTI matrices, handover Sections 8.3-8.5.
%   dx = A dx + B du + E dd,  dy = C dx + D du + Dd dd
%   with dd = [dT_bag dT_d1 dT_d2 dT_c d_reed]'.
A = zeros(9);
B = zeros(9, 4);
A(1:3, 1:3) = [-eq.g*eq.a_m,        -eq.g*eq.a_z,                  0;
               0,                    0,                             1;
               -P.A_c*eq.a_m/P.M,   -(P.k + P.A_c*eq.a_z)/P.M,     -P.b_a/P.M];
B(1, 1) = 1;
B(3, 2) = 1/P.M;
for j = 1:2
    sp = P.slide(j);
    r = 3*j + (1:3);
    A(r, r) = [0,  1,                       0;
               0, -sp.b_s/sp.m_eff,         sp.G*sp.K_t/sp.m_eff;
               0, -sp.K_e*sp.G/sp.L,       -sp.R_a/sp.L];
    B(r(3), 2+j) = 1/sp.L;
end

C = zeros(3, 9);
C(1, 1:2) = [eq.a_m, eq.a_z];
C(2, 1:2) = eq.beta_e(1)*[eq.a_m, eq.a_z];   C(2, 4) = -eq.h_s(1);
C(3, 1:2) = eq.beta_e(2)*[eq.a_m, eq.a_z];   C(3, 7) = -eq.h_s(2);
D = zeros(3, 4);

E = zeros(9, 5);
E(1, 1) = -eq.g*eq.a_T;
E(3, 1) = -P.A_c*eq.a_T/P.M;
kT = eq.fd_bar/(2*P.Tbar);                    % pitch sensitivity to air-column temperature
Dd = [eq.a_T,              0,  0, 0,    0;
      eq.beta_e(1)*eq.a_T, kT, 0, -kT, -0.5;
      eq.beta_e(2)*eq.a_T, 0, kT, -kT, -0.5];

sys = struct('A', A, 'B', B, 'C', C, 'D', D, 'E', E, 'Dd', Dd);
end
