function eq = compute_equilibrium(P)
%COMPUTE_EQUILIBRIUM  Operating point and linearisation constants, handover Sections 7-8.
T = P.Tbar;
eq.Vbar = P.V0 - P.A_c*P.zbar;
eq.mbar = (P.pbar + P.p_atm)*eq.Vbar/(P.R*T);
eq.qbar = (P.K_d + P.K_c)*sqrt(P.pbar);      % long-run mean blowpipe flow
eq.Fbar = P.k*P.zbar + P.A_c*P.pbar;
c = sound_speed(T, P);
eq.sbar = [c/(2*P.fc0) - P.L_d1 - P.L_e, c/(2*P.fc0) - P.L_d2 - P.L_e];

eq.xbar = [eq.mbar; P.zbar; 0; eq.sbar(1); 0; 0; eq.sbar(2); 0; 0];
eq.ubar = [eq.qbar; eq.Fbar; 0; 0];
eq.dbar = [T; T; T; T; 0];
eq.ybar = [P.pbar; 0; 0];

% Linearisation constants (Sections 8.1-8.4)
eq.a_m = P.R*T/eq.Vbar;                       % dp/dm
eq.a_z = (P.pbar + P.p_atm)*P.A_c/eq.Vbar;    % dp/dz
eq.a_T = (P.pbar + P.p_atm)/T;                % dp/dT_bag
eq.g   = (P.K_d + P.K_c)/(2*sqrt(P.pbar));    % dQ_out/dp
eq.beta_e = [P.beta_d1, P.beta_d2] - P.beta_c/2;
L_tot  = [P.L_d1 + eq.sbar(1), P.L_d2 + eq.sbar(2)] + P.L_e;
eq.h_s = c./(4*L_tot.^2);                     % drone pitch sensitivity to slide [Hz/m]
eq.fd_bar  = P.fc0/2;
eq.rho_bar = (P.pbar + P.p_atm)/(P.R*T);
eq.Qv_bar  = eq.qbar/eq.rho_bar;              % volume outflow at bag conditions [m^3/s]
end
