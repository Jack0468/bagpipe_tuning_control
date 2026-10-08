function P = bagpipe_params()
%BAGPIPE_PARAMS  Plant, actuator, sensor and target parameters (SI units, pitch in Hz).
%   Every value tagged PLACEHOLDER is an order-of-magnitude starting point from the
%   handover (Section 10) and must be replaced by the students' measurements
%   (handover Section 10.5). P.placeholders lists them so the run scripts can warn.
%   Derived values (flow coefficients, b_a, drone lengths, m_eff) are recomputed
%   here, so edit the measured inputs, not the derived ones.

%% Physical constants
P.R     = 287;          % specific gas constant, dry air [J/(kg K)]
P.p_atm = 101325;       % [Pa]
P.c0    = 331.3;        % speed of sound at 273.15 K [m/s]

%% Operating point
P.Tbar   = 303;         % nominal air temperature [K]                    PLACEHOLDER
P.pbar   = 5000;        % playing pressure, gauge [Pa]                   PLACEHOLDER
P.p_min  = 3500;        % reeds stop below this [Pa]                     PLACEHOLDER
P.p_max  = 6500;        % reeds choke above this [Pa]                    PLACEHOLDER
P.fc0    = 480;         % chanter Low A at the operating point [Hz]      PLACEHOLDER
P.fd_bar = P.fc0/2;     % tenor drone frequency when in tune [Hz]

%% Reed pressure sensitivities: local slopes at pbar [Hz/Pa]
P.beta_c  = 0.004;      % chanter                                        PLACEHOLDER
P.beta_d1 = 0.003;      % tenor 1                                        PLACEHOLDER
P.beta_d2 = 0.003;      % tenor 2                                        PLACEHOLDER

%% Reed flow: total air use at pbar, split between the sounding reeds (bass stopped)
P.Qv_total   = 0.4e-3;            % volume outflow at bag conditions [m^3/s]  PLACEHOLDER
P.flow_share = [0.5 0.25 0.25];   % chanter, tenor 1, tenor 2              PLACEHOLDER
rho_bar = (P.pbar + P.p_atm)/(P.R*P.Tbar);
K_total = rho_bar*P.Qv_total/sqrt(P.pbar);
P.K_c  = P.flow_share(1)*K_total;   % [kg s^-1 Pa^-1/2]
P.K_d1 = P.flow_share(2)*K_total;
P.K_d2 = P.flow_share(3)*K_total;
P.K_d  = P.K_d1 + P.K_d2;           % all sounding drone reeds

%% Bag and squeezing arm
P.V0       = 0.010;     % bag volume at z = 0 [m^3]                      PLACEHOLDER
P.A_c      = 0.02;      % arm-bag contact area [m^2]                     PLACEHOLDER
P.zbar     = 0.02;      % nominal squeeze displacement [m]               PLACEHOLDER
P.z_max    = 0.10;      % squeeze travel limit [m]                       PLACEHOLDER
P.M        = 1.0;       % arm + bag wall moving mass [kg]                PLACEHOLDER
P.k        = 1000;      % bag/arm stiffness, measured with bag vented [N/m] PLACEHOLDER
P.zeta_arm = 0.5;       % damping ratio used to set b_a                  PLACEHOLDER
Vbar = P.V0 - P.A_c*P.zbar;
a_z  = (P.pbar + P.p_atm)*P.A_c/Vbar;
P.b_a   = 2*P.zeta_arm*sqrt((P.k + P.A_c*a_z)*P.M);   % [N s/m]
P.F_max = 300;          % max squeeze force [N]                          PLACEHOLDER

%% Blowpipe and random breath gaps (fit these to the player recordings)
qbar = (P.K_d + P.K_c)*sqrt(P.pbar);
P.qb_max = 3*qbar;      % max blowpipe flow [kg/s]                       PLACEHOLDER
P.gap  = struct('mean', 1.2, 'cv', 0.30, 'min', 0.6, 'max', 2.0);   % [s] PLACEHOLDER
P.blow = struct('mean', 4.0, 'cv', 0.35, 'min', 2.0, 'max', 8.0);   % [s] PLACEHOLDER

%% Tenor drones (acoustics)
P.bore_radius = 0.006;            % tenor bore radius [m]               PLACEHOLDER
P.L_e  = 0.6*P.bore_radius;       % end correction [m]
L_tot  = P.c0*sqrt(P.Tbar/273.15)/(4*P.fd_bar);
P.L_d1 = L_tot - P.L_e;           % chosen so the slide sits mid-travel (sbar = 0)
P.L_d2 = L_tot - P.L_e;
P.s_min = -0.02;  P.s_max = 0.02; % slide travel [m]                    PLACEHOLDER

%% Slide drives: one DC motor + leadscrew per tenor (take values from a datasheet)
mot.R_a   = 5;        % armature resistance [Ohm]                        PLACEHOLDER
mot.L     = 0.5e-3;   % armature inductance [H]                          PLACEHOLDER
mot.K_t   = 0.01;     % torque constant [N m/A]                          PLACEHOLDER
mot.K_e   = mot.K_t;  % back-EMF constant [V s/rad] (= K_t in SI)
mot.J_m   = 1e-7;     % rotor inertia [kg m^2]                           PLACEHOLDER
mot.N     = 1;        % gearbox ratio (1 = direct drive)
mot.lead  = 1e-3;     % leadscrew lead [m/rev]                           PLACEHOLDER
mot.m_s   = 0.05;     % bare slide + drone section mass [kg]             PLACEHOLDER
mot.b_s   = 200;      % total viscous friction at the slide [N s/m]      PLACEHOLDER
mot.V_max = 12;       % supply voltage [V]
mot.G     = 2*pi*mot.N/mot.lead;          % [rad/m]
mot.m_eff = mot.m_s + mot.J_m*mot.G^2;    % includes reflected rotor inertia
P.slide = [mot mot];  % per-drone copies; edit one to model hardware mismatch
P.V_max = mot.V_max;

%% Sensors
P.sens.sigma_p  = 2;      % pressure sensor noise [Pa RMS]               PLACEHOLDER
P.sens.Tw_sigma = 5;      % microphone estimator window, e_Sigma [s]     PLACEHOLDER
P.sens.Tw_delta = 5;      % microphone estimator window, e_Delta [s]     PLACEHOLDER
P.sens.f_mic    = 10;     % microphone estimate update rate [Hz]
P.sens.sigma_e  = 0.02;   % estimator noise [Hz RMS]: run_estimator_demo gives ~0.02 Hz
                          % RMS for e_Sigma at Tw = 5 s on synthetic audio  PLACEHOLDER
P.sens.floor_n  = 0;      % 0: off; n > 0 reports 0 when |e| < 1/(2 n Tw) (Step 4)

%% Target
P.e_target = 0.05;        % |e_j| limit [Hz]: at most one beat every 10 s

P.placeholders = {'Tbar', 'pbar', 'p_min', 'p_max', 'fc0', 'beta_c', 'beta_d1', ...
    'beta_d2', 'Qv_total', 'flow_share', 'V0', 'A_c', 'zbar', 'z_max', 'M', 'k', ...
    'zeta_arm', 'F_max', 'qb_max', 'gap', 'blow', 'bore_radius', 's_min/s_max', ...
    'slide motor + leadscrew (R_a, L, K_t, J_m, lead, m_s, b_s)', 'sens.sigma_p', ...
    'sens.Tw_sigma', 'sens.Tw_delta', 'sens.sigma_e'};
end
