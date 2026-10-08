function S = mic_sensor_init(sens, Ts)
%MIC_SENSOR_INIT  State of the idealised microphone-estimator model (handover Section 13.3).
%   The estimator reports moving averages of e_Sigma and e_Delta over windows
%   Tw_sigma and Tw_delta, plus noise, updated at f_mic. It starts in tune.
S.sens = sens;
S.n_hold = max(1, round(1/(sens.f_mic*Ts)));   % simulation steps between updates
S.bufS = zeros(max(1, round(sens.Tw_sigma*sens.f_mic)), 1);
S.bufD = zeros(max(1, round(sens.Tw_delta*sens.f_mic)), 1);
S.iS = 1;  S.iD = 1;
S.count = S.n_hold - 1;     % first update on the first step
S.out = [0; 0];
end
