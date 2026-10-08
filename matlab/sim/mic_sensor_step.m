function [S, eS_m, eD_m] = mic_sensor_step(S, eS, eD, rs)
%MIC_SENSOR_STEP  Advance the microphone-estimator model one simulation step.
%   eS, eD: true common and differential tuning errors [Hz]; rs: RandStream for noise.
%   Between updates the last estimate is held.
S.count = S.count + 1;
if S.count >= S.n_hold
    S.count = 0;
    S.bufS(S.iS) = eS;  S.iS = mod(S.iS, numel(S.bufS)) + 1;
    S.bufD(S.iD) = eD;  S.iD = mod(S.iD, numel(S.bufD)) + 1;
    m = [mean(S.bufS); mean(S.bufD)] + S.sens.sigma_e*randn(rs, 2, 1);
    if S.sens.floor_n > 0      % Step 4: no beat inside the window -> report in tune
        fl = 1./(2*S.sens.floor_n*[S.sens.Tw_sigma; S.sens.Tw_delta]);
        m(abs(m) < fl) = 0;
    end
    S.out = m;
end
eS_m = S.out(1);
eD_m = S.out(2);
end
