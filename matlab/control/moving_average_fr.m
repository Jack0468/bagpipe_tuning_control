function H = moving_average_fr(Tw, f_mic, Ts, w)
%MOVING_AVERAGE_FR  Frequency response of the microphone-estimator model at w [rad/s].
%   mic_sensor_step averages samples taken every 1/f_mic over Tw and holds the result;
%   averaged over the hold phase this is a boxcar of length Nt = Tw*f_mic*n_hold steps
%   at Ts: H = (1 - z^-Nt)/(Nt*(1 - z^-1)), with linear phase (delay ~ Tw/2).
n_hold = round(1/(f_mic*Ts));
Nt = round(Tw*f_mic)*n_hold;
zi = exp(-1i*w*Ts);
H = (1 - zi.^Nt)./(Nt*(1 - zi));
H(abs(1 - zi) < 1e-12) = 1;
end
