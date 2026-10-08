function [y, f] = plant_outputs(x, d, P)
%PLANT_OUTPUTS  Outputs y = [p; e1; e2] (Pa, Hz, Hz), handover Sections 6.6-6.7.
%   f = [f_c; f_1; f_2] are the chanter Low A and tenor drone pitches [Hz].
%   Pitch models are in deviation form about pbar, so fc0 is the playing pitch.
p  = bag_pressure(x(1), x(2), d(1), P);
dp = p - P.pbar;
fc = P.fc0*sound_speed(d(4), P)/sound_speed(P.Tbar, P) + P.beta_c*dp + d(5);
f1 = sound_speed(d(2), P)/(4*(P.L_d1 + x(4) + P.L_e)) + P.beta_d1*dp;
f2 = sound_speed(d(3), P)/(4*(P.L_d2 + x(7) + P.L_e)) + P.beta_d2*dp;
y = [p; f1 - fc/2; f2 - fc/2];
f = [fc; f1; f2];
end
