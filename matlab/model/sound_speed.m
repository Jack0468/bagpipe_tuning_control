function c = sound_speed(T, P)
%SOUND_SPEED  Speed of sound in air [m/s] at temperature T [K], handover Section 6.6.
c = P.c0*sqrt(T/273.15);
end
