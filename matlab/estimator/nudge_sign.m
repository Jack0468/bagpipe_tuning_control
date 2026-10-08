function sgn = nudge_sign(absD_before, absD_after, nudge_Hz)
%NUDGE_SIGN  Sign of e_Delta = f1 - f2 from a known nudge of tenor 1 ("nudge and listen").
%   Raising f1 by nudge_Hz > 0 increases |f1 - f2| when f1 > f2 and decreases it when
%   f1 < f2 (for |e_Delta| > nudge/2). Pipers do the same by ear (handover Section 13.2).
sgn = sign((absD_after - absD_before)*nudge_Hz);
end
