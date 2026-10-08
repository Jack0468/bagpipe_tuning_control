function p = bag_pressure(m, z, T_bag, P)
%BAG_PRESSURE  Gauge bag pressure [Pa] from the ideal gas law, handover Section 6.1.
p = m.*P.R.*T_bag./(P.V0 - P.A_c.*z) - P.p_atm;
end
