function M = slide_design_model(P, eq)
%SLIDE_DESIGN_MODEL  Reduced slide model for control design: states [s; sd], input V [V].
%   The motor current is quasi-static (L/R_a ~ 0.1 ms is far below the mechanical time
%   constant), so  s'' = -a*s' + b*V. The full 3-state slide, including the current,
%   is kept in the simulation truth model. h is the drone pitch sensitivity [Hz/m].
sp = P.slide(1);
M.a = (sp.b_s + sp.K_e*sp.K_t*sp.G^2/sp.R_a)/sp.m_eff;   % effective damping [1/s]
M.b = sp.G*sp.K_t/(sp.R_a*sp.m_eff);                      % [m/s^2 per V]
M.h = mean(eq.h_s);
M.A = [0, 1; 0, -M.a];
M.B = [0; M.b];
end
