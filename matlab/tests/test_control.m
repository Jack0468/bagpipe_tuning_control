function tests = test_control
%TEST_CONTROL  Controller design and short closed-loop runs (handover Sections 14-15).
tests = functiontests(localfunctions);
end

function setupOnce(tc)
addpath(fileparts(fileparts(mfilename('fullpath'))));  startup_paths();
P = bagpipe_params();  eq = compute_equilibrium(P);  sys = lti_analytic(P, eq);
tc.TestData.P = P;  tc.TestData.eq = eq;  tc.TestData.sys = sys;
tc.TestData.ctrl = design_controllers(P, eq, sys);
end

function testLoopsStableWithMargins(tc)
LA = loop_analysis(tc.TestData.P, tc.TestData.eq, tc.TestData.sys, tc.TestData.ctrl, false);
verifyTrue(tc, LA.pressure.stable);
verifyGreaterThan(tc, LA.pressure.PM_deg, 40);
verifyGreaterThan(tc, LA.pressure.GM_dB, 6);
verifyTrue(tc, LA.slideS.stable);
verifyTrue(tc, LA.slideD.stable);
verifyEqual(tc, LA.slideS.PM_deg, tc.TestData.ctrl.opts.tuning_pm_deg, 'AbsTol', 1);
end

function testQuietRunStaysInTune(tc)
P = tc.TestData.P;
out = simulate_closed_loop(P, tc.TestData.eq, tc.TestData.ctrl, default_scenario(P, 'quiet', 1, 60));
verifyFalse(tc, out.failed);
verifyLessThan(tc, max(out.metrics.max_abs_e), P.e_target);
verifyLessThan(tc, out.metrics.max_abs_dp, 20);
end

function testBreathGapsKeepReedsSounding(tc)
P = tc.TestData.P;
out = simulate_closed_loop(P, tc.TestData.eq, tc.TestData.ctrl, default_scenario(P, 'gaps_only', 2, 60));
verifyFalse(tc, out.failed);
verifyEqual(tc, out.metrics.frac_p_in_window, 1);
verifyLessThan(tc, out.metrics.z_max, P.z_max);
end
