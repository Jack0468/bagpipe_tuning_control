function tests = test_model
%TEST_MODEL  Unit tests for the plant model and its linearisation (handover Sections 6-9).
tests = functiontests(localfunctions);
end

function setupOnce(tc)
addpath(fileparts(fileparts(mfilename('fullpath'))));  startup_paths();
tc.TestData.P = bagpipe_params();
tc.TestData.eq = compute_equilibrium(tc.TestData.P);
tc.TestData.sys = lti_analytic(tc.TestData.P, tc.TestData.eq);
end

function testEquilibriumIsSteady(tc)
P = tc.TestData.P;  eq = tc.TestData.eq;
xdot = plant_rhs(eq.xbar, eq.ubar, eq.dbar, P);
verifyLessThan(tc, abs(xdot(1))/eq.qbar, 1e-9);
verifyLessThan(tc, max(abs(xdot(2:end))), 1e-6);
y = plant_outputs(eq.xbar, eq.dbar, P);
verifyEqual(tc, y, eq.ybar, 'AbsTol', 1e-9);
end

function testAnalyticMatchesNumericJacobian(tc)
a = tc.TestData.sys;  n = lti_numeric(tc.TestData.P, tc.TestData.eq);
for f = {'A', 'B', 'C', 'E', 'Dd'}
    X = a.(f{1});  Y = n.(f{1});
    verifyLessThan(tc, max(abs(X(:) - Y(:)))/max(abs(X(:))), 1e-6, f{1});
end
end

function testBagBlockFormulas(tc)
R = structural_analysis(tc.TestData.sys, tc.TestData.P, tc.TestData.eq, false);
verifyEqual(tc, R.det_obsv_bag, R.det_obsv_bag_formula, 'RelTol', 1e-6);
verifyEqual(tc, R.det_Abag, R.det_Abag_formula, 'RelTol', 1e-6);
end

function testReachableAndObservable(tc)
R = structural_analysis(tc.TestData.sys, tc.TestData.P, tc.TestData.eq, false);
verifyEqual(tc, R.rank_ctrb, 9);
verifyEqual(tc, R.rank_obsv, 9);
end

function testSqueezeCannotHoldPressure(tc)
% p/F has a zero at s = 0 (Section 9.4): the DC gain from F to p is zero.
G = ss(tc.TestData.sys.A, tc.TestData.sys.B, tc.TestData.sys.C, 0);
verifyLessThan(tc, abs(dcgain(G(1, 2))), 1e-6*abs(evalfr(G(1, 2), 1i)));
end

function testCommonDifferentialDecoupled(tc)
cd = common_diff(tc.TestData.sys);
verifyLessThan(tc, max(max(abs(cd.A(4:6, 7:9)))), 1e-9);
verifyLessThan(tc, max(max(abs(cd.A(7:9, 4:6)))), 1e-9);
verifyLessThan(tc, max(abs(cd.B(4:6, 4))), 1e-9);
verifyLessThan(tc, max(abs(cd.B(7:9, 3))), 1e-9);
% chanter drift enters e_Sigma only
verifyEqual(tc, cd.Dd(3, 5), 0, 'AbsTol', 1e-12);
end

function testBreathGapsRespectBounds(tc)
P = tc.TestData.P;
g = breath_gaps(P, 2000, 'random', 3);
gap = diff(g(1:end-1, :), 1, 2);
blow = g(2:end, 1) - g(1:end-1, 2);
verifyGreaterThanOrEqual(tc, min(gap), P.gap.min);
verifyLessThanOrEqual(tc, max(gap), P.gap.max);
verifyGreaterThanOrEqual(tc, min(blow), P.blow.min);
w = breath_gaps(P, 60, 'worst');
verifyEqual(tc, diff(w(1, :)), P.gap.max, 'AbsTol', 1e-12);
end
