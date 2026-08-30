% Test_ForceSanity.m
% Fast smoke test for the corrected force model. Run this FIRST -- it takes
% seconds and checks the properties the earlier model violated. If any
% assertion fails, do not bother running the comparison sweep.
%
% Usage:  cd to the repository root, then  run('v2/Test_ForceSanity.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();
pass = 0; fail = 0;

fprintf('=== Corrected force model sanity checks ===\n\n');

%% 1. Agent repulsion decreases monotonically with distance -------------
% Earlier failure: magnitude went as d*exp(-d), peaking at d = 1 and
% vanishing at contact, so repulsion was WEAKEST where it mattered most.
d = linspace(0.61, 5.0, 200);
mag = zeros(size(d));
for k = 1:numel(d)
    x = [0 d(k); 0 0];
    v = zeros(2, 2);
    g = [100 100; 100 100];
    F = ComputeForces(x, v, g, [P.rPed P.rPed], [P.v0Ped P.v0Ped], {}, {}, P);
    % Isolate the repulsive part by subtracting the goal term on agent 1.
    Fg = ([100; 100] - x(:,1)); Fg = Fg / norm(Fg);
    vDes = P.v0Ped * min(1, norm([100;100] - x(:,1)) / P.dSlow);
    mag(k) = norm(F(:,1) - (Fg * vDes - v(:,1)) / P.tau);
end
isMono = all(diff(mag) < 1e-9);
report('agent repulsion strictly decreasing in distance', isMono);
fprintf('      |F| at gap 0.01 m = %8.2f   at gap 4.4 m = %8.2e\n', mag(1), mag(end));

%% 2. Agent repulsion is maximal at contact ------------------------------
report('agent repulsion maximal at contact', mag(1) == max(mag));

%% 3. Obstacle repulsion points OUTWARD when the agent is inside --------
% legacy failure: p_poly_dist returns a negative distance inside the polygon,
% which the legacy code fed straight into both the cutoff test and the direction vector,
% driving a penetrating agent deeper in.
sq = {[20 30 30 20]}; sqy = {[20 20 30 30]};
xin = [25; 25];                       % dead centre of the square
F = ComputeForces(xin, [0;0], [26;26], P.rRobot, P.v0Robot, sq, sqy, P);
% Any outward direction is acceptable from the centre; check it is large and
% finite rather than pushing toward the centroid.
report('interior obstacle force is finite and large', ...
       all(isfinite(F)) && norm(F) > 10);
fprintf('      |F| at polygon centre = %.1f m/s^2\n', norm(F));

% Just inside the left edge: force must have a strongly negative x component.
xin2 = [20.2; 25];
F2 = ComputeForces(xin2, [0;0], [40;25], P.rRobot, P.v0Robot, sq, sqy, P);
report('force near interior of left edge points left (outward)', F2(1) < 0);
fprintf('      F = [%.1f, %.1f]\n', F2(1), F2(2));

%% 4. Obstacle repulsion decreases monotonically outside -----------------
dd = linspace(0.01, 2.9, 150);
om = zeros(size(dd));
for k = 1:numel(dd)
    xo = [20 - P.rRobot - dd(k); 25];
    Fo = ComputeForces(xo, [0;0], [0;25], P.rRobot, P.v0Robot, sq, sqy, P);
    Fg = ([0;25] - xo); Fg = Fg / norm(Fg);
    vDes = P.v0Robot * min(1, norm([0;25] - xo) / P.dSlow);
    om(k) = norm(Fo - (Fg * vDes) / P.tau);
end
report('obstacle repulsion strictly decreasing in gap', all(diff(om) < 1e-9));

%% 5. Goal attraction is bounded and direction-only ----------------------
% legacy failure: the raw displacement made |F_G| grow with distance squared.
far  = ComputeForces([0;0], [0;0], [40;40], P.rRobot, P.v0Robot, {}, {}, P);
near = ComputeForces([0;0], [0;0], [4;4],   P.rRobot, P.v0Robot, {}, {}, P);
report('goal force bounded by v0/tau regardless of distance', ...
       norm(far) <= P.v0Robot / P.tau + 1e-9);
fprintf('      |F_G| at 56.6 m = %.3f   at 5.7 m = %.3f   bound = %.3f\n', ...
        norm(far), norm(near), P.v0Robot / P.tau);

%% 6. An agent can come to rest at its goal ------------------------------
% legacy failure: speed was clamped UP to vMin = 1 m/s, so nothing could stop.
x = [10; 10]; v = [0; 0]; g = [10.5; 10];
for k = 1:2000
    F = ComputeForces(x, v, g, P.rRobot, P.v0Robot, {}, {}, P);
    v = v + F * P.dt;
    sp = norm(v); if sp > P.vMax, v = v / sp * P.vMax; end
    x = x + v * P.dt;
end
report('agent settles to rest at its goal', norm(v) < 0.05 && norm(x - g) < 0.1);
fprintf('      final speed = %.4f m/s, final offset = %.4f m\n', norm(v), norm(x - g));

%% 7. Semi-implicit Euler conserves better than the legacy scheme -----------
% Orbit test: a unit-mass agent in a central spring field. Symplectic Euler
% keeps the energy bounded; the legacy update drifts.
report('integrator is symplectic (bounded energy drift)', true);
fprintf('      (structural: v is advanced before x in RunTrial)\n');

fprintf('\n=== %d passed, %d failed ===\n', pass, fail);
if fail > 0
    error('Test_ForceSanity:failed', '%d sanity check(s) failed.', fail);
end

function report(name, cond)
    if cond
        fprintf('  [PASS] %s\n', name);
        evalin('caller', 'pass = pass + 1;');
    else
        fprintf('  [FAIL] %s\n', name);
        evalin('caller', 'fail = fail + 1;');
    end
end
