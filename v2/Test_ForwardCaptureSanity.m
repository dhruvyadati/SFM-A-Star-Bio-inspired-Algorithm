% Test_ForwardCaptureSanity.m
% MANDATORY REGRESSION TEST for the rollout features, on controlled cases
% where the correct ranking is known in advance.
%
% Run this BEFORE applying the features to trial data. Two earlier composite
% scores looked reasonable and were wrong: one classified a robot decelerating
% to ARRIVE as maximally trapped, the other could not tell "trapped" from
% "little time elapsed" below a 2 s horizon. Both would have been caught here.
%
% A purpose-built map is used so the expected answer is not a matter of
% opinion. The geometry is an L-shaped corner whose interior angle faces AWAY
% from the goal at (50,50):
%
%   solid = {x in [20,32], y in [27,30]}  union  {x in [30,32], y in [20,30]}
%
% Goal attraction from inside the corner points up and to the right, into the
% horizontal wall; sliding right then meets the vertical wall. Both walls push
% back down-left, so the corner at (30,27) is a genuine equilibrium.
%
% An earlier version of this test used a pocket opening DOWNWARD while the
% goal lay up-right, so the robot slid along the wall and escaped around the
% edge. It was never a trap, and the test failed for that reason rather than
% because the features were wrong -- worth recording, since a broken fixture
% is indistinguishable from a broken metric if the geometry is not checked.
%
% Expected ranking:
%   HIGH risk : at rest in the corner; moving into the corner
%   LOW  risk : moving away from the corner; detour clear of the obstacle;
%               decelerating near the goal; open-space cruising; brief
%               slowdown then recovery
%
% Usage:  cd to the repository root, then  run('v2/Test_ForwardCaptureSanity.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();

% A concave pocket facing the start, with the goal beyond it. A robot pushed
% into the mouth of the pocket has goal attraction pointing into the wall.
S.seed = -1;
S.xObsList = { [20 30 30 32 32 20] };
S.yObsList = { [27 27 20 20 30 30] };
S.feasible = true;
S.P = P;

cases = {
%  label                              position      velocity            expect
  'at rest in corner',               [28; 25],     [0; 0],             'HIGH'
  'moving into corner',              [25; 24],     [0.92; 0.92],       'HIGH'
  'moving away from corner',         [28; 25],     [-1.30; 0],         'LOW'
  'detour clear of obstacle',        [15; 25],     [0; 1.30],          'LOW'
  'decelerating near goal',          [48; 48],     [0.20; 0.20],       'LOW'
  'cruising open space',             [10; 45],     [0.92; 0.92],       'LOW'
  'slowdown then recovery',          [10; 10],     [0.10; 0.10],       'LOW'
};

fprintf('=== ForwardCapture feature sanity (H = 3 s, L-corner map) ===\n\n');
fprintf('%-28s %6s %7s %8s %8s %8s %7s %6s %s\n', 'case', 'tailV', 'progR', ...
        'fracSlow', 'tailRate', 'distEq', 'arrive', 'slope', 'expect');
fprintf('%s\n', repmat('-', 1, 104));

n = size(cases, 1);
tailV = zeros(n,1); progR = zeros(n,1); fSlow = zeros(n,1);
arrived = false(n,1);
for k = 1:n
    C = ForwardCapture(S, P, cases{k,2}, cases{k,3}, 3.0);
    tailV(k) = C.tailSpeed; progR(k) = C.progressRatio; fSlow(k) = C.fracSlow;
    arrived(k) = C.arrived;
    fprintf('%-28s %6.3f %7.3f %8.3f %8.3f %8.2f %7d %6.2f %s\n', ...
            cases{k,1}, C.tailSpeed, C.progressRatio, C.fracSlow, ...
            C.tailProgressRate, C.distToEquilib, C.arrived, C.speedSlope, cases{k,4});
end

%% ---- Assertions ------------------------------------------------------
pass = 0; fail = 0;
isHigh = strcmp(cases(:,4), 'HIGH');

% 1. Arriving near the goal must never be flagged, whatever its speed.
if arrived(5)
    fprintf('\n  [PASS] near-goal deceleration is marked arrived, not trapped\n');
    pass = pass + 1;
else
    fprintf('\n  [FAIL] near-goal deceleration was NOT marked arrived\n');
    fail = fail + 1;
end

% 2. Open-space cruising must show near-nominal progress.
if progR(6) > 0.5
    fprintf('  [PASS] open-space cruising shows near-nominal progress (%.2f)\n', progR(6));
    pass = pass + 1;
else
    fprintf('  [FAIL] open-space cruising progress too low (%.2f)\n', progR(6));
    fail = fail + 1;
end

% 3. HIGH cases must end slower than the non-arriving LOW cases.
lowMoving = ~isHigh & ~arrived;
if max(tailV(isHigh)) < min(tailV(lowMoving))
    fprintf('  [PASS] all HIGH cases end slower than every moving LOW case\n');
    pass = pass + 1;
else
    fprintf('  [FAIL] tail speed does not separate HIGH from moving LOW\n');
    fprintf('         HIGH max %.3f vs LOW min %.3f\n', ...
            max(tailV(isHigh)), min(tailV(lowMoving)));
    fail = fail + 1;
end

% 4. HIGH cases must make materially less progress than moving LOW cases.
if max(progR(isHigh)) < min(progR(lowMoving))
    fprintf('  [PASS] progress ratio separates HIGH from moving LOW\n');
    pass = pass + 1;
else
    fprintf('  [FAIL] progress ratio does not separate HIGH from moving LOW\n');
    fprintf('         HIGH max %.3f vs LOW min %.3f\n', ...
            max(progR(isHigh)), min(progR(lowMoving)));
    fail = fail + 1;
end

% 5. A brief slowdown that recovers must not look captured.
if progR(7) > 0.3
    fprintf('  [PASS] slowdown-then-recovery is not flagged (progR %.2f)\n', progR(7));
    pass = pass + 1;
else
    fprintf('  [FAIL] slowdown-then-recovery looks captured (progR %.2f)\n', progR(7));
    fail = fail + 1;
end

fprintf('\n=== %d passed, %d failed ===\n', pass, fail);
if fail > 0
    error('Test_ForwardCaptureSanity:failed', ...
          '%d check(s) failed. Do not apply these features to trial data yet.', fail);
end
