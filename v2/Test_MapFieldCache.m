% Test_MapFieldCache.m
% MANDATORY REGRESSION TEST. The memoisation in MapField is an engineering
% change only and must not alter any value. It also must not leak between
% maps: a cache keyed carelessly would silently serve one map's field for
% another, which would corrupt every downstream analysis while looking fine.
%
% Also checks agreement with the independent path in TrapField, so the two
% implementations act as a mutual cross-check rather than a single point of
% failure.
%
% Usage:  cd to the repository root, then  run('v2/Test_MapFieldCache.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();
h = 0.5;
pass = 0; fail = 0;

rng(777, 'twister');
seeds = randi([1, 2^31-1], 2, 1);

S1 = MakeScenario(seeds(1), P);
S2 = MakeScenario(seeds(2), P);

fprintf('=== MapField cache integrity ===\n\n');

% ---- 1. Cached equals freshly computed -------------------------------
A = MapField(S1, P, h, true);       % force recompute
B = MapField(S1, P, h);             % should come from cache
C = MapField(S1, P, h, true);       % force recompute again

same = @(x, y) isequaln(x, y);
if same(A.Fx, B.Fx) && same(A.Fy, B.Fy) && same(A.stable, B.stable) && ...
   same(A.clearance, B.clearance)
    fprintf('  [PASS] cached output identical to computed output\n'); pass = pass + 1;
else
    fprintf('  [FAIL] cache returned different values\n'); fail = fail + 1;
end
if same(A.Fx, C.Fx) && same(A.stable, C.stable)
    fprintf('  [PASS] recomputation is deterministic\n'); pass = pass + 1;
else
    fprintf('  [FAIL] recomputation is not deterministic\n'); fail = fail + 1;
end

% ---- 2. No cross-map leakage -----------------------------------------
D = MapField(S2, P, h);
if D.seed == S2.seed && ~same(D.Fx, A.Fx)
    fprintf('  [PASS] a second map gets its own field\n'); pass = pass + 1;
else
    fprintf('  [FAIL] cache leaked between maps\n'); fail = fail + 1;
end
E = MapField(S1, P, h);
if same(E.Fx, A.Fx)
    fprintf('  [PASS] first map still correct after a second was cached\n'); pass = pass + 1;
else
    fprintf('  [FAIL] first map was overwritten\n'); fail = fail + 1;
end

% ---- 3. Grid step is part of the key ---------------------------------
G = MapField(S1, P, 0.25);
if abs(G.h - 0.25) < 1e-12 && ~isequal(size(G.Fx), size(A.Fx))
    fprintf('  [PASS] a different grid step gets its own entry\n'); pass = pass + 1;
else
    fprintf('  [FAIL] grid step is not part of the cache key\n'); fail = fail + 1;
end

% ---- 4. Agreement with the independent TrapField path ----------------
TF = TrapField(S1, P, h, [], []);
dFx = max(abs(A.Fx(A.free) - TF.Fx(A.free)));
dFy = max(abs(A.Fy(A.free) - TF.Fy(A.free)));
nStabDiff = sum(A.stable(:) ~= TF.stable(:));
fprintf('\n  max |dFx| vs TrapField : %.3e\n', dFx);
fprintf('  max |dFy| vs TrapField : %.3e\n', dFy);
fprintf('  stable-cell mismatches : %d\n', nStabDiff);
if dFx < 1e-9 && dFy < 1e-9 && nStabDiff == 0
    fprintf('  [PASS] independent implementations agree exactly\n'); pass = pass + 1;
else
    fprintf('  [FAIL] the two field implementations disagree\n'); fail = fail + 1;
end

% ---- 5. Clearance agrees with direct evaluation ----------------------
% Spot-check against p_poly_dist at random free points.
idx = find(A.free);
sel = idx(round(linspace(1, numel(idx), 25)));
worst = 0;
for t = sel(:).'
    d = inf;
    for k = 1:numel(S1.xObsList)
        d = min(d, p_poly_dist(A.X(t), A.Y(t), S1.xObsList{k}, S1.yObsList{k}));
    end
    worst = max(worst, abs(d - A.clearance(t)));
end
fprintf('\n  worst clearance error at 25 sampled points : %.3e\n', worst);
if worst < 1e-9
    fprintf('  [PASS] clearance field matches direct evaluation\n'); pass = pass + 1;
else
    fprintf('  [FAIL] clearance field is wrong\n'); fail = fail + 1;
end

fprintf('\n=== %d passed, %d failed ===\n', pass, fail);
if fail > 0
    error('Test_MapFieldCache:failed', '%d check(s) failed.', fail);
end
