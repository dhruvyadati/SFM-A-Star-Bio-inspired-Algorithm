% Run_TrapValidation.m
% Does the force-field trap diagnostic predict WHERE deadlocks occur, and
% does it beat cheap alternatives?
%
% The legacy analysis reported a trap ratio of 0.20 and argued from it that
% one-fifth of the workspace is hazardous. That is unfalsifiable as posed: it
% never checked whether real failures land inside those regions. The trap
% ratio's magnitude is not the quantity of interest. LIFT is:
%
%     lift = P(deadlock lands in a flagged region) / (flagged area fraction)
%
% A 2% flagged area capturing 60% of deadlocks (lift 30) is a strong
% diagnostic. A 20% area capturing 22% (lift 1.1) is worthless however
% convincing the contour plot looks.
%
% Every candidate is reduced to the same form -- a scalar field, thresholded
% to flag some fraction of free workspace -- so the force-field diagnostic
% competes against simple predictors on identical footing:
%
%   forceMag      ||F_net|| low                        (the legacy diagnostic)
%   attracting    ||F_net|| low AND locally attracting (stability-refined)
%   pedDensity    many pedestrians nearby              (crowding)
%   obsClearance  little room to the nearest obstacle  (clutter)
%   goalDist      far from the goal                    (trivial control)
%
% If a simple predictor matches the diagnostic, the diagnostic is not adding
% information and cannot carry a novelty claim.
%
% Threshold selection is HELD OUT: a calibration half picks the operating
% point, an evaluation half reports capture and lift there.
%
% Usage:  cd to the repository root, then  run('v2/Run_TrapValidation.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('gridStep','var');  gridStep  = 0.5; end
if ~exist('maxCases','var');  maxCases  = 40;  end
if ~exist('batchFile','var'); batchFile = fullfile('runs','v2','safety_gate.mat'); end
if ~exist('armName','var');   armName   = 'sfm'; end   % nominal reactive arm

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end

L = load(batchFile);
T0 = L.T; P = L.P;

sub  = T0(strcmp(T0.method, armName), :);
dead = sub(strcmp(sub.outcome, 'deadlock'), :);
nCase = min(maxCases, height(dead));
fprintf('=== Trap diagnostic validation (arm: %s) ===\n', armName);
fprintf('Deadlocks available: %d, using %d, grid %.2f m\n\n', ...
        height(dead), nCase, gridStep);
if nCase < 5
    error('Run_TrapValidation:tooFew', 'Need >= 5 deadlocks; found %d.', height(dead));
end

% Sweep the flagged AREA rather than each field's native threshold, so every
% predictor shares a common x-axis.
fracs = [0.01 0.02 0.05 0.10 0.15 0.20 0.30 0.40 0.50];
nF = numel(fracs);
% forceMag/attracting use the field AT the deadlock moment. Those are
% POST-HOC and partly circular: a stalled robot must sit at a force
% equilibrium, so finding one there restates Newton's second law. They are
% kept only as an upper bound.
%
% staticPre and staticPreAttr use the field computed from the STATIC MAP
% BEFORE the run, with no pedestrians. Those are genuinely prospective and
% are the only ones that can support a predictive claim.
preds = {'forceMag','attracting','staticPre','staticPreAttr', ...
         'pedDensity','obsClearance','goalDist'};
nP = numel(preds);

capture    = nan(nCase, nP, nF);
gradAtDead = nan(nCase, 1);
avgGrad    = nan(nCase, 1);
zAtDead    = nan(nCase, 1);

for c = 1:nCase
    seed = dead.seed(c);
    S = MakeScenario(seed, P);
    R = RunTrial(S, armName, struct('logTrajectory', true));
    if ~strcmp(R.outcome, 'deadlock')
        fprintf('case %2d seed=%-11d replay differs (%s) -- skipped\n', c, seed, R.outcome);
        continue;
    end
    dpos = R.finalRobotPos;

    % Field WITH pedestrians as they stood at the deadlock: the field the
    % robot actually experienced, not a static abstraction of it.
    F = TrapField(S, P, gridStep, R.pedPos, R.pedVel);
    free = ~F.inObs & isfinite(F.Z);

    % Component-wise interpolation, then magnitude. See the note in
    % TrapField: sampling the magnitude field directly is invalid near an
    % equilibrium and overstates the force by orders of magnitude.
    fxD = interp2(F.X, F.Y, F.Fx, dpos(1), dpos(2), 'linear', NaN);
    fyD = interp2(F.X, F.Y, F.Fy, dpos(1), dpos(2), 'linear', NaN);
    zAtDead(c)    = hypot(fxD, fyD);
    avgGrad(c)    = mean(F.gradMag(free), 'omitnan');
    gradAtDead(c) = interp2(F.X, F.Y, F.gradMag, dpos(1), dpos(2), 'linear', NaN);

    % Every predictor oriented so LOWER means "more suspicious".
    fields = cell(1, nP);
    fields{1} = F.Z;
    A = F.Z; A(~F.stable) = inf;
    fields{2} = A;
    % Value at the deadlock for the two force-based predictors comes from the
    % component interpolation above, not from sampling the magnitude grid.
    deadOverride = [zAtDead(c), zAtDead(c)];

    % Prospective: static map only, no pedestrians, knowable before the run.
    FS = TrapField(S, P, gridStep, [], []);
    fields{3} = FS.Z;
    AS = FS.Z; AS(~FS.stable) = inf;
    fields{4} = AS;

    D = zeros(size(F.X));
    for j = 1:size(R.pedPos, 2)
        D = D + (hypot(F.X - R.pedPos(1,j), F.Y - R.pedPos(2,j)) <= 2.0);
    end
    fields{5} = -D;

    C = inf(size(F.X));
    for k = 1:numel(S.xObsList)
        xv = S.xObsList{k}; yv = S.yObsList{k};
        for ii = 1:numel(F.X)
            if F.inObs(ii), continue; end
            C(ii) = min(C(ii), p_poly_dist(F.X(ii), F.Y(ii), xv, yv));
        end
    end
    fields{6} = C;
    fields{7} = -hypot(F.X - P.goal(1), F.Y - P.goal(2));

    for pI = 1:nP
        V = fields{pI};
        if pI <= 2
            vDead = deadOverride(pI);
            if pI == 2
                % Only counts if the deadlock sits on an attracting candidate.
                st = interp2(F.X, F.Y, double(F.stable), dpos(1), dpos(2), 'nearest', 0);
                if st < 0.5, vDead = inf; end
            end
        elseif pI == 3 || pI == 4
            % Component interpolation again -- same kink problem applies.
            fxS = interp2(FS.X, FS.Y, FS.Fx, dpos(1), dpos(2), 'linear', NaN);
            fyS = interp2(FS.X, FS.Y, FS.Fy, dpos(1), dpos(2), 'linear', NaN);
            vDead = hypot(fxS, fyS);
            if pI == 4
                stS = interp2(FS.X, FS.Y, double(FS.stable), dpos(1), dpos(2), 'nearest', 0);
                if stS < 0.5, vDead = inf; end
            end
        else
            vDead = interp2(F.X, F.Y, V, dpos(1), dpos(2), 'linear', NaN);
        end
        if ~isfinite(vDead) && pI ~= 2 && pI ~= 4, continue; end
        vFree = V(free);
        for fI = 1:nF
            capture(c, pI, fI) = vDead <= quantile_(vFree, fracs(fI));
        end
    end

    fprintf('case %2d seed=%-11d dead=(%5.1f,%5.1f)  |F|=%7.4f\n', ...
            c, seed, dpos(1), dpos(2), zAtDead(c));
end

%% ---- Capture vs flagged area ----------------------------------------
fprintf('\n============================================================\n');
fprintf('CAPTURE RATE vs FLAGGED AREA   (chance: capture == area)\n');
fprintf('============================================================\n');
fprintf('%-14s', 'area flagged'); fprintf('%8.0f%%', 100*fracs); fprintf('\n');
for pI = 1:nP
    fprintf('%-14s', preds{pI});
    for fI = 1:nF, fprintf('%9.2f', mean(capture(:, pI, fI), 'omitnan')); end
    fprintf('\n');
end

fprintf('\nLIFT over chance (capture / area):\n');
fprintf('%-14s', 'area flagged'); fprintf('%8.0f%%', 100*fracs); fprintf('\n');
for pI = 1:nP
    fprintf('%-14s', preds{pI});
    for fI = 1:nF
        fprintf('%9.1f', mean(capture(:, pI, fI), 'omitnan') / fracs(fI));
    end
    fprintf('\n');
end

%% ---- Held-out operating point ---------------------------------------
valid = find(all(isfinite(capture(:,1,:)), 3));
cal = valid(1:2:end); eva = valid(2:2:end);
fprintf('\n--- Held-out threshold selection (cal %d / eval %d) ---\n', ...
        numel(cal), numel(eva));
heldLift = zeros(1, nP);
if numel(cal) >= 3 && numel(eva) >= 3
    fprintf('%-14s %10s %10s %10s\n', 'predictor', 'area', 'capture', 'lift');
    for pI = 1:nP
        liftCal = zeros(1, nF);
        for fI = 1:nF
            cc = mean(capture(cal, pI, fI), 'omitnan');
            % Require non-trivial capture so lift is not maximised by a
            % vanishing flagged area that captures almost nothing.
            if cc >= 0.2, liftCal(fI) = cc / fracs(fI); end
        end
        [~, kb] = max(liftCal);
        cE = mean(capture(eva, pI, kb), 'omitnan');
        heldLift(pI) = cE / fracs(kb);
        fprintf('%-14s %9.0f%% %10.2f %10.1f\n', preds{pI}, 100*fracs(kb), cE, heldLift(pI));
    end
else
    fprintf('  too few valid cases for a held-out split\n');
end

fprintf('\nAverage gradient over free space : %.2f s^-2\n', mean(avgGrad, 'omitnan'));
fprintf('Gradient at deadlock locations   : %.2f s^-2\n', mean(gradAtDead, 'omitnan'));

save(fullfile(outDir,'trap_validation.mat'), 'capture','fracs','preds', ...
     'heldLift','avgGrad','gradAtDead','zAtDead','P','armName');

fprintf('\n------------------------------------------------------------\n');
fprintf('VERDICT\n');
diagLift = max(heldLift(3:4));      % PROSPECTIVE only
[bestOther, iOther] = max(heldLift(5:end));
fprintf('  post-hoc lift (circular, upper bound): %.1f\n', max(heldLift(1:2)));
fprintf('  PROSPECTIVE static-field lift        : %.1f\n', diagLift);
fprintf('  best simple-predictor lift           : %.1f (%s)\n', bestOther, preds{4+iOther});
if diagLift >= 3.0 && diagLift > 1.5 * bestOther
    fprintf('  PASS. Predicts deadlock location well above chance AND clearly\n');
    fprintf('  beats the simple predictors.\n');
elseif diagLift > bestOther
    fprintf('  MARGINAL. Ahead of the simple predictors, but not decisively.\n');
else
    fprintf('  FAIL. A simple predictor matches or beats the force-field\n');
    fprintf('  diagnostic, so it adds no information. Do not build a novelty\n');
    fprintf('  claim on it.\n');
end
fprintf('------------------------------------------------------------\n');

% Toolbox-free quantile, linear interpolation on sorted order statistics.
function q = quantile_(v, p)
    v = sort(v(isfinite(v)));
    n = numel(v);
    if n == 0, q = NaN; return; end
    idx = max(1, min(n, p * n));
    lo = floor(idx); hi = ceil(idx);
    if lo == hi, q = v(lo); else, q = v(lo) + (idx - lo) * (v(hi) - v(lo)); end
end
