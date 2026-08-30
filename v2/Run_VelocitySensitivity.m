% Run_VelocitySensitivity.m
% Does initial velocity affect capture outcome, and over what horizon?
%
% An earlier check compared two velocities at ONE position and found
% differences of ~3e-4 beyond one second. That is not evidence that velocity
% is irrelevant: the system is nonlinear, and near a separatrix a small early
% displacement can send two trajectories into different basins even if their
% velocity traces converge. One state cannot support a general claim.
%
% This sweeps the state space properly:
%   positions   -- random free points AND points adjacent to attracting
%                  equilibrium candidates, where basin boundaries live
%   directions  -- toward the nearest equilibrium, away from it, and both
%                  tangential directions
%   speeds      -- 0 up to vMax
%   horizons    -- 0.5 to 5 s
%   maps        -- several obstacle geometries
%
% Reported per horizon:
%   endpoint spread    max over velocity pairs of ||x_a(T) - x_b(T)||
%   capture disagreement  fraction of positions where velocity choice flips
%                         the binary capture classification
%
% If classifications are stable across the sweep, the timescale argument
% holds. If they flip near basin boundaries, velocity matters there and the
% claim must be restricted.
%
% Usage:  cd to the repository root, then  run('v2/Run_VelocitySensitivity.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('nMaps','var');   nMaps   = 3;  end
if ~exist('nRandom','var'); nRandom = 6;  end
if ~exist('nBasin','var');  nBasin  = 6;  end

P = SfmParams();
horizons = [0.5 1.0 2.0 3.0 5.0];
speeds   = [0 0.5 1.0 P.v0Robot P.vMax];
nH = numel(horizons);

rng(31337, 'twister');
seeds = randi([1, 2^31-1], nMaps, 1);

spreadAll = cell(1, nH);
flipAll   = cell(1, nH);
for i = 1:nH, spreadAll{i} = []; flipAll{i} = []; end
nearBasinFlag = [];

fprintf('=== Velocity sensitivity sweep ===\n');
fprintf('%d maps, %d random + %d basin-adjacent positions each\n\n', ...
        nMaps, nRandom, nBasin);

for m = 1:nMaps
    S = MakeScenario(seeds(m), P);
    F = MapField(S, P, 0.5);

    % Random free positions, away from start and goal.
    freeIdx = find(F.free);
    pick = freeIdx(randperm(numel(freeIdx), min(200, numel(freeIdx))));
    cand = [];
    for t = pick(:).'
        pt = [F.X(t); F.Y(t)];
        if norm(pt - P.start(:)) > 5 && norm(pt - P.goal(:)) > 5
            cand(:, end+1) = pt; %#ok<AGROW>
        end
        if size(cand, 2) >= nRandom, break; end
    end

    % Basin-adjacent positions: free cells that neighbour an attracting
    % equilibrium candidate. Boundaries are where velocity should matter most.
    [iy, ix] = find(F.stable);
    basinPts = [];
    for q = 1:numel(iy)
        for dy = [-3 3]
            yy = min(max(iy(q)+dy, 1), size(F.X,1));
            pt = [F.X(yy, ix(q)); F.Y(yy, ix(q))];
            if F.free(yy, ix(q)) && norm(pt - P.goal(:)) > 5
                basinPts(:, end+1) = pt; %#ok<AGROW>
            end
        end
        if size(basinPts, 2) >= nBasin, break; end
    end

    pts = [cand, basinPts];
    isBasin = [false(1, size(cand,2)), true(1, size(basinPts,2))];

    for pI = 1:size(pts, 2)
        x0 = pts(:, pI);

        % Direction toward the nearest attracting candidate, if any.
        if any(F.stable(:))
            dd = hypot(F.X - x0(1), F.Y - x0(2));
            dd(~F.stable) = inf;
            [~, q] = min(dd(:));
            toEq = [F.X(q); F.Y(q)] - x0;
            if norm(toEq) < eps, toEq = [1; 0]; end
            toEq = toEq / norm(toEq);
        else
            toEq = (P.goal(:) - x0); toEq = toEq / norm(toEq);
        end
        perp = [-toEq(2); toEq(1)];
        dirs = [toEq, -toEq, perp, -perp];

        for hI = 1:nH
            H = horizons(hI);
            ends = []; caps = [];
            for dI = 1:size(dirs, 2)
                for sp = speeds
                    C = ForwardCapture(S, P, x0, dirs(:,dI)*sp, H);
                    ends(:, end+1) = C.endPos; %#ok<AGROW>
                    caps(end+1) = C.captureScore > 0.5; %#ok<AGROW>
                end
            end
            % Max pairwise endpoint separation.
            sp_ = 0;
            for a = 1:size(ends,2)
                for b = a+1:size(ends,2)
                    sp_ = max(sp_, norm(ends(:,a) - ends(:,b)));
                end
            end
            spreadAll{hI}(end+1) = sp_;
            flipAll{hI}(end+1) = double(any(caps) && ~all(caps));
        end
        nearBasinFlag(end+1) = isBasin(pI); %#ok<AGROW>
    end
    fprintf('map %d (seed %d): %d positions, %d basin-adjacent\n', ...
            m, seeds(m), size(pts,2), sum(isBasin));
end

%% ---- Results ---------------------------------------------------------
fprintf('\n============================================================\n');
fprintf('ENDPOINT SPREAD AND CAPTURE DISAGREEMENT vs HORIZON\n');
fprintf('============================================================\n');
fprintf('%9s %14s %14s %16s %16s\n', 'horizon', 'med spread m', ...
        'max spread m', 'flip rate all', 'flip rate basin');
nb = logical(nearBasinFlag);
for hI = 1:nH
    s = spreadAll{hI}; f = flipAll{hI};
    fprintf('%8.1fs %14.4f %14.4f %15.2f %16.2f\n', horizons(hI), ...
            median(s), max(s), mean(f), mean(f(nb)));
end

fprintf('\nInterpretation: if flip rate is ~0 at every horizon, velocity does\n');
fprintf('not change capture classification for the sampled states. Non-zero\n');
fprintf('flip rates -- especially basin-adjacent -- mean the timescale claim\n');
fprintf('must be restricted to states away from basin boundaries.\n');

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end
save(fullfile(outDir,'velocity_sensitivity.mat'), 'spreadAll','flipAll', ...
     'horizons','speeds','nearBasinFlag','seeds');
