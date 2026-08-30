% Export_FCalibration.m
% Exports the Amendment F CALIBRATION scenario set. No controller runs.
%
% This set exists so the recovery-controller thresholds can be chosen WITHOUT
% touching the evaluation set. Its results may never be presented as the formal
% recovery test.
%
% SEED DISJOINTNESS IS STRUCTURAL, NOT PROBABILISTIC.
% Every seed used by any prior study (pilot, development, confirmation,
% fixtures, ORCA follow-up) lies in [835785, 2147023434], i.e. below 2^31.
% This set draws exclusively from [2^31, 2^32-1], so a collision is impossible
% rather than merely unlikely. The verification script re-checks this against
% the actual seed lists and does not take the claim on trust.
%
% Draws sequentially until the target count of FEASIBLE scenarios is reached,
% recording every attempt and every rejection. It does not iterate a fixed list
% and silently return fewer, which is what Export_ConfirmScenarios does by
% design for its frozen list.
%
% Usage, from the v2 directory:
%     >> Export_FCalibration
%
% Writes:  runs/v2/export_fcal/*.json
%          runs/v2/export_fcal/manifest.json

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

SEED_BASE   = 2147483648;      % 2^31 — above every previously used seed
PER_COND    = 40;              % accepted scenarios per condition
MAX_ATTEMPT = 400;             % safety stop per condition

outDir = fullfile('runs','v2','export_fcal');
if ~exist(outDir,'dir'); mkdir(outDir); end

conds = {
  'baseline',  10, 150, 48, 1.34
  'dense',     10, 150, 80, 1.34
  'cluttered', 16, 150, 48, 1.34
  'bigObs',    10, 260, 48, 1.34
};

M = struct();
M.set          = 'F_calibration';
M.purpose      = 'Amendment F recovery-controller threshold calibration. NOT an evaluation set.';
M.seed_base    = SEED_BASE;
M.per_condition = PER_COND;
M.created      = datestr(now, 'yyyy-mm-dd HH:MM:SS');
M.conditions   = struct();

nTotal = 0; nRejTotal = 0;
for ci = 1:size(conds,1)
    name = conds{ci,1};
    P = SfmParams('numObstacles', conds{ci,2}, 'maxArea', conds{ci,3}, ...
                  'numPedestrians', conds{ci,4}, 'v0Ped', conds{ci,5});

    accepted = []; rejected = [];
    attempt = 0;
    while numel(accepted) < PER_COND && attempt < MAX_ATTEMPT
        % Deterministic, reproducible, and disjoint by construction.
        s = SEED_BASE + (ci-1)*1000000 + attempt;
        attempt = attempt + 1;
        S = MakeScenario(s, P);
        if ~S.feasible
            % MakeScenario reports a single feasibility flag: the map has no
            % A* path from start to goal. Recorded per seed so the rejection
            % rate is a reported quantity, as the protocol requires.
            rejected(end+1) = s; %#ok<SAGROW>
            continue;
        end
        accepted(end+1) = s; %#ok<SAGROW>

        C.name        = sprintf('%s_%d', name, s);
        C.params      = P;
        C.obstacles   = cell(1, numel(S.xObsList));
        for j = 1:numel(S.xObsList)
            C.obstacles{j} = [S.xObsList{j}(:), S.yObsList{j}(:)];
        end
        C.robotStart  = P.start(:).';
        C.robotVel    = [0 0];
        C.pedPos      = S.pedX0.';
        C.pedVel      = S.pedV0.';
        C.pedGoal     = S.pedGoal0.';
        C.pedGoalSeed = S.pedGoalSeed;
        C.goalPool    = S.goalPool.';
        C.waypoints   = S.path.';
        C.condition   = name;
        C.matlabOutcome = '';

        fid = fopen(fullfile(outDir, sprintf('fcal_%s.json', C.name)), 'w');
        fwrite(fid, jsonencode(C, 'PrettyPrint', true), 'char');
        fclose(fid);
    end

    if numel(accepted) < PER_COND
        error('Export_FCalibration:short', ...
              '%s: only %d/%d accepted after %d attempts', ...
              name, numel(accepted), PER_COND, attempt);
    end

    M.conditions.(name) = struct( ...
        'accepted_seeds', accepted, ...
        'rejected_seeds', rejected, ...
        'n_accepted', numel(accepted), ...
        'n_rejected', numel(rejected), ...
        'attempts', attempt, ...
        'rejection_rate', numel(rejected) / max(attempt,1), ...
        'numObstacles', conds{ci,2}, 'maxArea', conds{ci,3}, ...
        'numPedestrians', conds{ci,4}, 'v0Ped', conds{ci,5});

    nTotal = nTotal + numel(accepted);
    nRejTotal = nRejTotal + numel(rejected);
    fprintf('%-10s accepted %3d  rejected %3d  (%d attempts)\n', ...
            name, numel(accepted), numel(rejected), attempt);
end

M.n_total     = nTotal;
M.n_rejected  = nRejTotal;
M.seed_min    = SEED_BASE;
M.seed_max    = SEED_BASE + 3*1000000 + MAX_ATTEMPT;

fid = fopen(fullfile(outDir,'manifest.json'), 'w');
fwrite(fid, jsonencode(M, 'PrettyPrint', true), 'char');
fclose(fid);

fprintf('\nTOTAL accepted %d, rejected %d\n', nTotal, nRejTotal);
fprintf('Wrote %s\n', fullfile(outDir,'manifest.json'));
fprintf('\nNEXT: run  python3 py/verify_fcal.py  from the repository root.\n');
