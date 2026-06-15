% Run_SuccessVsObstacles.m
% Sweeps the number of static obstacles and records SFM-only vs hybrid
% (A*+SFM) success rates. Fully reproducible: a single master seed derives a
% per-trial seed, and every seed is logged to runs/ so any trial can be
% replayed exactly with rng(seed,'twister').

if ~exist('obstacleCounts','var'); obstacleCounts = [2 4 6 8 10 12 14]; end  % yadat6
if ~exist('numTrials','var');      numTrials      = 50;  end                 % trials per value
if ~exist('MASTER_SEED','var');    MASTER_SEED    = 20260614; end

if ~exist('runs','dir'); mkdir('runs'); end

rng(MASTER_SEED,'twister');
nRows    = numel(obstacleCounts) * numTrials;
allSeeds = randi([1, 2^31-1], nRows, 1);

NumObstacles   = zeros(nRows,1);
Trial          = zeros(nRows,1);
Seed           = zeros(nRows,1);
SFM_success    = zeros(nRows,1);
Hybrid_success = zeros(nRows,1);

row = 0;
for ci = 1:numel(obstacleCounts)
    nObs = obstacleCounts(ci);
    for tr = 1:numTrials
        row = row + 1;
        s = allSeeds(row);
        rng(s,'twister');
        [sfm, hybrid] = SuccessParam(nObs, 150);
        NumObstacles(row)   = nObs;
        Trial(row)          = tr;
        Seed(row)           = s;
        SFM_success(row)    = sfm;
        Hybrid_success(row) = hybrid;
        fprintf('obs=%2d trial=%2d/%d seed=%d sfm=%d hybrid=%d\n', ...
            nObs, tr, numTrials, s, sfm, hybrid);
    end
end

T = table(NumObstacles, Trial, Seed, SFM_success, Hybrid_success);
writetable(T, 'runs/success_vs_obstacles_seeds.csv');

uObs    = unique(NumObstacles);
SFM_rate_pct    = zeros(numel(uObs),1);
Hybrid_rate_pct = zeros(numel(uObs),1);
for k = 1:numel(uObs)
    m = NumObstacles == uObs(k);
    SFM_rate_pct(k)    = 100*mean(SFM_success(m));
    Hybrid_rate_pct(k) = 100*mean(Hybrid_success(m));
end
S = table(uObs, SFM_rate_pct, Hybrid_rate_pct, ...
    'VariableNames', {'NumObstacles','SFM_rate_pct','Hybrid_rate_pct'});
writetable(S, 'runs/success_vs_obstacles_summary.csv');
save('runs/success_vs_obstacles.mat');

disp('=== Success vs Number of Obstacles ===');
disp(S);
