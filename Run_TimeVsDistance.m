% Run_TimeVsDistance.m
% Averages the distance-from-goal trajectory over time for SFM-only vs hybrid
% (A*+SFM) across many trials (yadat8). Fully reproducible: a master seed
% derives a per-trial seed, and all seeds are logged to runs/.

if ~exist('numTrials','var');   numTrials   = 100; end
if ~exist('MASTER_SEED','var'); MASTER_SEED = 20260617; end

if ~exist('runs','dir'); mkdir('runs'); end

rng(MASTER_SEED,'twister');
allSeeds = randi([1, 2^31-1], numTrials, 1);

dist_trials = zeros(numTrials, 2, 3000);
for i = 1:numTrials
    rng(allSeeds(i),'twister');
    dist_trials(i,:,:) = Visual_Distance_v_Time();
    fprintf('trial %d/%d seed=%d\n', i, numTrials, allSeeds(i));
end

dist_avg = squeeze(sum(dist_trials, 1) / numTrials);   % 2 x 3000
STD      = std(dist_trials, 0, 1);
CI       = squeeze(1.96 * STD / sqrt(numTrials));      % 2 x 3000

Trial = (1:numTrials)';
Seed  = allSeeds;
writetable(table(Trial, Seed), 'runs/time_vs_distance_seeds.csv');
save('runs/time_vs_distance.mat', 'dist_avg', 'CI', 'dist_trials', 'allSeeds', 'numTrials', 'MASTER_SEED');

fprintf('Saved time-vs-distance data over %d trials.\n', numTrials);
