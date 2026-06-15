% Run_ForceFieldStats.m
% Runs the SFM force-field analysis N times and averages the trap-point
% metrics (Table 1). Fully reproducible: a master seed derives a per-trial
% seed, and all seeds are logged to runs/.

if ~exist('N','var');           N           = 50; end
if ~exist('MASTER_SEED','var'); MASTER_SEED = 20260618; end

if ~exist('runs','dir'); mkdir('runs'); end

rng(MASTER_SEED,'twister');
allSeeds = randi([1, 2^31-1], N, 1);

trapPoints    = zeros(N,1);
trapRatio     = zeros(N,1);
totalForce    = zeros(N,1);
normForce     = zeros(N,1);
avgGrad       = zeros(N,1);
maxGrad       = zeros(N,1);
numBasins     = zeros(N,1);
meanBasinSize = zeros(N,1);
maxBasinSize  = zeros(N,1);

for i = 1:N
    rng(allSeeds(i),'twister');
    fprintf('force-field trial %d/%d seed=%d\n', i, N, allSeeds(i));
    stats = PlotNetForceFromAgentPerspective();
    trapPoints(i)    = stats.trapPoints;
    trapRatio(i)     = stats.trapRatio;
    totalForce(i)    = stats.totalForceAlongPath;
    normForce(i)     = stats.normForceCost;
    avgGrad(i)       = stats.avgGradient;
    maxGrad(i)       = stats.maxGradient;
    numBasins(i)     = stats.numBasins;
    meanBasinSize(i) = stats.meanBasinSize;
    maxBasinSize(i)  = stats.maxBasinSize;
    close all;
end

Trial = (1:N)';
Seed  = allSeeds;
perTrial = table(Trial, Seed, trapPoints, trapRatio, totalForce, normForce, ...
    avgGrad, maxGrad, numBasins, meanBasinSize, maxBasinSize);
writetable(perTrial, 'runs/force_field_seeds.csv');

Metric = {'Trap Points';'Trap Ratio';'Force Cost Along Path';'Normalized Force Cost'; ...
    'Average Gradient';'Max Gradient';'Number of Basins';'Mean Basin Size';'Max Basin Size'};
Mean = [mean(trapPoints); mean(trapRatio); mean(totalForce); mean(normForce); ...
    mean(avgGrad); mean(maxGrad); mean(numBasins); mean(meanBasinSize); mean(maxBasinSize)];
StdDev = [std(trapPoints); std(trapRatio); std(totalForce); std(normForce); ...
    std(avgGrad); std(maxGrad); std(numBasins); std(meanBasinSize); std(maxBasinSize)];
summary = table(Metric, Mean, StdDev);
writetable(summary, 'runs/force_field_summary.csv');
save('runs/force_field_stats.mat');

disp('=== Force-Field Metrics (mean +/- std over trials) ===');
disp(summary);
