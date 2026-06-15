% Run_SuccessVsArea.m
% Sweeps the maximum obstacle area and records SFM-only vs hybrid (A*+SFM)
% success rates. Fully reproducible: a single master seed derives a per-trial
% seed, and every seed is logged to runs/.

if ~exist('areaValues','var'); areaValues = [50 75 100 125 150 175 200 225 250]; end  % yadat7
if ~exist('numTrials','var');  numTrials  = 50;  end                                  % trials per value
if ~exist('MASTER_SEED','var'); MASTER_SEED = 20260615; end

if ~exist('runs','dir'); mkdir('runs'); end

rng(MASTER_SEED,'twister');
nRows    = numel(areaValues) * numTrials;
allSeeds = randi([1, 2^31-1], nRows, 1);

MaxArea        = zeros(nRows,1);
Trial          = zeros(nRows,1);
Seed           = zeros(nRows,1);
SFM_success    = zeros(nRows,1);
Hybrid_success = zeros(nRows,1);

row = 0;
for ai = 1:numel(areaValues)
    area = areaValues(ai);
    for tr = 1:numTrials
        row = row + 1;
        s = allSeeds(row);
        rng(s,'twister');
        [sfm, hybrid] = SuccessParam(10, area);
        MaxArea(row)        = area;
        Trial(row)          = tr;
        Seed(row)           = s;
        SFM_success(row)    = sfm;
        Hybrid_success(row) = hybrid;
        fprintf('area=%3d trial=%2d/%d seed=%d sfm=%d hybrid=%d\n', ...
            area, tr, numTrials, s, sfm, hybrid);
    end
end

T = table(MaxArea, Trial, Seed, SFM_success, Hybrid_success);
writetable(T, 'runs/success_vs_area_seeds.csv');

uArea = unique(MaxArea);
SFM_rate_pct    = zeros(numel(uArea),1);
Hybrid_rate_pct = zeros(numel(uArea),1);
for k = 1:numel(uArea)
    m = MaxArea == uArea(k);
    SFM_rate_pct(k)    = 100*mean(SFM_success(m));
    Hybrid_rate_pct(k) = 100*mean(Hybrid_success(m));
end
S = table(uArea, SFM_rate_pct, Hybrid_rate_pct, ...
    'VariableNames', {'MaxArea','SFM_rate_pct','Hybrid_rate_pct'});
writetable(S, 'runs/success_vs_area_summary.csv');
save('runs/success_vs_area.mat');

disp('=== Success vs Maximum Obstacle Area ===');
disp(S);
