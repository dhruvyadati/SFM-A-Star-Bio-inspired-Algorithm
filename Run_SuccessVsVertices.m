% Run_SuccessVsVertices.m
% Sweeps the number of vertices per obstacle and records SFM-only vs hybrid
% (A*+SFM) success rates. Fully reproducible: a single master seed derives a
% per-trial seed, and every seed is logged to runs/.

if ~exist('vertexCounts','var'); vertexCounts = 3:20; end   % yadat5: triangles -> 20-gons
if ~exist('numTrials','var');    numTrials    = 30;  end    % trials per value
if ~exist('MASTER_SEED','var');  MASTER_SEED  = 20260616; end

if ~exist('runs','dir'); mkdir('runs'); end

rng(MASTER_SEED,'twister');
nRows    = numel(vertexCounts) * numTrials;
allSeeds = randi([1, 2^31-1], nRows, 1);

NumVertices    = zeros(nRows,1);
Trial          = zeros(nRows,1);
Seed           = zeros(nRows,1);
SFM_success    = zeros(nRows,1);
Hybrid_success = zeros(nRows,1);

row = 0;
for vi = 1:numel(vertexCounts)
    nv = vertexCounts(vi);
    for tr = 1:numTrials
        row = row + 1;
        s = allSeeds(row);
        rng(s,'twister');
        % NOTE: returns [hybrid, sfm] in that order
        [hybrid, sfm] = VertexSuccessParam(nv);
        NumVertices(row)    = nv;
        Trial(row)          = tr;
        Seed(row)           = s;
        SFM_success(row)    = sfm;
        Hybrid_success(row) = hybrid;
        fprintf('verts=%2d trial=%2d/%d seed=%d sfm=%d hybrid=%d\n', ...
            nv, tr, numTrials, s, sfm, hybrid);
    end
end

T = table(NumVertices, Trial, Seed, SFM_success, Hybrid_success);
writetable(T, 'runs/success_vs_vertices_seeds.csv');

uV = unique(NumVertices);
SFM_rate_pct    = zeros(numel(uV),1);
Hybrid_rate_pct = zeros(numel(uV),1);
for k = 1:numel(uV)
    m = NumVertices == uV(k);
    SFM_rate_pct(k)    = 100*mean(SFM_success(m));
    Hybrid_rate_pct(k) = 100*mean(Hybrid_success(m));
end
S = table(uV, SFM_rate_pct, Hybrid_rate_pct, ...
    'VariableNames', {'NumVertices','SFM_rate_pct','Hybrid_rate_pct'});
writetable(S, 'runs/success_vs_vertices_summary.csv');
save('runs/success_vs_vertices.mat');

disp('=== Success vs Number of Vertices ===');
disp(S);
