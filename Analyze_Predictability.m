% Analyze_Predictability.m
% Decides whether the "hybrid paths are more consistent/predictable" claim is
% supported by the 300-trial distance data. Uses successful-run path lengths.
%
% Consistency is measured by the coefficient of variation (CV = SD/mean):
% lower CV = more predictable path length. We test whether the hybrid's CV is
% LOWER than SFM-only's via:
%   (1) a bootstrap 95% CI on the CV ratio (hybrid/SFM)  -- primary, assumption-free
%   (2) Brown-Forsythe Levene test on |x - median|        -- secondary, classic
% Both are implemented with base MATLAB only (no Statistics Toolbox).

S = load('path_distance_results.mat');   % dist_astar_all, dist_sfm_all
h = S.dist_astar_all(:);   % hybrid successful path lengths
s = S.dist_sfm_all(:);     % SFM-only successful path lengths

nh = numel(h); ns = numel(s);
mh = mean(h); ms = mean(s);
sdh = std(h); sds = std(s);
cvh = sdh/mh; cvs = sds/ms;
ratio = cvh/cvs;

fprintf('\n=== Predictability (path-length consistency) ===\n');
fprintf('Hybrid:   n=%d mean=%.2f sd=%.2f CV=%.3f\n', nh, mh, sdh, cvh);
fprintf('SFM-only: n=%d mean=%.2f sd=%.2f CV=%.3f\n', ns, ms, sds, cvs);
fprintf('CV ratio (hybrid/SFM) = %.3f  (<1 means hybrid more consistent)\n', ratio);

% ---- (1) Bootstrap 95% CI on the CV ratio -------------------------------
rng(424242,'twister');
B = 20000;
ratios = zeros(B,1);
for b = 1:B
    hb = h(randi(nh, nh, 1));
    sb = s(randi(ns, ns, 1));
    ratios(b) = (std(hb)/mean(hb)) / (std(sb)/mean(sb));
end
ci = quantile(ratios, [0.025 0.975]);
fprintf('\nBootstrap 95%% CI on CV ratio: [%.3f, %.3f]  (B=%d)\n', ci(1), ci(2), B);
if ci(2) < 1
    fprintf('=> Hybrid SIGNIFICANTLY more consistent (CI entirely below 1).\n');
    verdict_boot = "hybrid_more_consistent";
elseif ci(1) > 1
    fprintf('=> Hybrid SIGNIFICANTLY LESS consistent (CI entirely above 1).\n');
    verdict_boot = "hybrid_less_consistent";
else
    fprintf('=> No significant difference in consistency (CI spans 1).\n');
    verdict_boot = "no_difference";
end

% ---- (2) Brown-Forsythe Levene test (median-centered) -------------------
zh = abs(h - median(h));
zs = abs(s - median(s));
zall = [zh; zs];
g1 = 1:nh; g2 = nh+(1:ns);
N = nh + ns;
grandZ = mean(zall);
% one-way ANOVA F on the absolute deviations (2 groups => df1=1, df2=N-2)
ssBetween = nh*(mean(zh)-grandZ)^2 + ns*(mean(zs)-grandZ)^2;
ssWithin  = sum((zh-mean(zh)).^2) + sum((zs-mean(zs)).^2);
df1 = 1; df2 = N - 2;
F = (ssBetween/df1) / (ssWithin/df2);
% p-value from F-distribution via regularized incomplete beta (base MATLAB)
pLevene = 1 - betainc(df1*F/(df1*F + df2), df1/2, df2/2);
fprintf('\nBrown-Forsythe Levene: F(%d,%d)=%.3f, p=%.4f\n', df1, df2, F, pLevene);
if pLevene < 0.05
    if sdh < sds
        fprintf('=> Variances differ (p<0.05) and hybrid SD is smaller: more consistent.\n');
    else
        fprintf('=> Variances differ (p<0.05) but hybrid SD is LARGER: less consistent.\n');
    end
else
    fprintf('=> No significant variance difference (p>=0.05).\n');
end

save('runs/predictability_analysis.mat', 'nh','ns','mh','ms','sdh','sds', ...
    'cvh','cvs','ratio','ci','F','pLevene','verdict_boot');
fprintf('\nSaved runs/predictability_analysis.mat\n');
