classdef StatsLib
% StatsLib provides the statistical tests this project needs, implemented
% without the Statistics and Machine Learning Toolbox.
%
% Two reasons it exists. First, the toolbox is a paid add-on and the earlier
% analysis scripts silently depended on it (ranksum), so results could not be
% reproduced on a base MATLAB install. Everything here runs on base MATLAB.
%
% Second, three things the earlier analysis did not supply: an explicit METHOD
% for confidence intervals (previously plotted without one), EFFECT SIZES
% alongside p-values, and a correction for MULTIPLE COMPARISONS across the
% swept conditions. Each has a named method below.

methods (Static)

    % ---- Normal CDF (erfc-based; no toolbox) -------------------------
    function p = normcdf(z)
        p = 0.5 * erfc(-z ./ sqrt(2));
    end

    % ---- Wilson score interval for a proportion ----------------------
    % Preferred over the normal (Wald) approximation, which is badly behaved
    % near 0 and 1 -- exactly where these success rates live (98%, 10%).
    % Report this method by name in the figure captions.
    function [lo, hi] = wilsonCI(k, n, alpha)
        if nargin < 3, alpha = 0.05; end
        if n == 0, lo = NaN; hi = NaN; return; end
        z = StatsLib.zCrit(alpha);
        phat = k / n;
        denom = 1 + z^2 / n;
        centre = (phat + z^2 / (2*n)) / denom;
        half = z * sqrt(phat*(1-phat)/n + z^2/(4*n^2)) / denom;
        lo = max(0, centre - half);
        hi = min(1, centre + half);
    end

    function z = zCrit(alpha)
        % Inverse normal CDF at 1 - alpha/2, by bisection.
        target = 1 - alpha/2;
        lo = 0; hi = 10;
        for k = 1:200
            mid = (lo + hi) / 2;
            if StatsLib.normcdf(mid) < target, lo = mid; else, hi = mid; end
        end
        z = (lo + hi) / 2;
    end

    % ---- Wilcoxon signed-rank test (paired) --------------------------
    % Replaces the earlier unpaired rank-sum test. Both arms run on the SAME seeded
    % scenarios, so the observations are paired and a paired test is both
    % more appropriate and more powerful. Normal approximation with tie and
    % continuity corrections.
    function [p, z, effect] = signedRank(a, b)
        d = a(:) - b(:);
        d = d(d ~= 0);              % drop exact ties (Wilcoxon convention)
        n = numel(d);
        if n < 6
            p = NaN; z = NaN; effect = NaN; return;
        end
        [r, tieCorr] = StatsLib.tiedRank(abs(d));
        Wpos = sum(r(d > 0));
        mu = n * (n + 1) / 4;
        sigma = sqrt(n*(n+1)*(2*n+1)/24 - tieCorr/48);
        z = (Wpos - mu - 0.5 * sign(Wpos - mu)) / sigma;
        p = 2 * (1 - StatsLib.normcdf(abs(z)));
        % Matched-pairs rank-biserial correlation.
        Wneg = n*(n+1)/2 - Wpos;
        effect = (Wpos - Wneg) / (n*(n+1)/2);
    end

    % ---- Mann-Whitney U / rank-sum (unpaired) ------------------------
    function [p, z, effect] = rankSum(a, b)
        a = a(:); b = b(:);
        na = numel(a); nb = numel(b);
        if na < 3 || nb < 3, p = NaN; z = NaN; effect = NaN; return; end
        [r, tieCorr] = StatsLib.tiedRank([a; b]);
        Ra = sum(r(1:na));
        U = Ra - na*(na+1)/2;
        mu = na*nb/2;
        N = na + nb;
        sigma = sqrt(na*nb/12 * ((N + 1) - tieCorr/(N*(N-1))));
        z = (U - mu - 0.5*sign(U - mu)) / sigma;
        p = 2 * (1 - StatsLib.normcdf(abs(z)));
        effect = 2*U/(na*nb) - 1;    % Cliff's delta
    end

    function [r, tieCorr] = tiedRank(x)
        x = x(:);
        [s, ord] = sort(x);
        n = numel(x);
        r = zeros(n, 1);
        tieCorr = 0;
        i = 1;
        while i <= n
            j = i;
            while j < n && s(j+1) == s(i), j = j + 1; end
            r(i:j) = (i + j) / 2;
            t = j - i + 1;
            if t > 1, tieCorr = tieCorr + t^3 - t; end
            i = j + 1;
        end
        out = zeros(n, 1);
        out(ord) = r;
        r = out;
    end

    % ---- Exact McNemar test on paired binary outcomes ----------------
    % b = successes for arm A only, c = successes for arm B only.
    % Exact two-sided binomial test; no chi2cdf, no continuity fudge.
    function [p, oddsRatio] = mcnemarExact(b, c)
        n = b + c;
        if n == 0, p = 1; oddsRatio = NaN; return; end
        k = min(b, c);
        % Two-sided exact p = 2 * P(X <= k), X ~ Binomial(n, 0.5), capped at 1.
        cum = 0;
        for i = 0:k
            cum = cum + exp(gammaln(n+1) - gammaln(i+1) - gammaln(n-i+1) ...
                            + n * log(0.5));
        end
        p = min(1, 2 * cum);
        oddsRatio = b / max(c, eps);
    end

    % ---- Two-proportion z-test + Cohen's h ---------------------------
    function [p, z, h] = twoProportion(k1, n1, k2, n2)
        p1 = k1/n1; p2 = k2/n2;
        pPool = (k1 + k2) / (n1 + n2);
        se = sqrt(pPool * (1 - pPool) * (1/n1 + 1/n2));
        z = (p1 - p2) / se;
        p = 2 * (1 - StatsLib.normcdf(abs(z)));
        h = 2*asin(sqrt(p1)) - 2*asin(sqrt(p2));   % Cohen's h effect size
    end

    % ---- Bootstrap CI of the mean ------------------------------------
    % Percentile bootstrap. State this by name wherever a mean CI is plotted.
    function [lo, hi] = bootCI(x, alpha, nBoot, seed)
        if nargin < 2, alpha = 0.05; end
        if nargin < 3, nBoot = 10000; end
        if nargin < 4, seed = 12345; end
        x = x(~isnan(x));
        n = numel(x);
        if n < 2, lo = NaN; hi = NaN; return; end
        s = RandStream('twister', 'Seed', seed);
        means = zeros(nBoot, 1);
        for k = 1:nBoot
            idx = randi(s, n, n, 1);
            means(k) = mean(x(idx));
        end
        means = sort(means);
        lo = means(max(1, floor(alpha/2 * nBoot)));
        hi = means(min(nBoot, ceil((1 - alpha/2) * nBoot)));
    end

    % ---- Matched 2x2 interaction contrast ----------------------------
    % For binary outcomes measured on the SAME scenarios under four arms,
    % the estimand is
    %     delta = [p(G,T) - p(G,0)] - [p(0,T) - p(0,0)]
    % computed per scenario and averaged, so scenario difficulty cancels.
    % Bootstrap CI over scenarios plus a sign-flip permutation p-value, so
    % no asymptotic assumption is needed at small n.
    function [delta, lo, hi, pPerm] = interaction(sGT, sG0, s0T, s00, nBoot, seed)
        if nargin < 5, nBoot = 20000; end
        if nargin < 6, seed = 2468; end
        d = (double(sGT(:)) - double(sG0(:))) - (double(s0T(:)) - double(s00(:)));
        n = numel(d);
        delta = mean(d);

        st = RandStream('twister', 'Seed', seed);
        bs = zeros(nBoot, 1);
        for k = 1:nBoot
            bs(k) = mean(d(randi(st, n, n, 1)));
        end
        bs = sort(bs);
        lo = bs(max(1, floor(0.025 * nBoot)));
        hi = bs(min(nBoot, ceil(0.975 * nBoot)));

        % Under the null of no interaction the per-scenario contrast is
        % symmetric about zero, so sign flipping is the exact exchange.
        st2 = RandStream('twister', 'Seed', seed + 1);
        cnt = 0;
        for k = 1:nBoot
            fl = d .* (2 * (rand(st2, n, 1) > 0.5) - 1);
            if abs(mean(fl)) >= abs(delta) - 1e-12, cnt = cnt + 1; end
        end
        pPerm = (cnt + 1) / (nBoot + 1);
    end

    % ---- Holm-Bonferroni step-down correction ------------------------
    % Applied across the swept conditions (obstacle count, area, vertices)
    % so that the family-wise error rate is controlled, as requested.
    function padj = holm(p)
        p = p(:);
        m = numel(p);
        [ps, ord] = sort(p);
        adj = zeros(m, 1);
        running = 0;
        for i = 1:m
            val = (m - i + 1) * ps(i);
            running = max(running, val);
            adj(i) = min(1, running);
        end
        padj = zeros(m, 1);
        padj(ord) = adj;
    end

end
end
