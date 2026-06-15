% Make_Figures.m
% Regenerates the statistical figures from the seeded re-run data, using the
% paper's figure-file numbering (yadat3 = success bar, generated separately):
%   yadat4 (success vs vertices), yadat5 (success vs obstacles),
%   yadat6 (success vs area), yadat7 (distance vs timestep),
%   yadat8 (total path distance bar).
% yadat9/yadat10 are single-environment method illustrations and are not
% part of the statistical re-run.
% Style: blue = hybrid (A*+SFM), orange = SFM-only, matching yadat3/yadat4.

outdir = 'runs/figures';
if ~exist(outdir,'dir'); mkdir(outdir); end
% Match yadat3.png: MATLAB default blue for the hybrid; MATLAB default
% orange for the SFM-only line (line plots need two distinguishable colors).
BLUE = [0 0.4470 0.7410]; MAG = [0.8500 0.3250 0.0980];

% ---- Helper: success-rate-vs-X sweep plot with 95% CI bands ------------
function sweepFig(csvFile, xcol, xlabelStr, outPng, BLUE, MAG)
    T = readtable(csvFile);
    x = unique(T.(xcol));
    hybRate = zeros(numel(x),1); sfmRate = zeros(numel(x),1);
    hybCI = zeros(numel(x),1);  sfmCI = zeros(numel(x),1);
    for k = 1:numel(x)
        m = T.(xcol) == x(k); n = nnz(m);
        ph = mean(T.Hybrid_success(m)); ps = mean(T.SFM_success(m));
        hybRate(k) = 100*ph; sfmRate(k) = 100*ps;
        hybCI(k) = 100*1.96*sqrt(max(ph*(1-ph),0)/n);
        sfmCI(k) = 100*1.96*sqrt(max(ps*(1-ps),0)/n);
    end
    fig = figure('Visible','off','Position',[100 100 900 650]); hold on; box on;
    xf = [x; flipud(x)];
    fill(xf, [hybRate-hybCI; flipud(hybRate+hybCI)], BLUE, 'FaceAlpha',0.15,'EdgeColor','none');
    fill(xf, [sfmRate-sfmCI; flipud(sfmRate+sfmCI)], MAG,  'FaceAlpha',0.15,'EdgeColor','none');
    p1 = plot(x, hybRate, '-o', 'Color',BLUE,'LineWidth',2,'MarkerFaceColor',BLUE,'MarkerSize',5);
    p2 = plot(x, sfmRate, '-s', 'Color',MAG, 'LineWidth',2,'MarkerFaceColor',MAG, 'MarkerSize',5);
    xlabel(xlabelStr); ylabel('Success Rate (%)');
    ylim([-2 105]); grid on;
    legend([p1 p2], {'A* + SFM','SFM only'}, 'Location','best');
    set(gca,'FontSize',13);
    exportgraphics(fig, outPng, 'Resolution',200);
    close(fig);
    fprintf('wrote %s\n', outPng);
end

% ---- yadat6: success vs obstacles -------------------------------------
sweepFig('runs/success_vs_obstacles_seeds.csv','NumObstacles', ...
    'Number of Obstacles', fullfile(outdir,'yadat5.png'), BLUE, MAG);

% ---- yadat7: success vs area ------------------------------------------
sweepFig('runs/success_vs_area_seeds.csv','MaxArea', ...
    'Maximum Obstacle Area (sq. nodes)', fullfile(outdir,'yadat6.png'), BLUE, MAG);

% ---- yadat5: success vs vertices --------------------------------------
sweepFig('runs/success_vs_vertices_seeds.csv','NumVertices', ...
    'Number of Vertices per Obstacle', fullfile(outdir,'yadat4.png'), BLUE, MAG);

% ---- yadat8: distance from goal vs timestep ---------------------------
D = load('runs/time_vs_distance.mat');     % dist_avg (2x3000), CI (2x3000)
dist_avg = D.dist_avg; CI = D.CI;
tt = 1:size(dist_avg,2);
fig = figure('Visible','off','Position',[100 100 900 650]); hold on; box on;
xf = [tt, fliplr(tt)];
fill(xf, [dist_avg(1,:)-CI(1,:), fliplr(dist_avg(1,:)+CI(1,:))], BLUE,'FaceAlpha',0.2,'EdgeColor','none');
fill(xf, [dist_avg(2,:)-CI(2,:), fliplr(dist_avg(2,:)+CI(2,:))], MAG, 'FaceAlpha',0.2,'EdgeColor','none');
p1 = plot(tt, dist_avg(1,:), 'Color',BLUE,'LineWidth',2);
p2 = plot(tt, dist_avg(2,:), 'Color',MAG, 'LineWidth',2);
xlabel('Timesteps'); ylabel('Distance from Goal');
legend([p1 p2], {'A* + SFM','SFM only'}, 'Location','best');
grid on; set(gca,'FontSize',13);
exportgraphics(fig, fullfile(outdir,'yadat7.png'), 'Resolution',200);
close(fig); fprintf('wrote yadat7.png\n');

% ---- yadat9: total path distance bar ----------------------------------
S = load('path_distance_results.mat');     % dist_astar_all, dist_sfm_all
h = S.dist_astar_all(:); s = S.dist_sfm_all(:);
means = [mean(s), mean(h)];
sems  = [std(s)/sqrt(numel(s)), std(h)/sqrt(numel(h))];
ci95  = 1.96*sems;
fig = figure('Visible','off','Position',[100 100 800 650]); hold on; box on;
b = bar(categorical({'SFM only','A* + SFM'},{'SFM only','A* + SFM'}), means, 0.6);
% Match yadat3.png: both bars MATLAB default blue, black error bars
b.FaceColor = 'flat'; b.CData(1,:) = BLUE; b.CData(2,:) = BLUE;
errorbar(1:2, means, ci95, 'k','LineStyle','none','LineWidth',1.5,'CapSize',12);
ylabel('Average Total Path Distance');
set(gca,'FontSize',13); grid on;
text(1, means(1)+ci95(1)+4, sprintf('%.1f', means(1)),'HorizontalAlignment','center','FontSize',12);
text(2, means(2)+ci95(2)+4, sprintf('%.1f', means(2)),'HorizontalAlignment','center','FontSize',12);
exportgraphics(fig, fullfile(outdir,'yadat8.png'), 'Resolution',200);
close(fig); fprintf('wrote yadat8.png\n');

fprintf('ALL_FIGURES_DONE\n');
