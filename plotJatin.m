trials = 50; 

% Maximum area vs success rate of AS and S (two graphs, x axis is max area)

maxAreaValues = 50:25:200;
succAS = zeros(size(maxAreaValues));
succS = zeros(size(maxAreaValues));
%succMatAS = zeros(trials, numel(maxAreaValues));
%succMatS = zeros(trials, numel(maxAreaValues));
for k = 1:numel(maxAreaValues)
    hitAS = 0; 
    hitS = 0; 
    for t=1:trials
        [outcomeAS, outcomeS] = runExperiment(maxAreaValues(k), 10);
        %succMatAS(t,k) = outcomeAS;
        %succMatS (t,k) = outcomeS;
        hitAS = hitAS + outcomeAS; 
        hitS = hitS + outcomeS;
    end 
    succAS(k) = hitAS/trials;
    succS(k) = hitS/trials;
end

%stdAS = std(succMatAS, 0, 1);
%stdS = std(succMatS, 0, 1);
figure;
plot(maxAreaValues, succAS, 'rs-', 'LineWidth', 2);
hold on;
plot(maxAreaValues, succS, 'bo-', 'LineWidth',2);
%errorbar(maxAreaValues, succAS, stdAS, 'r.', 'Capsize', 0, 'LineWidth', 1.5);
%errorbar(maxAreaValues, succS, stdS, 'b.', 'Capsize', 0, 'LineWidth', 1.5);
xlabel('Maximum Obstacle Area');
ylabel('Average Success Rate');
title('Average Success Rate vs. Maximum Obstacle Area');
grid on; 
legend('A* + SFM', 'SFM Only', 'Location', 'northeast');

%{
% 9. Number of polygon vs success rate of AS and S (two graphs, x axis is max area)
polyValues = 2:2:14;
succAS2 = zeros(size(polyValues));
succS2 = zeros(size(polyValues));

for k = 1:numel(polyValues)
    hitAS = 0; 
    hitS = 0; 
    for t=1:trials
        [outcomeAS, outcomeS] = runExperiment(150, polyValues(k));
        hitAS = hitAS + outcomeAS; 
        hitS = hitS + outcomeS;
    end 
    succAS2(k) = hitAS/trials;
    succS2(k) = hitS/trials;
end

figure;
plot(polyValues, succAS2, 'rs-', 'LineWidth', 2);
hold on;
plot(polyValues, succS2, 'bo-', 'LineWidth',2);
xlabel('Number of Obstacles');
ylabel('Average Success Rate');
title('Average Success Rate vs. Number of Obstacles');
grid on;
legend('A* + SFM', 'SFM Only', 'Location', 'northeast');

%}
