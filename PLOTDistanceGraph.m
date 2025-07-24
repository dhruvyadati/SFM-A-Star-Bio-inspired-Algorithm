function PLOTDistanceGraph

numberOfTrials = 50;

sfm_distances = [];
astar_sfm_distances = [];

for i = 1:numberOfTrials
    [dist_astarsfm, dist_sfm, SFM_A_success, SFM_success] = GetTotalDistanceVsNumberTrialsVariables;

    if SFM_success == 1
        sfm_distances(end+1) = dist_sfm;
    end
    if SFM_A_success == 1
        astar_sfm_distances(end+1) = dist_astarsfm;
    end

    fprintf('Trial %d Done\n', i);
end

% Display
fprintf('SFM Only - Mean: %.3f, Std: %.3f\n', avg_sfm, std_sfm);
fprintf('A* + SFM - Mean: %.3f, Std: %.3f\n', avg_astar_sfm, std_astar_sfm);

% Create bar plot with error bars
figure;
barData = [avg_sfm, avg_astar_sfm];
bar(barData);
hold on;

errorbar(1:2, barData, [std_sfm, std_astar_sfm], '.k', 'LineWidth', 1.5);

set(gca, 'XTickLabel', {'SFM Only', 'A* + SFM'});
%ylabel('Average Total Distance');
%title(sprintf('Average Total Distance over %d Trials (with Std Dev)', numberOfTrials));
grid on;

end
