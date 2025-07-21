function PLOTDistanceGraph

numberOfTrials = 5;

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

% Compute averages
avg_sfm = mean(sfm_distances);
avg_astar_sfm = mean(astar_sfm_distances);

% Create bar plot
figure;
bar([avg_sfm, avg_astar_sfm]);
set(gca, 'XTickLabel', {'SFM Only', 'A* + SFM'});
ylabel('Average Total Distance');
title(sprintf('Average Total Distance over %d Trials For Each Model', numberOfTrials));
grid on;
