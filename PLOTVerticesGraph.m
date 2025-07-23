function PLOTVerticesGraph

%rng(123, "twister");

numVerticesMax = 20;
numberOfTrials = 15;

vertexCounts = 3:numVerticesMax;
sfm_success_trials = zeros(numberOfTrials, length(vertexCounts));
astar_sfm_success_trials = zeros(numberOfTrials, length(vertexCounts));

for idx = 1:length(vertexCounts)
    vertexCount = vertexCounts(idx);

    for trial = 1:numberOfTrials
        [SFM_A_success, SFM_success] = GetNumberVerticesVsSuccessRate(vertexCount);
        sfm_success_trials(trial, idx) = SFM_success;
        astar_sfm_success_trials(trial, idx) = SFM_A_success;
        fprintf('Trial %d done for vertex count index %d (actual: %d vertices)\n', trial, idx, vertexCount);
    end
end

% Plotting
figure;
hold on;
errorbar(vertexCounts, astar_sfm_success_rates, astar_sfm_std, 'rs-', 'LineWidth', 2, 'DisplayName', 'A* + SFM');
errorbar(vertexCounts, sfm_success_rates, sfm_std, 'bo-', 'LineWidth', 2, 'DisplayName', 'SFM Only');
%xlabel('Number of Vertices per Obstacle');
%ylabel('Average Success Rate');
%title('Average Success Rate vs. Number of Vertices per Obstacle');
%legend('Location', 'northeast');
grid on;
xlim([min(vertexCounts) max(vertexCounts)]);
xticks(vertexCounts);

end
