function PLOTVerticesGraph

%rng(123, "twister");

numVerticesMax = 20;
numberOfTrials = 30;

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

sfm_success_rates = mean(sfm_success_trials, 1);
astar_sfm_success_rates = mean(astar_sfm_success_trials, 1);

% Plotting
figure;
hold on;

sfm_std = std(sfm_success_trials, 0, 1);
astar_sfm_std = std(astar_sfm_success_trials, 0, 1);

sfm_CI = 1.96 * sfm_std / sqrt(numberOfTrials);
astar_sfm_CI = 1.96 * astar_sfm_std / sqrt(numberOfTrials);

x_plot = [vertexCounts, fliplr(vertexCounts)];
sfm_y_fill = min(max([sfm_success_rates - sfm_CI, fliplr(sfm_success_rates + sfm_CI)], 0), 1);
astar_y_fill = min(max([astar_sfm_success_rates - astar_sfm_CI, fliplr(astar_sfm_success_rates + astar_sfm_CI)], 0), 1);

% Plot shaded confidence intervals
fill(x_plot, astar_y_fill, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
fill(x_plot, sfm_y_fill, 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

plot(vertexCounts, astar_sfm_success_rates, 'rs-', 'LineWidth', 2, 'DisplayName', 'A* + SFM');
plot(vertexCounts, sfm_success_rates, 'bo-', 'LineWidth', 2, 'DisplayName', 'SFM Only');
xlabel('Number of Vertices per Obstacle');
ylabel('Average Success Rate');
title('Average Success Rate vs. Number of Vertices per Obstacle');
legend('Location', 'northeast');
grid on;

end
