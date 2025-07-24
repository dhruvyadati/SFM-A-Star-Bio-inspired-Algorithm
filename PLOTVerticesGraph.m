function PLOTVerticesGraph

%rng(123, "twister");

numVerticesMax = 5;
numberOfTrials = 2;

vertexCounts = 3:numVerticesMax;
sfm_success_rates = zeros(size(vertexCounts));
astar_sfm_success_rates = zeros(size(vertexCounts));
sfm_success_trials = zeros(numberOfTrials, length(vertexCounts));
astar_sfm_success_trials = zeros(numberOfTrials, leAngth(vertexCounts));

for i = 1:length(vertexCounts)
    vertexCount = vertexCounts(i);

    for trial = 1:numberOfTrials
        [SFM_A_success, SFM_success] = GetNumberVerticesVsSuccessRate(vertexCount);
        sfm_success_trials(trial, i) = SFM_success;
        astar_sfm_success_trials(trial, i) = SFM_A_success;

        
        fprintf('Trial %d done for vertex count index %d (actual: %d vertices)\n', trial, i, vertexCount);
    end

    sfm_success_rates = mean(sfm_success_trials, 1);
    astar_sfm_success_rates = mean(astar_sfm_success_trials, 1);
end

% Plotting
figure;
hold on;

plot(vertexCounts, astar_sfm_success_rates, 'rs-', 'LineWidth', 2, 'DisplayName', 'A* + SFM');
plot(vertexCounts, sfm_success_rates, 'bo-', 'LineWidth', 2, 'DisplayName', 'SFM Only');
xlabel('Number of Vertices per Obstacle');
ylabel('Average Success Rate');
title('Average Success Rate vs. Number of Vertices per Obstacle');
legend('Location', 'northeast');
grid on;