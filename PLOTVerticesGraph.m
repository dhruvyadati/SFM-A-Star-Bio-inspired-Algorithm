function PLOTVerticesGraph

%rng(123, "twister");

numVerticesMax = 10;
numberOfTrials = 10;

vertexCounts = 3:numVerticesMax;
sfm_success_rates = zeros(size(vertexCounts));
astar_sfm_success_rates = zeros(size(vertexCounts));

for idx = 1:length(vertexCounts)
    vertexCount = vertexCounts(idx);

    sfm_success_total = 0;
    astar_sfm_success_total = 0;

    for trial = 1:numberOfTrials
        [SFM_A_success, SFM_success] = GetNumberVerticesVsSuccessRate(vertexCount);
        sfm_success_total = sfm_success_total + SFM_success;
        astar_sfm_success_total = astar_sfm_success_total + SFM_A_success;
        
        fprintf('Trial %d done for vertex count index %d (actual: %d vertices)\n', trial, idx, vertexCount);
    end

    sfm_success_rates(idx) = sfm_success_total / numberOfTrials;
    astar_sfm_success_rates(idx) = astar_sfm_success_total / numberOfTrials;
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