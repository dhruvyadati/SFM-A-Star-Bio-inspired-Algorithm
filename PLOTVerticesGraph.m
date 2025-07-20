function PLOTVerticesGraph

numVerticesMax = 10;
numberOfTrials = 5;

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
    end

    sfm_success_rates(idx) = sfm_success_total / numberOfTrials;
    astar_sfm_success_rates(idx) = astar_sfm_success_total / numberOfTrials;
end

% Plotting
figure;
hold on;
plot(vertexCounts, sfm_success_rates, '-o', 'LineWidth', 2, 'DisplayName', 'SFM Only');
plot(vertexCounts, astar_sfm_success_rates, '-x', 'LineWidth', 2, 'DisplayName', 'A* + SFM');
xlabel('Number of Vertices per Obstacle');
ylabel('Average Success Rate');
title('Success Rate vs. Number of Obstacle Vertices');
legend('Location', 'southwest');
grid on;
