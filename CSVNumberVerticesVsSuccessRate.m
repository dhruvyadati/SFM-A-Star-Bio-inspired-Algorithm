function CSVNumberVerticesVsSuccessRate

%% Save Total Distance vs # Trials Graph

%fprintf('A*+SFM: %.2f\n', dist_astarsfm);
%fprintf('SFM-only: %.2f\n', dist_sfm);

% Create a table from the variables

numberOfTrials = 2;
numVertices = 4;

T = table(); % Initialize empty table
for i = 3:numVertices
    for j = 1:numberOfTrials
        [SFM_A_success, SFM_success] = GetNumberVerticesVsSuccessRate(i);
        newRow = table(SFM_success, SFM_A_success, i,'VariableNames', {'SFM Success', 'AStarSFM Success', 'Number of Vertices'});
        T = [T; newRow]; % Append each row
    end
end

writetable(T, 'VerticesData.csv');