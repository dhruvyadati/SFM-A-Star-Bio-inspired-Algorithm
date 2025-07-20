[dist_astarsfm, dist_sfm] = VisualDhruv;

%% Save Total Distance vs # Trials Graph

%fprintf('A*+SFM: %.2f\n', dist_astarsfm);
%fprintf('SFM-only: %.2f\n', dist_sfm);

% Create a table from the variables

T = table(); % Initialize empty table
for i = 1:2
    [dist_astarsfm, dist_sfm] = VisualDhruv;
    newRow = table(dist_sfm, dist_astarsfm,'VariableNames', {'SFM', 'AStarSFM'});
    T = [T; newRow]; % Append each row
end

writetable(T, 'PathLengthData.csv');


