function TotalDistanceVsNumberTrials

[dist_astarsfm, dist_sfm, SFM_A_success, SFM_success] = VisualDhruv;

%% Save Total Distance vs # Trials Graph

%fprintf('A*+SFM: %.2f\n', dist_astarsfm);
%fprintf('SFM-only: %.2f\n', dist_sfm);

% Create a table from the variables

numberOfTrials = 2;

T = table(); % Initialize empty table
for i = 1:numberOfTrials
    [dist_astarsfm, dist_sfm, SFM_A_success, SFM_success] = VisualDhruv;
    if (SFM_A_success == 0)
        dist_astarsfm = 0;
    end
    if (SFM_success == 0)
        dist_sfm = 0;
    end
    newRow = table(dist_sfm, dist_astarsfm,'VariableNames', {'SFM', 'AStarSFM'});
    T = [T; newRow]; % Append each row
end

writetable(T, 'PathLengthData.csv');


