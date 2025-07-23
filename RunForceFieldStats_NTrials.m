function RunForceFieldStats_NTrials()
    % Runs the SFM Force Field simulation N times and averages metrics.
    
    N = 50;

    % Store metrics
    trapRatio_all = zeros(1, N);
    totalForce_all = zeros(1, N);
    normForce_all = zeros(1, N);
    avgGrad_all = zeros(1, N);
    maxGrad_all = zeros(1, N);
    numBasins_all = zeros(1, N);
    meanBasinSize_all = zeros(1, N);
    maxBasinSize_all = zeros(1, N);

    for i = 1:N
        fprintf("Running trial %d of %d...\n", i, N);

        % Call your modified plotting function that returns metrics
        stats = PlotNetForceFromAgentPerspective();

        % Accumulate metrics
        trapRatio_all(i)     = stats.trapRatio;
        totalForce_all(i)    = stats.totalForceAlongPath;
        normForce_all(i)     = stats.normForceCost;
        avgGrad_all(i)       = stats.avgGradient;
        maxGrad_all(i)       = stats.maxGradient;
        numBasins_all(i)     = stats.numBasins;
        meanBasinSize_all(i) = stats.meanBasinSize;
        maxBasinSize_all(i)  = stats.maxBasinSize;

        close all; % Optional: close figures each trial to avoid clutter
    end

    %% Average and STD results
    results = struct(...
        'trapRatio_avg', mean(trapRatio_all), ...
        'trapRatio_std', std(trapRatio_all), ...
        'totalForce_avg', mean(totalForce_all), ...
        'totalForce_std', std(totalForce_all), ...
        'normForce_avg', mean(normForce_all), ...
        'normForce_std', std(normForce_all), ...
        'avgGrad_avg', mean(avgGrad_all), ...
        'avgGrad_std', std(avgGrad_all), ...
        'maxGrad_avg', mean(maxGrad_all), ...
        'maxGrad_std', std(maxGrad_all), ...
        'numBasins_avg', mean(numBasins_all), ...
        'numBasins_std', std(numBasins_all), ...
        'meanBasinSize_avg', mean(meanBasinSize_all), ...
        'meanBasinSize_std', std(meanBasinSize_all), ...
        'maxBasinSize_avg', mean(maxBasinSize_all), ...
        'maxBasinSize_std', std(maxBasinSize_all) ...
    );

    disp('Averaged Results over Trials');
    disp(results);
end
