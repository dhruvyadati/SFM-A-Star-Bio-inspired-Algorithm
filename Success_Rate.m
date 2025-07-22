numTrials = 100;

Success_Counts = [0,0];
for i = 1:numTrials
    Success_i = [0,0];
    [Success_i(1), Success_i(2)] = Success();
    Success_Counts(i,:) = Success_i;
end

Success_Percent = sum(Success_Counts,1)/numTrials;
x = ["SFM", "SFM + A*"];

Standard_Deviation = std(Success_Counts);


hold on
bar(x,Success_Percent);
er = errorbar([1:2], Success_Percent, Success_Percent - Standard_Deviation, Success_Percent + Standard_Deviation);
er.Color = [0 0 0];
er.LineStyle = 'none';
ylabel("Percentage of Trials That Reached the Goal");
title("Success Rate of SFM Only vs. A* + SFM");
