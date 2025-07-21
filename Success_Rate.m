numTrials = 50;

Success_Counts = [0,0];
for i = 1:numTrials
    Success_i = [0,0];
    [Success_i(1), Success_i(2)] = Success();
    Success_Counts = Success_Counts + Success_i;
end

Success_Percent = Success_Counts/numTrials;
x = ["SFM", "SFM + A*"];
bar(x,Success_Percent);
ylabel("Percentage of Trials That Reached the Goal");
title("Success Rate of SFM Only vs. A* + SFM");
