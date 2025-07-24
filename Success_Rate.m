numTrials = 100;

Success_Counts = [0,0];
for i = 1:numTrials
    Success_i = [0,0];
    [Success_i(1), Success_i(2)] = Success();
    Success_Counts(i,:) = Success_i;
end

Success_Percent = sum(Success_Counts,1)/numTrials;
x = ["SFM", "SFM + A*"];

p_hat = Success_Percent;
for i = 1:2
    SE(i) = sqrt(p_hat(i) * (1-p_hat(i))/numTrials);
end
ME = 1.96 * SE;


hold on
bar(x,Success_Percent);
h = errorbar([1 2],Success_Percent, ME,'o');
h.LineStyle = "none";


ylabel("Percentage of Trials That Reached the Goal");
title("Success Rate of SFM Only vs. A* + SFM");
