numTrials = 10;

dist_trials = zeros(numTrials, 2, 3000);
for i = 1:numTrials
    dist_trials(i,:,:) = Visual_Distance_v_Time();
end

dist_avg = zeros(2,3000);
dist_avg = sum(dist_trials, 1) / numTrials;


hold on
dist_avg = squeeze(dist_avg);
plot([1:3000], dist_avg(1,:), "Blue");
plot([1:3000], dist_avg(2,:), "Magenta");
legend("SFM + A*", "SFM only")
Xlabel("Timesteps")
Ylabel("Distance from Goal")