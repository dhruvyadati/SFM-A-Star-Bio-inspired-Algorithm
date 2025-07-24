numTrials = 100;

dist_trials = zeros(numTrials, 2, 3000);
for i = 1:numTrials
    dist_trials(i,:,:) = Visual_Distance_v_Time();
end

dist_avg = zeros(2,3000);
dist_avg = sum(dist_trials, 1) / numTrials;
dist_avg = squeeze(dist_avg);

STD = std(dist_trials,0,1);
CI = 1.96 * STD/sqrt(numTrials);

CI = squeeze(CI);

x_plot = [1:3000, fliplr(1:3000)];

y_plot_A = [dist_avg(1,:) - CI(1,:),fliplr(dist_avg(1,:) + CI(1,:))];
y_plot_SFM = [dist_avg(2,:) - CI(2,:),fliplr(dist_avg(2,:) + CI(2,:))];



hold on
plot([1:3000], dist_avg(1,:), "Blue");
plot([1:3000], dist_avg(2,:), "Magenta");
fill(x_plot, y_plot_A, "Blue", "FaceAlpha", 0.3);
fill(x_plot, y_plot_SFM, "Magenta", "FaceAlpha", 0.3);
legend("SFM + A*", "SFM only")
xlabel("Timesteps")
ylabel("Distance from Goal")