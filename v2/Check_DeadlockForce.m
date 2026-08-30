% Check_DeadlockForce.m
% Is the force at a deadlock actually small?
%
% The grid-interpolated field suggested |F| ~ 3-6 m/s^2 at deadlock points,
% which is self-contradictory: a robot at rest under 3 m/s^2 exceeds the
% 0.05 m/s stall threshold within two timesteps and could not remain stalled
% for 500. Either the deadlock is not a force equilibrium, or the field is
% under-resolved -- the grid was 0.5 m while the obstacle decay length is
% 0.2 m.
%
% This evaluates the force EXACTLY at the recorded position with the full
% simulator, alongside grid interpolations at several resolutions, so the two
% explanations can be told apart.
%
% Usage:  cd to the repository root, then  run('v2/Check_DeadlockForce.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

L = load(fullfile('runs','v2','safety_gate.mat'));
T0 = L.T; P = L.P;
sub  = T0(strcmp(T0.method, 'sfm'), :);
dead = sub(strcmp(sub.outcome, 'deadlock'), :);
n = min(6, height(dead));

fprintf('=== Force at recorded deadlock positions ===\n\n');
fprintf('%-12s %8s %9s %9s %9s %9s %8s\n', 'seed', 'speed', 'EXACT|F|', ...
        'grid0.50', 'grid0.25', 'grid0.10', 'obsGap');
fprintf('%s\n', repmat('-', 1, 74));

for c = 1:n
    seed = dead.seed(c);
    S = MakeScenario(seed, P);
    R = RunTrial(S, 'sfm', struct('logTrajectory', true));
    if ~strcmp(R.outcome, 'deadlock'), continue; end
    dp = R.finalRobotPos;

    % EXACT: rebuild the full agent set at the terminal state and read off
    % the robot's own net force from the simulator itself.
    xa = [dp, R.pedPos];
    va = [R.finalRobotVel, R.pedVel];
    ga = [P.goal(:), R.pedGoal];
    ra = [P.rRobot, repmat(P.rPed, 1, size(R.pedPos, 2))];
    ua = [P.v0Robot, repmat(P.v0Ped, 1, size(R.pedPos, 2))];
    Fa = ComputeForces(xa, va, ga, ra, ua, S.xObsList, S.yObsList, P);
    exactF = norm(Fa(:, 1));

    gv = zeros(1, 3); hs = [0.5 0.25 0.10];
    for k = 1:3
        Fg = TrapField(S, P, hs(k), R.pedPos, R.pedVel);
        gv(k) = interp2(Fg.X, Fg.Y, Fg.Z, dp(1), dp(2), 'linear', NaN);
    end

    og = inf;
    for kk = 1:numel(S.xObsList)
        og = min(og, p_poly_dist(dp(1), dp(2), S.xObsList{kk}, S.yObsList{kk}));
    end

    fprintf('%-12d %8.4f %9.4f %9.4f %9.4f %9.4f %8.3f\n', ...
            seed, norm(R.finalRobotVel), exactF, gv(1), gv(2), gv(3), og - P.rRobot);
end

fprintf(['\nIf EXACT is small while the coarse grids are large, the field was\n' ...
         'under-resolved and the earlier verdict is void. If EXACT is large,\n' ...
         'the deadlock is not a force equilibrium and the trap premise itself\n' ...
         'does not hold under the corrected model.\n']);
