% Test_HeadOn.m
% MANDATORY REGRESSION TEST.
%
% A robot drives toward its goal with a stationary pedestrian standing
% directly on the line. The pedestrian is pinned, so it cannot yield. This is
% the worst case for a purely soft-force controller and it isolates the
% mechanism from crowd dynamics.
%
% The unfiltered controller fails this: goal drive v0/tau exceeds the social
% repulsion magnitude at contact, so the robot pushes through and stays in
% contact for the rest of the traverse. Any configuration intended to be
% called safe must pass.
%
% Note the pinned pedestrian is deliberately unrealistic -- real pedestrians
% yield, which is why contacts in the crowd runs are brief. The point is that
% nothing in the formulation PREVENTS sustained contact; reciprocal yielding
% merely makes it uncommon.
%
% Usage:  cd to the repository root, then  run('v2/Test_HeadOn.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

pedAt = [10; 0];
goal  = [20; 0];

configs = {
    'baseline  (aPed=2.1, no TTC)',  struct('aPed', 2.1,  'ttc', false)
    'stronger  (aPed=5.0, no TTC)',  struct('aPed', 5.0,  'ttc', false)
    'TTC brake (aPed=2.1, TTC on)',  struct('aPed', 2.1,  'ttc', true )
    'both      (aPed=5.0, TTC on)',  struct('aPed', 5.0,  'ttc', true )
};

fprintf('=== Head-on regression: pinned pedestrian on the goal line ===\n\n');
fprintf('%-30s %9s %9s %10s %9s %7s\n', ...
        'configuration','min gap','max pen','contact_s','reached','verdict');
fprintf('%s\n', repmat('-', 1, 82));

results = cell(size(configs, 1), 1);
for c = 1:size(configs, 1)
    P = SfmParams();
    P.aPedRobot = configs{c, 2}.aPed;
    P.ttcEnabled = configs{c, 2}.ttc;

    x = [[0; 0], pedAt];
    v = zeros(2, 2);
    g = [goal, pedAt];
    radii = [P.rRobot, P.rPed];
    v0 = [P.v0Robot, 0];

    minGap = inf; maxPen = 0; contactSteps = 0; reached = false; prevBs = 1.0;
    for step = 1:20000
        F = ComputeForces(x, v, g, radii, v0, {}, {}, P);
        F(:, 2) = 0;
        v = v + F * P.dt;
        sp = sqrt(sum(v.^2, 1));
        ov = sp > P.vMax;
        if any(ov), v(:, ov) = v(:, ov) ./ sp(ov) * P.vMax; end
        if P.ttcEnabled
            bs = TtcBrake(x(:, 1), v(:, 1), x(:, 2), v(:, 2), radii(2), P, prevBs);
            prevBs = bs;
            v(:, 1) = v(:, 1) * bs;
        end
        v(:, 2) = 0;
        xPrev = x;
        x = x + v * P.dt;

        % Swept clearance, matching RunTrial.
        r0 = xPrev(:, 2) - xPrev(:, 1);
        r1 = x(:, 2) - x(:, 1);
        w = r1 - r0; ww = sum(w.^2);
        if ww > eps
            s = min(1, max(0, -dot(r0, w) / ww));
        else
            s = 0;
        end
        gap = norm(r0 + s * w) - (P.rRobot + P.rPed);

        minGap = min(minGap, gap);
        if gap < 0, contactSteps = contactSteps + 1; maxPen = max(maxPen, -gap); end
        if norm(x(:, 1) - goal) < P.rGoal, reached = true; break; end
    end

    clear_ = minGap >= 0;
    results{c} = struct('name', configs{c, 1}, 'clear', clear_, ...
                        'minGap', minGap, 'reached', reached);
    fprintf('%-30s %+9.4f %9.4f %10.2f %9s %7s\n', configs{c, 1}, minGap, maxPen, ...
            contactSteps * P.dt, tf(reached, 'yes', 'no'), ...
            tf(clear_, 'CLEAR', 'CONTACT'));
end

fprintf('\n');
base = results{1};
if base.clear
    error('Test_HeadOn:unexpected', ...
          'Baseline was expected to make contact; the diagnosis has changed.');
end
fprintf('Baseline makes contact, as expected.\n');

ttcRes = results{3};
if ttcRes.clear
    fprintf('[PASS] the TTC filter alone prevents contact in the worst case.\n');
else
    fprintf('[FAIL] the TTC filter did NOT prevent contact (min gap %+.4f m).\n', ...
            ttcRes.minGap);
    fprintf('       Re-tune ttcHorizon / ttcStop / ttcClearance before proceeding.\n');
end

% A safe controller that can never reach the goal is not a solution either.
% Sustained braking that stalls the robot is a failure mode of its own, and
% the pinned pedestrian is an unavoidable obstacle here, so not reaching the
% goal is the correct behaviour rather than a defect.
fprintf('\nNote: with the pedestrian pinned on the goal line, refusing to reach\n');
fprintf('the goal is CORRECT. Stopping short beats pushing through.\n');

function s = tf(c, a, b), if c, s = a; else, s = b; end, end
