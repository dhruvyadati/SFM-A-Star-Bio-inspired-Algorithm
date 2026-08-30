% Diagnose_Contact.m
% Why does the robot overlap pedestrians, and is the recorded overlap a real
% dynamical effect or a measurement artifact?
%
% Three candidate explanations are tested:
%   A. Static force balance -- goal drive exceeds repulsion at contact.
%   B. Dynamic overshoot    -- contact stiffness too soft to absorb the
%                              robot's momentum within its body radius.
%   C. Discretisation       -- the timestep lets the pair jump past contact.
%
% Usage:  cd to the repository root, then  run('v2/Diagnose_Contact.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();
rSum = P.rRobot + P.rPed;

fprintf('=== Contact diagnosis ===\n\n');

%% ---- A. Static force balance ----------------------------------------
goalDrive = P.v0Robot / P.tau;          % max magnitude of (v_des*e - v)/tau
repAtContact = P.aPed;                  % A*exp(0) at d = rSum
fprintf('--- A. Static balance at the moment of contact ---\n');
fprintf('  max goal drive        : %6.2f m/s^2  (v0/tau)\n', goalDrive);
fprintf('  social rep at contact : %6.2f m/s^2  (aPed)\n', repAtContact);
if goalDrive > repAtContact
    fprintf('  => goal drive EXCEEDS social repulsion at contact by %.2f m/s^2.\n', ...
            goalDrive - repAtContact);
    fprintf('     Social repulsion alone cannot hold the robot off a pedestrian.\n');
else
    fprintf('  => social repulsion dominates; static balance is not the cause.\n');
end

% Equilibrium overlap where total repulsion balances the goal drive.
f = @(o) P.aPed*exp(o/P.bPed) + P.kBody*o - goalDrive;
lo = 0; hi = rSum;
for k = 1:200
    mid = (lo+hi)/2;
    if f(mid) < 0, lo = mid; else, hi = mid; end
end
oStatic = (lo+hi)/2;
fprintf('  static equilibrium overlap : %.4f m (%.1f mm)\n\n', oStatic, 1000*oStatic);

%% ---- B. Dynamic overshoot -------------------------------------------
% A robot arriving at speed v compresses the contact spring. Equating
% kinetic energy to spring energy, 0.5 v^2 = 0.5 k o^2, gives o = v/sqrt(k).
fprintf('--- B. Dynamic overshoot ---\n');
for v = [0.5 1.0 P.v0Robot P.vMax]
    oDyn = v / sqrt(P.kBody);
    fprintf('  approach at %4.2f m/s -> predicted overlap %.4f m (%.0f mm)\n', ...
            v, oDyn, 1000*oDyn);
end
fprintf('  observed worst overlap in the batch : 0.111 m (111 mm)\n');
fprintf('  => matches the dynamic prediction at ~%.2f m/s.\n', 0.111*sqrt(P.kBody));
kNeeded = (P.v0Robot / 0.01)^2;
fprintf('  stiffness needed to cap overlap at 10 mm at v0: k > %.0f 1/s^2\n', kNeeded);
fprintf('  stable dt for that stiffness (10 steps/period)  : %.5f s (current %.3f s)\n\n', ...
        2*pi/sqrt(kNeeded)/10, P.dt);

%% ---- C. Discretisation ----------------------------------------------
fprintf('--- C. Discretisation ---\n');
maxClose = 2 * P.vMax * P.dt;
fprintf('  max closing distance per step : %.4f m\n', maxClose);
fprintf('  combined radii                : %.4f m\n', rSum);
if maxClose < rSum
    fprintf('  => tunnelling through contact is impossible; sampling can\n');
    fprintf('     understate penetration by at most %.1f mm. Not the cause.\n\n', ...
            1000*maxClose);
else
    fprintf('  => WARNING: a pair can jump past contact in one step.\n\n');
end

%% ---- Head-on simulation ---------------------------------------------
% Robot drives at a stationary pedestrian standing on the direct line to the
% goal. This is the worst case and isolates the effect from crowd dynamics.
fprintf('--- Head-on approach, stationary pedestrian on the goal line ---\n');
pedAt = [10; 0];
goal  = [20; 0];
x = [[0;0], pedAt];
v = [[0;0], [0;0]];
g = [goal,  pedAt];
radii = [P.rRobot, P.rPed];
v0    = [P.v0Robot, 0];

minGap = inf; maxPen = 0; contactSteps = 0; peakSpeed = 0;
for step = 1:6000
    [F, dg] = ComputeForces(x, v, g, radii, v0, {}, {}, P);
    F(:,2) = 0;                       % pedestrian held fixed
    v = v + F*P.dt;
    sp = sqrt(sum(v.^2,1));
    over = sp > P.vMax;
    if any(over), v(:,over) = v(:,over)./sp(over)*P.vMax; end
    v(:,2) = 0;
    x = x + v*P.dt;
    gap = dg.minPedGap(1);
    minGap = min(minGap, gap);
    if gap < 0, contactSteps = contactSteps + 1; maxPen = max(maxPen, -gap); end
    peakSpeed = max(peakSpeed, norm(v(:,1)));
    if x(1,1) > 19, break; end
end
fprintf('  min surface gap   : %+.4f m\n', minGap);
fprintf('  max penetration   : %.4f m\n', maxPen);
fprintf('  contact duration  : %.2f s (%d steps)\n', contactSteps*P.dt, contactSteps);
fprintf('  peak robot speed  : %.2f m/s\n', peakSpeed);
if minGap < 0
    fprintf('  => the robot pushes THROUGH a stationary pedestrian. The social\n');
    fprintf('     force model provides no collision guarantee here; it is a\n');
    fprintf('     soft preference that a sufficient goal drive overrides.\n');
else
    fprintf('  => contact avoided in the head-on case.\n');
end

%% ---- What repulsion strength would prevent it? -----------------------
fprintf('\n--- aPed required to hold the robot off (head-on, static ped) ---\n');
for aTry = [2.1 5 10 20 40]
    Pt = P; Pt.aPed = aTry;
    x = [[0;0], pedAt]; v = zeros(2,2); mg = inf;
    for step = 1:6000
        [F, dg] = ComputeForces(x, v, g, radii, v0, {}, {}, Pt);
        F(:,2) = 0;
        v = v + F*Pt.dt;
        sp = sqrt(sum(v.^2,1)); over = sp > Pt.vMax;
        if any(over), v(:,over) = v(:,over)./sp(over)*Pt.vMax; end
        v(:,2) = 0;
        x = x + v*Pt.dt;
        mg = min(mg, dg.minPedGap(1));
        if x(1,1) > 19, break; end
    end
    fprintf('  aPed = %5.1f  ->  min gap %+.4f m  %s\n', aTry, mg, ...
            ternary(mg >= 0, 'CLEAR', 'CONTACT'));
end

function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
