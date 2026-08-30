% Test_GainAsymmetry.m
% MANDATORY REGRESSION TEST for the robot social-repulsion gain.
%
% aPedRobot must alter the force the ROBOT RECEIVES from pedestrians, and
% must not alter the force pedestrians receive from the robot or from each
% other. If the interaction matrix were indexed the other way round -- row as
% source rather than receiver -- raising the gain would silently change
% pedestrian behaviour instead, and every arm using a raised gain would be
% comparing different crowds while appearing to compare the same one.
%
% The convention under test:
%   dx(i,j) = x_i - x_j,  F(i) = sum_j mag(i,j) * unit(i,j)
%   => ROW  = receiving agent
%   => aMat(1,:) therefore scales what the ROBOT feels.
%
% Usage:  cd to the repository root, then  run('v2/Test_GainAsymmetry.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

fprintf('=== Robot gain asymmetry ===\n\n');

% Robot at the origin, one pedestrian 0.9 m away, both at rest, goals far off
% along +x so the goal term is identical in both configurations.
x = [0 0.9; 0 0];
v = zeros(2, 2);
g = [100 100; 0 0];

P = SfmParams();
radii = [P.rRobot, P.rPed];
v0    = [P.v0Robot, P.v0Ped];

Pa = P; Pa.aPedRobot = [];      % nominal: robot uses the pedestrian gain
Pb = P; Pb.aPedRobot = 5.0;     % raised robot gain

Fa = ComputeForces(x, v, g, radii, v0, {}, {}, Pa);
Fb = ComputeForces(x, v, g, radii, v0, {}, {}, Pb);

dRobot = norm(Fb(:, 1) - Fa(:, 1));
dPed   = norm(Fb(:, 2) - Fa(:, 2));

fprintf('  robot force, nominal gain : [%+8.4f %+8.4f]\n', Fa(1,1), Fa(2,1));
fprintf('  robot force, raised gain  : [%+8.4f %+8.4f]\n', Fb(1,1), Fb(2,1));
fprintf('  |change| on ROBOT         : %.6f\n', dRobot);
fprintf('  ped force,   nominal gain : [%+8.4f %+8.4f]\n', Fa(1,2), Fa(2,2));
fprintf('  ped force,   raised gain  : [%+8.4f %+8.4f]\n', Fb(1,2), Fb(2,2));
fprintf('  |change| on PEDESTRIAN    : %.6e\n\n', dPed);

pass = 0; fail = 0;
if dRobot > 1e-6
    fprintf('  [PASS] raising the gain changes the force on the robot\n'); pass = pass + 1;
else
    fprintf('  [FAIL] the gain had no effect on the robot\n'); fail = fail + 1;
end
if dPed < 1e-12
    fprintf('  [PASS] the pedestrian is unaffected (row = receiving agent)\n'); pass = pass + 1;
else
    fprintf('  [FAIL] the pedestrian WAS affected -- indexing is transposed,\n');
    fprintf('         so the gain is altering crowd behaviour, not the robot\n'); fail = fail + 1;
end

% The robot must be pushed AWAY from the pedestrian, i.e. in -x here.
if Fb(1, 1) < Fa(1, 1)
    fprintf('  [PASS] raised gain pushes the robot further from the pedestrian\n'); pass = pass + 1;
else
    fprintf('  [FAIL] raised gain did not increase repulsion away from the ped\n'); fail = fail + 1;
end

% Pedestrian-pedestrian pairs must never see the robot gain.
x3 = [0 0.9 1.8; 0 0 0];
v3 = zeros(2, 3);
g3 = [100 100 100; 0 0 0];
r3 = [P.rRobot, P.rPed, P.rPed];
u3 = [P.v0Robot, P.v0Ped, P.v0Ped];
F3a = ComputeForces(x3, v3, g3, r3, u3, {}, {}, Pa);
F3b = ComputeForces(x3, v3, g3, r3, u3, {}, {}, Pb);
dPedPed = norm(F3b(:, 3) - F3a(:, 3));
fprintf('  |change| on far pedestrian: %.6e\n', dPedPed);
if dPedPed < 1e-12
    fprintf('  [PASS] pedestrian-pedestrian interaction is untouched\n'); pass = pass + 1;
else
    fprintf('  [FAIL] pedestrian-pedestrian interaction changed\n'); fail = fail + 1;
end

fprintf('\n=== %d passed, %d failed ===\n', pass, fail);
if fail > 0
    error('Test_GainAsymmetry:failed', ...
          'Robot gain is not correctly scoped; arms with raised gain would not share a crowd model.');
end
