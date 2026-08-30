% Test_StabilityCriterion.m
% Verifies which Jacobian test is valid for classifying trap points, by
% checking both criteria against direct simulation of the actual dynamics.
%
% THE QUESTION. The navigation system is SECOND order:
%
%     xdot = v
%     vdot = F(x, v)
%
% so the state is (x, v) in R^4, not x in R^2. Classifying an equilibrium by
% the eigenvalues of the 2x2 position Jacobian A = dF/dx therefore tests the
% wrong system. Only the goal term depends on velocity, and linearly, so
% dF/dv = -I/tau exactly and the true Jacobian is
%
%     J4 = [  0     I   ]
%          [  A   -I/tau]
%
% whose characteristic polynomial factors as lambda^2 + lambda/tau - mu = 0
% for each eigenvalue mu of A. Hence:
%
%     mu real     : stable  <=>  mu < 0                    (2x2 test correct)
%     mu = a + bi : stable  <=>  a < 0 AND b^2 < |a|/tau^2 (2x2 test WRONG)
%
% The 2x2 test is necessary but not sufficient: it declares underdamped
% rotational equilibria stable when the second-order system actually spirals
% outward. This script constructs such a case and confirms it by simulation.
%
% Usage:  cd to the repository root, then  run('v2/Test_StabilityCriterion.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();
tau = P.tau;
fprintf('=== Stability criterion verification (tau = %.2f) ===\n\n', tau);
fprintf('Damping condition for complex mu = a+bi:  b^2 < |a|/tau^2 = %.2f|a|\n\n', 1/tau^2);

% Each row: a rotational A = [a -b; b a] with eigenvalues a +/- bi.
cases = [ -1.0  0.5;      % weak rotation   -> genuinely stable
          -1.0  3.0;      % strong rotation -> 2x2 says stable, truth unstable
          -0.2  2.0;      % weak damping    -> 2x2 says stable, truth unstable
          -2.0  1.0;      % strong damping  -> genuinely stable
           0.5  1.0 ];    % positive real   -> both agree: unstable

fprintf('%6s %6s | %-9s %-9s | %-10s | %s\n', ...
        'a','b','2x2 test','4x4 test','simulated','agreement');
fprintf('%s\n', repmat('-', 1, 76));

nDisagree = 0;
for k = 1:size(cases,1)
    a = cases(k,1); b = cases(k,2);
    A = [a -b; b a];

    stable2 = all(real(eig(A)) < 0);

    J4 = [zeros(2), eye(2); A, -eye(2)/tau];
    stable4 = all(real(eig(J4)) < 0);

    % Ground truth: integrate the real second-order system from a small
    % perturbation and see whether it decays or grows.
    x = [1e-3; 0]; v = [0; 0]; dt = 1e-4;
    for i = 1:round(40/dt)
        F = A*x - v/tau;          % linearised dynamics about the equilibrium
        v = v + F*dt;             % semi-implicit Euler, as in RunTrial
        x = x + v*dt;
        if norm(x) > 1e3, break; end
    end
    simStable = norm(x) < 1e-3;

    agree = (stable4 == simStable);
    if stable2 ~= simStable, nDisagree = nDisagree + 1; end

    fprintf('%6.2f %6.2f | %-9s %-9s | %-10s | %s\n', a, b, ...
        yn(stable2), yn(stable4), yn(simStable), ...
        tern(agree, '4x4 correct', '4x4 WRONG'));

    assert(agree, 'Test_StabilityCriterion:mismatch', ...
           '4x4 criterion disagreed with simulation at a=%.2f b=%.2f', a, b);
end

fprintf('\n');
fprintf('4x4 criterion matched simulation in all %d cases.\n', size(cases,1));
fprintf('2x2 criterion was WRONG in %d of %d cases -- all false positives\n', ...
        nDisagree, size(cases,1));
fprintf('(declaring an outward spiral stable).\n\n');

if nDisagree > 0
    fprintf('CONCLUSION: the 2x2 position-Jacobian test is invalid for this\n');
    fprintf('system. TrapField.m must use the full 4x4 Jacobian.\n');
else
    error('Test_StabilityCriterion:noEvidence', ...
          'Expected the 2x2 test to fail on at least one rotational case.');
end

function s = yn(b), if b, s = 'stable'; else, s = 'unstable'; end, end
function s = tern(c, a, b), if c, s = a; else, s = b; end, end
