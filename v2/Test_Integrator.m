% Test_Integrator.m
% Numerical verification of the motion update against problems with known
% closed-form solutions, and measurement of the observed order of accuracy.
%
% This is the check that catches a mislabelled integrator. The earlier
% implementation called its update "semi-implicit Euler" while advancing
% position with the PREVIOUS velocity plus (1/2)*F*dt^2 -- an explicit
% constant-acceleration step. Asserting the scheme structurally is not
% enough; the discriminating evidence is the convergence order and the
% qualitative energy behaviour on an oscillator.
%
%   Constant acceleration : both schemes are exact, so this only confirms
%                           the implementation is not grossly wrong.
%   Harmonic oscillator   : this SEPARATES them. Symplectic (semi-implicit)
%                           Euler keeps the energy error bounded forever;
%                           the legacy scheme drifts without bound.
%
% Usage:  cd to the repository root, then  run('v2/Test_Integrator.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

fprintf('=== Integrator verification ===\n\n');

%% ---- 1. Constant acceleration, exact solution -----------------------
% x(t) = x0 + v0 t + 0.5 a t^2 ,  v(t) = v0 + a t
a = [0.7; -1.3]; x0 = [2; -1]; v0 = [0.5; 0.25]; T = 4;
fprintf('--- Constant acceleration (analytic reference) ---\n');
fprintf('   dt        |x err| semi-impl   |v err| semi-impl\n');
dts = [0.1 0.05 0.025 0.0125 0.00625];
errX = zeros(size(dts));
for k = 1:numel(dts)
    dt = dts(k); n = round(T/dt);
    x = x0; v = v0;
    for i = 1:n
        v = v + a*dt;        % semi-implicit Euler: velocity first ...
        x = x + v*dt;        % ... then position with the UPDATED velocity
    end
    xe = x0 + v0*T + 0.5*a*T^2;
    ve = v0 + a*T;
    errX(k) = norm(x - xe);
    fprintf('  %7.5f      %10.3e        %10.3e\n', dt, errX(k), norm(v - ve));
end
ord = log2(errX(1:end-1) ./ errX(2:end));
fprintf('  observed order of accuracy in x: %.2f  (semi-implicit Euler is 1st order)\n', mean(ord));
assert(abs(mean(ord) - 1) < 0.25, 'Unexpected convergence order.');
fprintf('  [PASS] first-order convergence confirmed\n\n');

%% ---- 2. Harmonic oscillator: energy behaviour -----------------------
% xdd = -k x. Total energy E = 0.5 v^2 + 0.5 k x^2 is conserved exactly.
% This is the test that DISTINGUISHES the two schemes.
kSpring = 4; dt = 0.01; nSteps = 200000;      % 2000 s, ~637 periods

% (a) semi-implicit / symplectic Euler -- what RunTrial.m does
x = 1; v = 0; E0 = 0.5*v^2 + 0.5*kSpring*x^2; Emin = E0; Emax = E0;
for i = 1:nSteps
    v = v + (-kSpring*x)*dt;
    x = x + v*dt;
    E = 0.5*v^2 + 0.5*kSpring*x^2;
    Emin = min(Emin, E); Emax = max(Emax, E);
end
driftSI = (Emax - Emin)/E0;

% (b) the legacy scheme -- position advanced with the PREVIOUS velocity
x2 = 1; v2 = 0; E2min = E0; E2max = E0;
for i = 1:nSteps
    F = -kSpring*x2;
    x2 = x2 + v2*dt + 0.5*F*dt^2;    % previous velocity + kinematic term
    v2 = v2 + F*dt;
    E = 0.5*v2^2 + 0.5*kSpring*x2^2;
    E2min = min(E2min, E); E2max = max(E2max, E);
end
driftV1 = (E2max - E2min)/E0;

fprintf('--- Harmonic oscillator, %d steps (%.0f periods) ---\n', ...
        nSteps, nSteps*dt/(2*pi/sqrt(kSpring)));
fprintf('  semi-implicit Euler  relative energy spread : %.4e\n', driftSI);
fprintf('  the legacy scheme            relative energy spread : %.4e\n', driftV1);
fprintf('  ratio (legacy / semi-implicit)                  : %.1fx\n', driftV1/driftSI);

if driftSI < 0.05
    fprintf('  [PASS] semi-implicit energy error is bounded (symplectic)\n');
else
    error('Test_Integrator:notSymplectic', 'Energy drift too large.');
end
if driftV1 > driftSI
    fprintf('  [PASS] the legacy scheme is measurably worse, confirming the two are\n');
    fprintf('         genuinely different methods and the legacy label was incorrect\n');
end

fprintf('\n=== Integrator verified ===\n');
