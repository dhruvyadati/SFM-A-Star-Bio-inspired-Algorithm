function P = SfmParams(varargin)
% SfmParams returns the single source of truth for every model, environment,
% and evaluation constant used by the v2 (corrected) implementation.
%
% Usage:
%   P = SfmParams();                        % defaults
%   P = SfmParams('numObstacles', 6);       % override any field by name
%
% UNITS. One grid cell is defined as 1 metre. The workspace is therefore a
% 50 m x 50 m open plaza. All speeds are m/s, accelerations m/s^2, times s.
% Fixing this convention removes the ambiguity between "square nodes",
% "units", and grid coordinates left open by the legacy model.

%% ---- Workspace -------------------------------------------------------
P.xMin = 1;  P.xMax = 50;
P.yMin = 1;  P.yMax = 50;
P.cellSize = 1.0;              % metres per grid cell (declared, not implied)

%% ---- Obstacles -------------------------------------------------------
P.numObstacles = 10;
P.maxArea      = 150;          % m^2, polygon area AFTER shoelace, before rasterisation
P.vertexRange  = [3 10];       % inclusive; the vertex sweep overrides this
P.graphMargin  = 1.0;          % m, node-pruning safety margin around polygons

%% ---- Agents ----------------------------------------------------------
P.numPedestrians = 48;
P.rPed   = 0.30;               % m, pedestrian body radius
P.rRobot = 0.35;               % m, robot body radius
% graphMargin (1.0 m) > rRobot (0.35 m), so the pruned A* graph is
% conservative with respect to robot geometry. This is now stated rather
% than left as an unexplained "one-cell margin".

%% ---- Social Force Model (Helbing & Molnar 1995; Helbing et al. 2000) --
% Every repulsion term below uses a UNIT direction vector and decays
% monotonically with distance. This corrects the legacy formulation, whose
% magnitude went as d*exp(-d) and therefore vanished at contact.
P.tau      = 0.5;              % s, relaxation time
P.v0Ped    = 1.34;             % m/s, preferred pedestrian speed
P.v0Robot  = 1.30;             % m/s, preferred robot speed
P.vMax     = 2.00;             % m/s, hard speed cap
% NOTE: there is no vMin. the legacy code clamped speed UP to 1 m/s, so no agent could
% ever stop. Agents may now decelerate to rest.
P.dSlow    = 2.0;              % m, distance over which desired speed tapers to 0

P.aPed     = 2.10;             % m/s^2, social repulsion between PEDESTRIANS
P.aPedRobot = [];              % m/s^2, the ROBOT's own sensitivity to
                               % pedestrians. Empty means "same as aPed".
% These are separate because tuning the robot's caution is a change to the
% ROBOT'S CONTROLLER, not to the crowd. Applying one scalar to every pair
% would alter pedestrian-pedestrian dynamics as well, so arms with different
% repulsion strength would no longer share a crowd and the matched-seed
% comparison would be void.
%
% The social term is therefore allowed to be asymmetric: the robot may be
% more wary of people than they are of it. That is a behavioural asymmetry,
% not a physical one, and social forces carry no third-law obligation. The
% CONTACT term (kBody) stays symmetric because it does model real contact.
P.bPed     = 0.30;             % m, pedestrian social repulsion range
P.rInfPed  = 5.0;              % m, cutoff (exp(-14.7) beyond this: negligible)

P.aObs     = 10.0;             % m/s^2, obstacle social repulsion strength
P.bObs     = 0.20;             % m, obstacle social repulsion range
P.rInfObs  = 3.0;              % m, cutoff

P.kBody    = 200.0;            % 1/s^2, contact stiffness on physical overlap
% kBody gives the "strong barrier near obstacle surfaces" that the legacy code claimed but
% did not implement. Natural period 2*pi/sqrt(k) ~ 0.44 s, so dt = 0.01 s
% resolves it with ~44 steps per period and integrates stably.

%% ---- Integration -----------------------------------------------------
P.dt     = 0.01;               % s
P.maxSteps = 15000;            % 150 s of simulated time
% The start-goal diagonal is 69.3 m; at v0Robot = 1.3 m/s an unobstructed
% traverse takes ~53 s. A 60 s budget would make the horizon itself the
% binding constraint and inflate the timeout rate, so the limit is set to
% ~2.8x the free-space traverse time. Genuine entrapment is caught earlier
% and separately by the deadlock detector (stallSteps), so a 'timeout'
% outcome now means slow progress rather than an exhausted clock.
% Semi-implicit (symplectic) Euler is used in RunTrial: velocity is advanced
% first, then position is advanced with the UPDATED velocity. the legacy code advanced
% position with the previous velocity plus 0.5*F*dt^2, which is an explicit
% constant-acceleration step, not semi-implicit Euler.

%% ---- Goal / success criteria ----------------------------------------
P.start      = [1 1];
P.goal       = [50 50];
P.rGoal      = 0.5;            % m, goal acceptance radius
P.vGoal      = 0.10;           % m/s, speed must be below this to count
P.dwellSteps = 100;            % 1.0 s the robot must REMAIN inside rGoal
P.rGoalSweep = [0.25 0.5 1.0]; % reported sensitivity band
% Arrival requires position, SPEED, and dwell together, with no contact
% during the dwell window. A 2 m radius was too coarse to defend against a
% 0.35 m robot on a 1 m grid, and it made the final-distance metric almost
% tautological: any successful run ended within 2 m by construction.
% Demanding the robot actually come to rest also exercises the stopping
% capability that removing vMin made possible.
P.rWaypoint  = 2.0;            % m, waypoint advance radius

%% ---- Time-to-collision safety filter --------------------------------
% A speed-scaling filter layered on top of the social-force controller. It
% is OFF by default so the unfiltered controller remains measurable as its
% own ablation arm.
P.ttcEnabled   = false;
P.ttcHorizon   = 2.0;          % s, start braking inside this time to contact
P.ttcStop      = 0.8;          % s, full stop at or inside this time
P.ttcClearance = 0.15;         % m, margin beyond combined radii to defend
P.ttcRecedeBand = 0.05;        % m/s, closing speed below -this counts as
                               % clearly receding and releases the floor
P.ttcSlewRate  = 4.0;          % 1/s, max change in brake scale per second.
                               % Bounds the acceleration the filter itself
                               % can inject; 4.0 lets it go full->stop in
                               % 0.25 s, fast enough to matter at 1.3 m/s.
P.ttcMinGap    = 0.40;         % m, surface gap at which distance braking starts
P.ttcHardGap   = 0.12;         % m, surface gap at which the robot must stop
% The distance floor covers the TTC blind spot: as relative speed falls, the
% predicted time to contact diverges and the horizon test stops firing, so a
% slow creep into a pedestrian goes unbraked.
%
% ttcHardGap MUST sit at or above the evaluation near-collision boundary
% (zoneNearCollision). An emergency floor below the threshold used to score
% safety would let the controller enter, by design, the very band the metric
% counts as unsafe. 0.12 > 0.10 leaves a small hysteresis band so the robot
% halts just outside it rather than on the boundary.
%
% ttcMinGap is where braking BEGINS (desired predictive clearance);
% ttcHardGap is where it reaches zero (emergency current-separation floor).
% The 0.28 m ramp between them is a tuning parameter: too narrow makes the
% response abrupt and raises jerk, too wide makes the robot timid.

%% ---- Failure classification -----------------------------------------
P.stallSpeed  = 0.05;          % m/s, below this the robot counts as stalled
P.stallSteps  = 500;           % 5 s stalled and far from goal => deadlock
P.pedGoalTol  = 1.0;           % m, pedestrian resamples its goal within this

%% ---- Social / safety metric thresholds -------------------------------
P.zoneIntimate = 0.45;         % m, Hall intimate zone
P.zonePersonal = 1.20;         % m, Hall personal zone
P.zoneNearCollision = 0.10;    % m, SURFACE gap below which a pass counts as
                               % a near miss (not centre distance)
P.typeLookback      = 100;     % steps (1.0 s) before contact onset at which
                               % velocities are read for causal attribution.
                               % Reading at onset alone biases the labels:
                               % a braked robot looks stationary at contact
                               % even when its own approach created the
                               % situation, so filtered arms would be
                               % systematically mislabelled pedestrian-driven.
P.typeNegligible    = 0.05;    % m/s, closing component below this counts
                               % as "not moving toward" when typing a contact
P.obsNearClearance  = 0.15;    % m, SURFACE gap to an obstacle below which the
                               % pass counts as a clearance violation
% Pedestrian and obstacle safety are kept conceptually separate and are NOT
% governed by a shared threshold. Pedestrian risk is dynamic and depends on
% relative motion, so it is handled predictively. Obstacle risk is static
% footprint geometry against a known map, so it is handled by inflation plus
% swept-footprint checking. A predictive filter tuned for moving people is
% the wrong instrument for a wall.

%% ---- Internal consistency -------------------------------------------
% Checked after overrides so a sweep cannot silently break the ordering.
%% ---- Apply caller overrides -----------------------------------------
if mod(numel(varargin), 2) ~= 0
    error('SfmParams:badArgs', 'Overrides must be name/value pairs.');
end
for k = 1:2:numel(varargin)
    name = varargin{k};
    if ~isfield(P, name)
        error('SfmParams:unknownField', 'Unknown parameter "%s".', name);
    end
    P.(name) = varargin{k+1};
end

if P.ttcHardGap < P.zoneNearCollision
    error('SfmParams:unsafeFloor', ...
        ['ttcHardGap (%.3f m) is below the near-collision boundary (%.3f m). ' ...
         'The emergency stop must engage before the band that scores as ' ...
         'unsafe, not inside it.'], P.ttcHardGap, P.zoneNearCollision);
end
if P.ttcMinGap <= P.ttcHardGap
    error('SfmParams:badRamp', 'ttcMinGap must exceed ttcHardGap.');
end
end
