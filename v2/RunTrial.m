function R = RunTrial(S, arm, opts)
% RunTrial simulates ONE navigation arm on ONE scenario.
%
%   S     scenario struct from MakeScenario
%   arm   either a planner name ('hybrid' | 'sfm') or a struct:
%           .planner  'hybrid' | 'sfm'
%           .aPed     optional social-repulsion override
%           .ttc      optional logical, enable the time-to-collision filter
%           .label    optional display name
%   opts  optional struct:
%           .logTrajectory  (default false) record the robot path and the
%                           terminal state of every pedestrian
%
% Scenarios are fully determined by their seed, so any trial can be REPLAYED
% with logging enabled instead of re-running a whole sweep.
%
% SAFETY MEASUREMENT. The primary outcome is SAFE SUCCESS: reached the goal
% and never made contact. Goal success alone badly overstates performance --
% an unfiltered run can reach the goal on every trial while colliding on most
% of them. Contacts are tracked per pedestrian identity, and clearance is
% evaluated on the swept segment between successive states rather than only
% at sample points.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'logTrajectory'), opts.logTrajectory = false; end
if ~isfield(opts, 'logProximity'), opts.logProximity = false; end

if ischar(arm) || isstring(arm)
    arm = struct('planner', char(arm));
end
if ~isfield(arm, 'ttc'),   arm.ttc = [];    end
if ~isfield(arm, 'aPed'),  arm.aPed = [];   end
if ~isfield(arm, 'label'), arm.label = arm.planner; end

P = S.P;
if ~isempty(arm.aPed), P.aPedRobot = arm.aPed; end   % robot only
if ~isempty(arm.ttc),  P.ttcEnabled = arm.ttc; end

nPed = P.numPedestrians;
N = 1 + nPed;                       % agent 1 is the robot

R.method   = arm.label;
R.planner  = arm.planner;
R.aPed     = P.aPed; R.aPedRobot = P.aPedRobot;
R.ttc      = P.ttcEnabled;
R.seed     = S.seed;
R.feasible = S.feasible;

if ~S.feasible
    R.outcome = 'infeasible';
    R = fillEmpty(R);
    return;
end

%% ---- State initialisation -------------------------------------------
x = zeros(2, N);
v = zeros(2, N);
% Canonical cross-validation scenarios specify an arbitrary robot state;
% generated trials always start from P.start at rest.
if isfield(opts, 'robotStart') && ~isempty(opts.robotStart)
    x(:, 1) = opts.robotStart(:);
else
    x(:, 1) = P.start(:);
end
if isfield(opts, 'robotVel') && ~isempty(opts.robotVel)
    v(:, 1) = opts.robotVel(:);
end
if nPed > 0
    x(:, 2:end) = S.pedX0;
    v(:, 2:end) = S.pedV0;
end

radii = [P.rRobot, repmat(P.rPed, 1, nPed)];
v0    = [P.v0Robot, repmat(P.v0Ped, 1, nPed)];

goals = zeros(2, N);
goals(:, 2:end) = S.pedGoal0;

% Goals are consumed IN ORDER from the scenario's pre-generated pool, so the
% sequence is explicit scenario data rather than a side effect of RNG state.
poolIdx = 0;

switch arm.planner
    case 'hybrid'
        waypoints = S.path;  wi = 1;
        goals(:, 1) = waypoints(:, 1);
    case 'sfm'
        waypoints = [];      wi = 1;
        goals(:, 1) = P.goal(:);
    otherwise
        error('RunTrial:badPlanner', 'Unknown planner "%s".', arm.planner);
end

%% ---- Accumulators ----------------------------------------------------
pathLength = 0;  dwell = 0;  stall = 0;
minPedGap = inf; minObsGap = inf;
intimateSteps = 0; personalSteps = 0;
accelSum = 0; jerkSum = 0; jerkSq = 0; prevAccel = [0; 0];
forceSeconds = 0; clampSteps = 0;

inContact      = false(1, nPed);     % per-pedestrian contact state
runLen         = zeros(1, nPed);     % current contact run, in steps
longestContact = 0;
pedEpisodes    = 0;
everContacted  = false(1, nPed);
pedContactSteps = 0;  maxPedPen = 0;
contactMovingSteps = 0;

obsCollisions = 0; inObsContact = false;
nearMissSteps = 0; inNearMiss = false; nearMissEpisodes = 0;
obsNearSteps = 0; inObsNear = false; obsNearEpisodes = 0;
nRobotDriven = 0; nPedDriven = 0; nMutual = 0; nNeitherClosing = 0;
nRadialDom = 0; nTangentialDom = 0;
% Ring buffer of velocities so contacts can be attributed using the approach
% that preceded them rather than the instant of touch.
lookN = P.typeLookback;
vHist = zeros(2, N, lookN);
vHistFill = 0;
minObsGapSwept = inf;
obsContactSteps = 0; maxObsPen = 0;

ttcActiveSteps = 0; ttcActivations = 0; ttcWasActive = false; prevBrake = 1.0;
% Slowest point away from the goal. For a SUCCESSFUL trial this is the
% closest the robot came to stalling without doing so -- the hard negative
% for the deadlock-prediction test. Points near the goal are excluded because
% decelerating to arrive is not a near-stall.
minSpeedSeen = inf; minSpeedPos = [NaN; NaN]; minSpeedPed = [];
% Tracking only begins once the robot has been up to cruising speed at least
% once. Without that gate the minimum is always the FIRST step: the robot
% starts at rest, so step 1 gives v0/tau*dt = 0.026 m/s, below anything it
% reaches later. Every negative would then be the start corner rather than a
% genuine near-stall.
cruised = false; minSpeedStep = NaN;
% Snapshot taken when a stall FIRST begins. If the stall runs to stallSteps
% this snapshot is the onset state; if it breaks, it is simply overwritten.
% Storing it this way avoids buffering 500 steps of full crowd state.
onsetSnap = struct('step', NaN, 'pos', [NaN; NaN], 'obsGap', NaN, ...
                   'nPedNear', NaN, 'nearestPed', NaN, 'brake', NaN);
brakeScaleSum = 0;

R.outcome = 'timeout';
stepsTaken = P.maxSteps;
% Initialised for every outcome so the result structs stay field-compatible
% and can be concatenated across trials.
R.deadlockOnsetStep = NaN;
R.deadlockCause = 'none';
R.onset = struct('step', NaN, 'pos', [NaN; NaN], 'obsGap', NaN, ...
                 'nPedNear', NaN, 'nearestPed', NaN, 'brake', NaN);

if opts.logTrajectory
    traj = zeros(2, P.maxSteps);
    brakeSeries = ones(1, P.maxSteps);   % brake scale per step, for inspecting
end                                      % activation shape and chatter
if opts.logProximity
    % Count of pedestrians within a fixed surface gap, per step. Needed to
    % build an EMPIRICAL null for pedestrian proximity: a uniform-density
    % estimate is the wrong denominator because pedestrians avoid obstacles
    % and concentrate in navigable space.
    proxSeries = zeros(1, P.maxSteps);
end

%% ---- Main loop -------------------------------------------------------
for step = 1:P.maxSteps
    tF = tic;
    [F, dg] = ComputeForces(x, v, goals, radii, v0, S.xObsList, S.yObsList, P);
    forceSeconds = forceSeconds + toc(tF);

    vPrev = v;
    xPrev = x;

    % --- Semi-implicit (symplectic) Euler ------------------------------
    v = v + F * P.dt;

    speed = sqrt(sum(v.^2, 1));
    over = speed > P.vMax;
    if any(over)
        v(:, over) = v(:, over) ./ speed(over) * P.vMax;
    end

    % --- Time-to-collision speed filter (robot only) -------------------
    % Applied AFTER the force update and the speed cap, so it is the last
    % authority on commanded speed and can be ablated cleanly.
    if P.ttcEnabled
        [bScale, bActive] = TtcBrake(x(:, 1), v(:, 1), x(:, 2:end), v(:, 2:end), ...
                                     radii(2:end), P, prevBrake);
        prevBrake = bScale;
        v(:, 1) = v(:, 1) * bScale;
        brakeScaleSum = brakeScaleSum + bScale;
        if opts.logTrajectory, brakeSeries(step) = bScale; end
        if bActive
            ttcActiveSteps = ttcActiveSteps + 1;
            if ~ttcWasActive, ttcActivations = ttcActivations + 1; end
        end
        ttcWasActive = bActive;
    else
        brakeScaleSum = brakeScaleSum + 1;
    end

    vHistIdx = mod(step - 1, lookN) + 1;
    vHist(:, :, vHistIdx) = v;
    vHistFill = min(vHistFill + 1, lookN);

    trueAccel = (v - vPrev) / P.dt;
    x = x + v * P.dt;

    if opts.logTrajectory, traj(:, step) = x(:, 1); end

    %% --- Robot metrics ------------------------------------------------
    pathLength = pathLength + norm(x(:, 1) - xPrev(:, 1));

    accel = trueAccel(:, 1);
    accelSum = accelSum + norm(accel);
    jd = (accel - prevAccel) / P.dt;
    jerkSum = jerkSum + norm(jd);
    jerkSq  = jerkSq + sum(jd.^2);
    prevAccel = accel;

    robSpeed = norm(v(:, 1));
    if robSpeed >= P.vMax - 1e-9, clampSteps = clampSteps + 1; end

    if robSpeed > 0.5 * P.v0Robot, cruised = true; end
    if cruised && norm(x(:, 1) - P.goal(:)) > 3.0 && ...
       norm(x(:, 1) - P.start(:)) > 3.0 && robSpeed < minSpeedSeen
        minSpeedSeen = robSpeed;
        minSpeedPos  = x(:, 1);
        minSpeedPed  = x(:, 2:end);
        minSpeedStep = step;
    end

    % --- Swept clearance to each pedestrian ----------------------------
    % Minimum separation over the CONTINUOUS segment between the previous
    % and current states, not just at the sample points. Sampling alone can
    % miss a grazing contact that opens and closes inside one step, and the
    % swept test stays correct if the timestep or speed limits change later.
    % Guard the pedestrian-free case: generated trials always carry a crowd,
    % but canonical cross-validation scenarios may have none, and min([])
    % returns empty rather than inf.
    if nPed == 0
        gapsSwept = inf;
        gMinNow = inf;
    else
    r0 = xPrev(:, 2:end) - xPrev(:, 1);
    r1 = x(:, 2:end)     - x(:, 1);
    w  = r1 - r0;
    ww = sum(w.^2, 1);
    sPar = zeros(1, nPed);
    nz = ww > eps;
    sPar(nz) = min(1, max(0, -sum(r0(:, nz) .* w(:, nz), 1) ./ ww(nz)));
    closest = r0 + sPar .* w;
    gapsSwept = sqrt(sum(closest.^2, 1)) - (P.rRobot + radii(2:end));
    gMinNow = min(gapsSwept);
    end

    minPedGap = min(minPedGap, gMinNow);

    if opts.logProximity
        proxSeries(step) = sum(gapsSwept < 1.5);
    end

    % Near miss: positive but sub-threshold surface clearance. Reported
    % separately from contact so "no collisions" cannot hide a stream of
    % grazing passes.
    isNear = gMinNow >= 0 && gMinNow < P.zoneNearCollision;
    if isNear
        nearMissSteps = nearMissSteps + 1;
        if ~inNearMiss, nearMissEpisodes = nearMissEpisodes + 1; inNearMiss = true; end
    else
        inNearMiss = false;
    end

    touching = gapsSwept < 0;
    if nPed > 0 && any(touching)
        pedContactSteps = pedContactSteps + 1;
        maxPedPen = max(maxPedPen, -min(gapsSwept));
        if robSpeed > P.stallSpeed
            contactMovingSteps = contactMovingSteps + 1;
        end
    end

    % Episode bookkeeping, per pedestrian identity.
    if nPed == 0, touching = false(1, 0); end
    newContact = touching & ~inContact;
    pedEpisodes = pedEpisodes + sum(newContact);

    % Classify each NEW contact on TWO INDEPENDENT axes. Forcing causation
    % and encounter geometry into one exclusive category would let a
    % marginally larger tangential component mask genuine closing behaviour,
    % so they are labelled separately.
    %
    %   causation : who was closing -- robot / pedestrian / mutual / neither
    %   geometry  : radial-dominant vs tangential-dominant encounter
    %
    % Velocities are read from typeLookback steps BEFORE onset wherever that
    % history exists, so a robot that braked hard just before contact is
    % still attributed with the approach it made.
    for iNew = find(newContact)
        rvec = x(:, 1 + iNew) - x(:, 1);
        rn = norm(rvec);
        if rn < eps, continue; end
        rhat = rvec / rn;

        if vHistFill >= lookN
            backIdx = mod(step - lookN, lookN) + 1;
            vR = vHist(:, 1, backIdx);
            vP = vHist(:, 1 + iNew, backIdx);
        else
            vR = v(:, 1);  vP = v(:, 1 + iNew);
        end

        robClose = dot(vR, rhat);          % >0: robot was moving at the ped
        pedClose = -dot(vP, rhat);         % >0: ped was moving at the robot
        urel = vP - vR;
        closing = -dot(urel, rhat);
        tangential = norm(urel - dot(urel, rhat) * rhat);

        rC = robClose > P.typeNegligible;
        pC = pedClose > P.typeNegligible;
        if rC && pC
            nMutual = nMutual + 1;
        elseif rC
            nRobotDriven = nRobotDriven + 1;
        elseif pC
            nPedDriven = nPedDriven + 1;
        else
            nNeitherClosing = nNeitherClosing + 1;
        end

        if tangential > abs(closing)
            nTangentialDom = nTangentialDom + 1;
        else
            nRadialDom = nRadialDom + 1;
        end
    end
    everContacted = everContacted | touching;
    runLen(touching) = runLen(touching) + 1;
    ended = ~touching & inContact;
    if any(ended), longestContact = max(longestContact, max(runLen(ended))); end
    runLen(~touching) = 0;
    inContact = touching;

    % --- Obstacle clearance ---------------------------------------------
    % Swept along the step segment. Motion is at most 2 cm per step, so three
    % samples bound the continuous minimum tightly. Handled separately from
    % pedestrian clearance: this is static geometry, not relative motion.
    gapObs = dg.minObsGap(1);
    midPt = 0.5 * (xPrev(:, 1) + x(:, 1));
    gSwept = gapObs;
    for probe = [midPt, x(:, 1)]
        dO = inf;
        for kObs = 1:numel(S.xObsList)
            dO = min(dO, p_poly_dist(probe(1), probe(2), ...
                                     S.xObsList{kObs}, S.yObsList{kObs}));
        end
        gSwept = min(gSwept, dO - P.rRobot);
    end
    gapObs = gSwept;
    minObsGapSwept = min(minObsGapSwept, gapObs);
    minObsGap = min(minObsGap, gapObs);

    if gapObs >= 0 && gapObs < P.obsNearClearance
        obsNearSteps = obsNearSteps + 1;
        if ~inObsNear, obsNearEpisodes = obsNearEpisodes + 1; inObsNear = true; end
    else
        inObsNear = false;
    end
    if gapObs < 0
        if ~inObsContact, obsCollisions = obsCollisions + 1; inObsContact = true; end
        obsContactSteps = obsContactSteps + 1;
        maxObsPen = max(maxObsPen, -gapObs);
    else
        inObsContact = false;
    end

    % --- Proxemics -------------------------------------------------------
    centreGap = gMinNow + P.rRobot + P.rPed;
    if centreGap < P.zoneIntimate, intimateSteps = intimateSteps + 1; end
    if centreGap < P.zonePersonal, personalSteps = personalSteps + 1; end

    % --- Waypoint advance ------------------------------------------------
    if strcmp(arm.planner, 'hybrid')
        if norm(x(:, 1) - waypoints(:, wi)) < P.rWaypoint && wi < size(waypoints, 2)
            wi = wi + 1;
        end
        goals(:, 1) = waypoints(:, wi);
    end

    % --- Pedestrian goal resampling --------------------------------------
    for i = 2:N
        if norm(x(:, i) - goals(:, i)) < P.pedGoalTol
            poolIdx = poolIdx + 1;
            if poolIdx > size(S.goalPool, 2)
                error('RunTrial:poolExhausted', ...
                      'Goal pool exhausted at step %d; increase nPool.', step);
            end
            goals(:, i) = S.goalPool(:, poolIdx);
        end
    end

    % --- Termination ------------------------------------------------------
    % Arrival needs position, SPEED, and dwell together, with no contact
    % during the dwell window. A robot that sails through the goal region or
    % is pinned against a pedestrian inside it has not arrived.
    distToGoal = norm(x(:, 1) - P.goal(:));
    if distToGoal <= P.rGoal && robSpeed <= P.vGoal && ~any(touching)
        dwell = dwell + 1;
        if dwell >= P.dwellSteps
            R.outcome = 'success';
            stepsTaken = step;
            break;
        end
    else
        dwell = 0;
    end

    if robSpeed < P.stallSpeed && distToGoal > P.rGoal
        if stall == 0
            gapsNow = sqrt(sum((x(:, 2:end) - x(:, 1)).^2, 1)) - (P.rRobot + radii(2:end));
            onsetSnap.step       = step;
            onsetSnap.pos        = x(:, 1);
            onsetSnap.obsGap     = gapObs;
            onsetSnap.nPedNear   = sum(gapsNow < 1.5);
            onsetSnap.nearestPed = min(gapsNow);
            if P.ttcEnabled, onsetSnap.brake = prevBrake; else, onsetSnap.brake = 1; end
        end
        stall = stall + 1;
        if stall >= P.stallSteps
            R.outcome = 'deadlock';
            stepsTaken = step;
            % Onset is when the stall BEGAN, not when it was confirmed. The
            % horizon task predicts onset, so using the termination step would
            % shift every label by stallSteps (5 s) and make the prediction
            % task easier than stated.
            R.deadlockOnsetStep = step - P.stallSteps + 1;
            R.onset = onsetSnap;
            % Objective cause classification with frozen thresholds. The
            % static force field represents static structure only, so claims
            % about what it predicts must be scoped to the class it covers.
            if onsetSnap.brake < 0.5
                R.deadlockCause = 'controller';
            elseif onsetSnap.nPedNear >= 1 && onsetSnap.obsGap > 1.0
                R.deadlockCause = 'pedestrian';
            elseif onsetSnap.obsGap <= 1.0 && onsetSnap.nPedNear == 0
                R.deadlockCause = 'static';
            elseif onsetSnap.obsGap <= 1.0 && onsetSnap.nPedNear >= 1
                R.deadlockCause = 'mixed';
            else
                R.deadlockCause = 'other';
            end
            break;
        end
    else
        stall = 0;
    end
end

if any(inContact)
    longestContact = max(longestContact, max(runLen(inContact)));
end

%% ---- Report ----------------------------------------------------------
R.success         = strcmp(R.outcome, 'success');
R.steps           = stepsTaken;
R.simSeconds      = stepsTaken * P.dt;
R.pathLength      = pathLength;
R.finalDistToGoal = norm(x(:, 1) - P.goal(:));
R.finalSpeed      = norm(v(:, 1));

% Safety -- primary outcomes.
R.pedCollisions   = pedEpisodes;
R.obsCollisions   = obsCollisions;
R.pedsContacted   = sum(everContacted);
R.collisionFree   = (pedEpisodes == 0) && (obsCollisions == 0);
R.safeSuccess     = R.success && R.collisionFree;
R.pedContactTime  = pedContactSteps * P.dt;
R.obsContactTime  = obsContactSteps * P.dt;
R.longestContact  = longestContact * P.dt;
R.contactMoving   = contactMovingSteps * P.dt;
R.maxPedPen       = maxPedPen;
R.maxObsPen       = maxObsPen;
R.minPedGap       = minPedGap;
R.nearMisses      = nearMissEpisodes;
R.obsNearMisses   = obsNearEpisodes;
R.cRobotDriven    = nRobotDriven;   % robot was closing, pedestrian was not
R.cPedDriven      = nPedDriven;     % pedestrian was closing, robot was not
R.cMutual         = nMutual;
R.cNeither        = nNeitherClosing;
R.cRadialDom      = nRadialDom;     % geometry axis, independent of causation
R.cTangentialDom  = nTangentialDom;
R.obsNearTime     = obsNearSteps * P.dt;
R.minObsGapSwept  = minObsGapSwept;
R.nearMissTime    = nearMissSteps * P.dt;
R.minObsGap       = minObsGap;

% Proportions, so unequal trial lengths cannot drive a comparison. All are
% censored at termination: the denominator is time actually simulated.
R.personalFrac    = personalSteps / stepsTaken;
R.intimateFrac    = intimateSteps / stepsTaken;
R.pedContactFrac  = pedContactSteps / stepsTaken;
R.personalTime    = personalSteps * P.dt;
R.intimateTime    = intimateSteps * P.dt;
R.collPerMetre    = (pedEpisodes + obsCollisions) / max(pathLength, eps);

% Smoothness. RMS is the headline: mean |da/dt| is dominated by rare contact
% spikes and understates how spiky the trajectory actually is.
R.meanAccel       = accelSum / stepsTaken;
R.meanJerk        = jerkSum / stepsTaken;
R.rmsJerk         = sqrt(jerkSq / stepsTaken);
R.clampFraction   = clampSteps / stepsTaken;

% Safety-filter duty cycle. A filter that engages almost always is not an
% emergency mechanism, it is the controller.
R.ttcActiveFrac   = ttcActiveSteps / stepsTaken;
R.ttcActivations  = ttcActivations;
R.ttcBrakeTime    = ttcActiveSteps * P.dt;
R.meanBrakeScale  = brakeScaleSum / stepsTaken;

R.astarSeconds    = S.astarSeconds;
R.forceSeconds    = forceSeconds;
R.meanStepSeconds = forceSeconds / stepsTaken;
R.controlHz       = 1 / max(R.meanStepSeconds, eps);

R.minSpeed      = minSpeedSeen;
R.minSpeedStep  = minSpeedStep;
R.minSpeedPos   = minSpeedPos;
R.minSpeedPed   = minSpeedPed;
R.finalRobotPos = x(:, 1);
R.finalRobotVel = v(:, 1);
if opts.logProximity
    R.proxSeries = proxSeries(1:stepsTaken);
end
if opts.logTrajectory
    R.traj    = traj(:, 1:stepsTaken);
    R.brakeSeries = brakeSeries(1:stepsTaken);
    R.pedPos  = x(:, 2:end);
    R.pedVel  = v(:, 2:end);
    R.pedGoal = goals(:, 2:end);
end
end

% ---------------------------------------------------------------------
function R = fillEmpty(R)
    f = {'success','steps','simSeconds','pathLength','finalDistToGoal', ...
         'finalSpeed','pedCollisions','obsCollisions','pedsContacted', ...
         'pedContactTime','obsContactTime','longestContact','contactMoving', ...
         'maxPedPen','maxObsPen','minPedGap','minObsGap','personalFrac', ...
         'intimateFrac','pedContactFrac','personalTime','intimateTime', ...
         'collPerMetre','meanAccel','meanJerk','rmsJerk','clampFraction', ...
         'ttcActiveFrac','ttcActivations','ttcBrakeTime','meanBrakeScale', ...
         'nearMisses','nearMissTime','obsNearMisses','obsNearTime', ...
         'minObsGapSwept','cRobotDriven','cPedDriven','cMutual', ...
         'cNeither','cRadialDom','cTangentialDom', ...
         'astarSeconds','forceSeconds','meanStepSeconds','controlHz'};
    for k = 1:numel(f), R.(f{k}) = NaN; end
    R.success = false;
    R.minSpeed = NaN; R.minSpeedStep = NaN; R.minSpeedPos = [NaN; NaN]; R.minSpeedPed = [];
    R.collisionFree = false;
    R.safeSuccess = false;
    R.finalRobotPos = [NaN; NaN];
    R.finalRobotVel = [NaN; NaN];
end

