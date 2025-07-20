function M = Metrics(u, path, dt, varargin)
%------------------------------------------------------------------
% Metrics  Compute navigation‑quality metrics for ONE robot.
%
%   M = Metrics(u, path, dt, 'Name', Value, ...)
%
% INPUTS
%   u(4,1,N)   Robot state history [x;y;vx;vy] over N steps.
%              (If you have 4×p×N, slice u(:,robotIdx,:) first.)
%   path(2,K)  Way‑points produced by A*  (columns = way‑points).
%   dt         Simulation time‑step (s).
%
% OPTIONAL NAME–VALUE PAIRS
%   'SafetyRadius'   Collision radius (m)       [default 0.30]
%   'ComfortRadius'  Personal‑space radius (m)  [default 0.60]
%   'GoalEps'        Goal tolerance (m)         [default 0.10]
%   'ReplanCount'    # global re‑plans          [default 0]
%   'CPUTime'        Planning CPU time  (s)     [default NaN]
%
% OUTPUT STRUCT M
%   success          1 if goal reached within GoalEps
%   travelTime       Total simulated time (s)
%   optimalLen       Length of A* path (m)
%   actualLen        Length actually travelled (m)
%   SPL              Success‑Weighted Path Length
%   minSep           *Fill externally* (nearest human distance, m)
%   collisionCount   *Fill externally* (# frames dist < SafetyRadius)
%   intrusionFrac    *Fill externally* (time inside ComfortRadius)
%   pathSmooth       Mean |Δheading| per step (deg)
%   replanCount      Global re‑plan calls (for baselines)
%   cpuTime          Planning CPU time (s)
%
%------------------------------------------------------------------

% ---------- parse options ----------
p = inputParser;
addParameter(p,'SafetyRadius',0.30);
addParameter(p,'ComfortRadius',0.60);
addParameter(p,'GoalEps',0.10);
addParameter(p,'ReplanCount',0);
addParameter(p,'CPUTime',NaN);
parse(p,varargin{:});
opt = p.Results;

% ---------- basic trajectory data ---
robotPos = squeeze(u(1:2,1,:)).';           % N×2
N = size(robotPos,1);

% ---------- success & timing ---------
goalPos = path(:,end).';
success = all(abs(robotPos(end,:) - goalPos) < opt.GoalEps);
travelT = (N-1)*dt;

% ---------- path lengths -------------
optLen = sum(vecnorm(diff(path.',1,1),2,2));
actLen = sum(vecnorm(diff(robotPos, 1,1),2,2));
SPL    = success * optLen / max(actLen,eps);

% ---------- smoothness ---------------
if N < 3
    smooth = NaN;
else
    seg   = diff(robotPos,1,1);                 % segment vectors
    hdg   = unwrap(atan2(seg(:,2), seg(:,1)));  % rad
    smooth= mean(abs(diff(hdg)));               % rad/step
    smooth= rad2deg(smooth);                    % deg/step
end

% ---------- package -------------------
M.success         = logical(success);
M.travelTime      = travelT;
M.optimalLen      = optLen;
M.actualLen       = actLen;
M.SPL             = SPL;
% crowdsourced fields left NaN for caller to fill:
M.minSep          = NaN;
M.collisionCount  = NaN;
M.intrusionFrac   = NaN;
M.pathSmooth      = smooth;
M.replanCount     = opt.ReplanCount;
M.cpuTime         = opt.CPUTime;
end
