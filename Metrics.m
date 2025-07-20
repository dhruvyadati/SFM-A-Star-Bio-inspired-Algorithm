function M = Metrics (~, ~, ~, varargin)
p = inputParser;
addParameter(p, 'CollisionCount', NaN, @isnumeric);
parse(p, varargin{:});
M.CollisionCount = p.Results.CollisionCount;
end