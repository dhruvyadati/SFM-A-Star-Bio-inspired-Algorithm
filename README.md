# Read Me

Combining bio-inspired algorithms, SFM + A*, for enhanced path finding and navigation.


For : GenerateEnvironment, GenerateObstacle, and VisualizePathfindingEnvironment
  - Obstacles are shown as red transparent polygons.
  - A start point (black) and goal point (green) are marked.
  - An occupancy grid matrix `g` is created (use function GenerateEnvironment):
      1 = free space
      0 = obstacle
  - The matrix size is [yMax, xMax] and can be used for pathfinding.
