clc;
clearvars;

data = load('parkingLotCostmapReducedInflation.mat');
costmap = data.parkingLotCostmapReducedInflation;
plot(costmap)

startPose = [11, 10, 0]; % [meters, meters, degrees]
goalPose  = [31.5, 17, 90];

planner = pathPlannerRRT(costmap);
refPath = plan(planner,startPose,goalPose);

plot(planner)
