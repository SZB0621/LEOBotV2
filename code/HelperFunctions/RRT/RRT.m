function pathPoints = RRT(start,goal,map,mapTranslation,mapScale)
% % Basic funtion to plan a path between the start and goal points on the
% given map using the provided map transformation parameters
% it uses the built in RRT/RRTStart matlab planner

show(map)

startScaled = (start+mapTranslation)*mapScale;
startScaled(3) = 0.0;
goalScaled = (goal+mapTranslation)*mapScale;
goalScaled(3) = 0.0;

% Show start and goal positions of robot.
hold on
plot(startScaled(1),startScaled(2),'ro')
plot(goalScaled(1),goalScaled(2),'mo')

% Show start and goal headings.
r = 1;
plot([startScaled(1),startScaled(1) + r*cos(startScaled(3))],[startScaled(2),startScaled(2) + r*sin(startScaled(3))],'r-')
plot([goalScaled(1),goalScaled(1) + r*cos(goalScaled(3))],[goalScaled(2),goalScaled(2) + r*sin(goalScaled(3))],'m-')
hold off

bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.1;

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 1;

planner = plannerRRTStar(ss,stateValidator);
% planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 10;
planner.GoalBias = 0.025;
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 1000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

rng default

[pthObj, solnInfo] = plan(planner,double(startScaled),double(goalScaled));

show(map)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Show start and goal in grid map.
plot(startScaled(1),startScaled(2),'ro')
plot(goalScaled(1),goalScaled(2),'mo')
hold off

pathPoints = (pthObj.States(1:length(pthObj.States),1:2)/mapScale)-mapTranslation;
end



