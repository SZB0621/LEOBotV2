function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
% % Basic ecampe function provided by mathworks to determine if the robot's
% pose is within a given trashold to the goal state
isReached = false;
    threshold = 0.5;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end