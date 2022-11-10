function followPath(path)
% % Simple function to move the robot through the provided path

vrep=remApi('remoteApi');
global simulationHandlers_t;
turnToPoint = true;
direction = 1;
pointOrientation = 0;

for i=1:1:size(path,1)
    % Get robot data
    [~, robotPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
    robotPosition = round(robotPosition,3);
    currentPathPoint = round(path(i,1:2),3);

    [~, robotOrientationEuler]=vrep.simxGetObjectOrientation(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
    robotOrientationEuler_deg = rad2deg(robotOrientationEuler);

    % Determine quadrant (important to calculate the goal angle)
    if (currentPathPoint(1) > robotPosition(1)) && (currentPathPoint(2) > robotPosition(2))
        pointOrientation = atand((currentPathPoint(2)-robotPosition(2))/(currentPathPoint(1)-robotPosition(1)));
    elseif (currentPathPoint(1) < robotPosition(1)) && (currentPathPoint(2) > robotPosition(2))
        pointOrientation = 180+atand((currentPathPoint(2)-robotPosition(2))/(currentPathPoint(1)-robotPosition(1)));
    elseif (currentPathPoint(1) < robotPosition(1)) && (currentPathPoint(2) < robotPosition(2))
        pointOrientation = 180+atand((currentPathPoint(2)-robotPosition(2))/(currentPathPoint(1)-robotPosition(1)));
    elseif (currentPathPoint(1) > robotPosition(1)) && (currentPathPoint(2) < robotPosition(2))
        pointOrientation = 360+atand((currentPathPoint(2)-robotPosition(2))/(currentPathPoint(1)-robotPosition(1)));
    elseif (currentPathPoint(1) == robotPosition(1)) && (currentPathPoint(2) > robotPosition(2))
        pointOrientation = 90;
    elseif (currentPathPoint(1) == robotPosition(1)) && (currentPathPoint(2) < robotPosition(2))
        pointOrientation = 270;
    elseif (currentPathPoint(1) > robotPosition(1)) && (currentPathPoint(2) == robotPosition(2))
        pointOrientation = 0;
    elseif (currentPathPoint(1) < robotPosition(1)) && (currentPathPoint(2) == robotPosition(2))
        pointOrientation = 180;
    else
        pointOrientation = robotOrientation;
    end

    
    if robotOrientationEuler_deg(3) < pointOrientation
        direction = 1;
    else 
        direction = -1;
    end

    turn(direction,turnToPoint,pointOrientation,false,1,0.1); 
    while ~isNearby([path(i,1:2) robotPosition(3)],0.5)
        move(1.4)
    end
end
move(0)
end

