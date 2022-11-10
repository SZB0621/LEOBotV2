% % Patroling main
% This script plans path between the given april tags and moves the robot
% through this path

% Init section
vrep=remApi('remoteApi'); % Initializing the remote api file for coppelia
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',20000,true,true,5000,5); % Setup connection parameters

% Global variables
global simulationHandlers_t;
simulationHandlers_t.clientID = clientID;
simulationHandlers_t.pioneer_Robot = NaN;
simulationHandlers_t.reference_Box = NaN;
simulationHandlers_t.left_Motor = NaN;
simulationHandlers_t.right_Motor = NaN;
simulationHandlers_t.front_LaserSensor = NaN;
simulationHandlers_t.front_LaserSensor_leftAngle = NaN;
simulationHandlers_t.front_LaserSensor_rightAngle = NaN;
simulationHandlers_t.right_LaserSensor_front = NaN;
simulationHandlers_t.right_LaserSensor_rear = NaN;
simulationHandlers_t.left_LaserSensor_front = NaN;
simulationHandlers_t.left_LaserSensor_rear = NaN;
simulationHandlers_t.back_LaserSensor_right = NaN;
simulationHandlers_t.back_LaserSensor_left = NaN;

if (simulationHandlers_t.clientID>-1)
       disp('Connection succesful...')
       % Setup Handlers
       initializeHandlers();
        % Get current robotposition
        [~, robotPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);

        start = [2, 2, 0];
        tag1 = [11, 11, 0];
        tag2 = [2, 13, 0];
        tag3 = [5, 5, 0];
        goal = [11, 2, 0];
        
        nodes = [robotPosition ;start ;tag1 ;tag2; tag3; goal];

        % Set map parameters -- this can be read from the
        % PrepareEnvironment.m and can be adaptable based on the values
        % of the occpancy grid map --
        mapTranslation = 2;
        mapScale = 10;

        % Plan path between nodes
        for i=1:1:size(nodes,1)-1
            figure(i)
            pathPoints = RRT(nodes(i,1:3),nodes(i+1,1:3),map,mapTranslation,mapScale);
            disp('Path planned...')
            followPath(pathPoints);
            % Placeholder -- Here you can scan the tag, if it's not there
            % it has been moved/stolen --
       end

end