% % This script initializes the neccessary variables and loads the
% neccessary databases etc..
% - ApriTag parameters
% - Camera parameters
% - Creates datasets from bag
% - Creates and initializes poseGraph
% - Creates occupancy grid map based on the scans from the robot's lasers

% Add script path to matlab path

% AprilTag settings
tagFamily = "tag36h11";
tagSize = 240; % mm

% Camera parameters
imageSize = [480,640]; % [pixels] H, W
sensorSize = [3.2,2.4]; % [mm] W, H
principalPoint = [imageSize(2)/2,imageSize(1)/2];
simulatedFocalLength = 2.8; % [mm]
measuredFocalLengthFromBag = [2.8226,2.8322]; % [mm]
horizontalFieldOfView = 67; % [deg]
verticalFieldOfView = 51; % [deg]
fxy_simulated = [simulatedFocalLength*(imageSize(2)/sensorSize(1)),...
                 simulatedFocalLength*(imageSize(1)/sensorSize(2))]; % [pixels]
fxy_measured = [measuredFocalLengthFromBag(1)*(imageSize(2)/sensorSize(1)),...
                measuredFocalLengthFromBag(2)*(imageSize(1)/sensorSize(2))]; % [pixels]

cameraIntrinsics = cameraIntrinsics(fxy_measured,principalPoint,imageSize); % Camera intrinsics parameters

% Set map correction parameters
mapTranslation = 2;
mapScale = 10;

% Create datasets from rosbag
% [imageData, odomData,pose2D, detectionObjs] = helperExtractDataFromRosbag("/home/ben/Documents/University/Diplomaterv/datasets/OwnDataset/2022-11-01-17-31-39.bag");
[imageData, odomData,pose2D, detectionObjs] = helperExtractDataFromRosbag("/home/ben/Documents/University/Diplomaterv/datasets/OwnDataset/2022-11-03-22-20-12.bag");

% Create a pose graph object
pg = poseGraph3D;

% Storing the previous poses and nodeIds
lastTform = [];
lastPoseNodeId = 1;
% Container.Map mapping between tag IDs and Node IDs
tagToNodeIDMap = containers.Map('KeyType','double','ValueType','double');

% Create occupancy grid map from laser measure
map = binaryOccupancyMap(200,200,1,"grid");
for i = 1:numel(detectionObjs)
    setOccupancy(map,[(detectionObjs{i}.X+mapTranslation)*mapScale (detectionObjs{i}.Y+mapTranslation)*mapScale],1);
end
inflate(map, 2)
figure(1)
show(map)



% Apply the fixed transformation between the robot frame and the vision
% sensor
R1 = [-0.0871562  0  0.9961947; 0.9961947  0  0.0871562;0 1 0];
Ta = blkdiag(R1,1); % The camera frame has z axis pointing forward and y axis pointing down
Tb = eye(4); Tb(3,4) = 0.2; % Fixed translation of camera frame origin to 'laser' frame