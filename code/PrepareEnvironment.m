% % This script initializes the neccessary variables and loads the
% neccessary databases etc..
% - ApriTag parameters
% - Camera parameters
% - Creates datasets from bag
% - Creates and initializes poseGraph

% AprilTag settings
tagFamily = "tag36h11";
tagSize = 300; % mm

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

cameraIntrinsics = cameraIntrinsics(fxy_simulated,principalPoint,imageSize); % Camera intrinsics parameters

% Create datasets from rosbag
% [imageData, odomData,pose2D] = helperExtractDataFromRosbag("/home/ben/Documents/University/Diplomaterv/datasets/OwnDataset/StraightBag_202204151235.bag");
[imageData, odomData,pose2D] = helperExtractDataFromRosbag("/home/ben/Documents/University/Diplomaterv/datasets/OwnDataset/onlab_round_20220511202953.bag");

% Create a pose graph object
pg = poseGraph3D;

% Storing the previous poses and nodeIds
lastTform = [];
lastPoseNodeId = 1;
% Container.Map mapping between tag IDs and Node IDs
tagToNodeIDMap = containers.Map('KeyType','double','ValueType','double');

% Apply the fixed transformation between the robot frame and the vision
% sensor
R1 = [-0.0871562  0  0.9961947; 0.9961947  0  0.0871562;0 1 0];
Ta = blkdiag(R1,1); % The camera frame has z axis pointing forward and y axis pointing down
Tb = eye(4); Tb(3,4) = 0.2; % Fixed translation of camera frame origin to 'laser' frame