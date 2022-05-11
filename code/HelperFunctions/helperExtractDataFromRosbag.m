function [imageData, odomData,pose2D] = helperExtractDataFromRosbag(bagName)
%exampleHelperExtractDataFromRosbag Extract synchronized image and odometry
%   data from a rosbag recorded on a Jackal UGV robot. ROS Toolbox license
%   is required to use this function.

%   Copyright 2020 The MathWorks, Inc.

bagSel = rosbag(bagName);
imgSel = bagSel.select('Topic','/image_raw/compressed'); % Image topic

% Needed for computing change between SE(2) poses
ss = stateSpaceSE2;
ss.WeightTheta = 1;

lastPoseSE2 = [];
lastT = [];

imageData = {};
odomData = {};
pose2D = {};

% Grab all the image messages
imgObjs = imgSel.readMessages;
% Create waitbar
f = waitbar(0,'1','Name','Preparing environment...',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for i = 1:numel(imgObjs)
    % Check for clicked Cancel button
    if getappdata(f,'canceling')
        break
    end
    % Odom data is extracted from tf 
    if bagSel.canTransform('reference', 'Pioneer_p3dx', imgObjs{i}.Header.Stamp) % From odom frame to laser frame
        % ith odom reading is extracted at the time of the ith image
        Tstamped = bagSel.getTransform('reference', 'Pioneer_p3dx', imgObjs{i}.Header.Stamp);
        [T, poseSE2] = translateTransformStampedMsg(Tstamped);
    
        if isempty(lastT)
            takeThisImage = true;
        else
            takeThisImage = true;
            % Only accept a new pair of sensor measurements if the robot odom pose
            % has changed more than this threshold since last accepted one
            % Here we use the stateSpaceSE2's distance function to calculate the 2D pose difference
            if ss.distance(poseSE2, lastPoseSE2) < 0.06
                takeThisImage = false;
            end
        end

        if takeThisImage
            I = readImage(imgObjs{i}); % An alternative is to use the newer convenience function "rosReadImage"
            imageData{end+1} = I;
            odomData{end+1} = T;
            pose2D{end+1} = poseSE2;
            lastPoseSE2 = poseSE2;
            lastT = T;
        end
    end
    % Update waitbar and message
    waitbar(i/numel(imgObjs),f,sprintf('%12.1f',(i/numel(imgObjs))*100))
end
delete(f)

end

function [ToutSE3, poseSE2] = translateTransformStampedMsg(Tin)
%translateTransformMsg Extract the 4x4 homogeneous transformation matrix,
%   ToutSE3, from a TransformStamped message object. This function also 
%   returns a second output, poseSE2, which is an SE(2) pose vector 
%   computed by projecting ToutSE2 to the XY plane. Note the formulation
%   used is approximate and relies on the assumption that the robot mostly
%   moves on the flat ground. 

%   Copyright 2020 The MathWorks, Inc.

    %Tin - TransformStamped message object
    x = Tin.Transform.Translation.X;
    y = Tin.Transform.Translation.Y;
    z = Tin.Transform.Translation.Z;
    
    qx = Tin.Transform.Rotation.X;
    qy = Tin.Transform.Rotation.Y;
    qz = Tin.Transform.Rotation.Z;
    qw = Tin.Transform.Rotation.W;
    q = [ qw, qx, qy, qz]; % Note the sequence for quaternion in MATLAB

    ToutSE3 = robotics.core.internal.SEHelpers.poseToTformSE3([x,y,z,q]);
    YPR = quat2eul(q);
    poseSE2 = [x,y,YPR(1)];
end