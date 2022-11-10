% % This script performs Apriltag based SLAM and show the results

figure(2)
figure('Visible','on')
for i = 1:numel(imageData)
    
    % Add odometry data to pose graph
    T = odomData{i};
    if isempty(lastTform)
        nodePair = addRelativePose(pg,[0 0 0 1 0 0 0],[],lastPoseNodeId);
    else
        relPose = exampleHelperComputeRelativePose(lastTform,T);
        nodePair = addRelativePose(pg,relPose,[],lastPoseNodeId);
    end
    currPoseNodeId = nodePair(2);
    
    % Add landmark measurement based on AprilTag observation in the image.
    I = imageData{i};
    [id,loc,poseRigid3d,detectedFamily] = readAprilTag(I,tagFamily,cameraIntrinsics,tagSize);
    
    for j = 1:numel(id)
        % Insert observation markers to image.
        markerRadius = 6;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,j),repmat(markerRadius,numCorners,1)];
        I = insertShape(I, "FilledCircle", markerPosition, "Color", "red", "Opacity", 1);

        t = poseRigid3d(j).Translation/1000; % change from mm to meter
        po = [t(:);1];
        p = Tb*Ta*po;
                
        if tagToNodeIDMap.isKey(id(j))
            lmkNodeId = tagToNodeIDMap(id(j));
            addPointLandmark(pg, p(1:3)', [], currPoseNodeId, lmkNodeId);
        else
            nodePair = addPointLandmark(pg, p(1:3)', [], currPoseNodeId);
            tagToNodeIDMap(id(j)) = nodePair(2);
        end
    end
    
    % Show the image with AprilTag observation markers.
    imshow(I)
    drawnow
            
    lastTform = T;
    lastPoseNodeId = currPoseNodeId;
end

