function relPose = exampleHelperComputeRelativePose(T1, T2)
%EXAMPLEHELPERCOMPUTERELATIVEPOSE Compute the relative pose between two
%   SE(3) poses (in the form of 4x4 homogeneous transformation matrices)

%   Copyright 2020 The MathWorks, Inc.

relPose = robotics.core.internal.SEHelpers.tformToPoseSE3(...
                       robotics.core.internal.SEHelpers.tforminvSE3(T1)*T2);
end

