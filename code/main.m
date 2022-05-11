%% This is the main scirpt of the Landmark SLAM using Apriltags
clear
close all
clc


disp("----------------------------------------")
fprintf('Script Started %s\n', datestr(now,'HH:MM:SS.FFF'))
disp("----------------------------------------")

% % First we have to prepare the environmant
% - Rosbag and the extracted data
% - Camera parameters
% - Posegraph initialization
% - Container.Map structure to map the IDs between TagID and nodeID
disp("Preparing the environment!")
run("PrepareEnvironment.m")
disp("Environment is ready to use!")

disp("----------------------------------------")
disp("----------------------------------------")

% % Performing Apriltag based SLAM in the prepared environment
disp("Running SLAM!")
run("ApriltagSlam.m")
disp("Excecution done!")

% % Visualize the result poseGraph
figure('Visible','on');
show(pg);
axis equal;
title("The result of the Landmarkslam's posegraph");
