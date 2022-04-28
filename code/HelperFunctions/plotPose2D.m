function plotPose2D(pose2D)
%PLOTPOSE2D Plots the 2D positions stored in a vector array
close all
    for i=1:numel(pose2D)
        figure(1)
        plot(pose2D{i}(1),pose2D{i}(2),'bo')
        hold on
    end
    axis equal
end

