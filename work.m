
rosIP = '192.168.174.131';
rosinit(rosIP,11311);

%edit exampleHelperFlowChartPickPlaceROSGazebo.sfx

%temp = load('exampleHelperYolo2DetectorROSGazebo.mat');

load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
initialRobotJConfig =  [0   0   0   2.613    0   -1.0024   -1.5708];
endEffectorFrame = "gripper";
coordinator = exampleHelperCoordinatorPickPlaceROSGazebo(robot,initialRobotJConfig, endEffectorFrame);
coordinator.HomeRobotTaskConfig = getTransform(robot, initialRobotJConfig, endEffectorFrame); 
% coordinator.PlacingPose{1} = trvec2tform([[0.2 0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);
% coordinator.PlacingPose{2} = trvec2tform([[0.2 -0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);
coordinator.PlacingPose{1} = trvec2tform([[0.2 0.55 0.26]])*axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]);
coordinator.PlacingPose{2} = trvec2tform([[0.2 -0.55 0.26]])*axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]);


coordinator.FlowChart = exampleHelperFlowChartPickPlaceROSGazebo('coordinator', coordinator);

answer = questdlg('Do you want to start the pick-and-place job now?', ...
         'Start job','Yes','No', 'No');

switch answer
    case 'Yes'
        % Trigger event to start Pick and Place in the Stateflow Chart
        coordinator.FlowChart.startPickPlace;       
    case 'No'
        coordinator.FlowChart.endPickPlace;
        delete(coordinator.FlowChart)
        delete(coordinator);
end

if strcmp(answer,'Yes')
    while  coordinator.NumDetectionRuns <  4
        % Wait for no parts to be detected.
    end
end

rosshutdown;
