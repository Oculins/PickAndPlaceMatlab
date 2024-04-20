function exampleCommandDetectPartsROSGazebo(coordinator)

% Detect parts and identify their poses
% This function detects parts using a pre-trained deep learning model. Each
% part is a struct with 2 elements: centerPoint and type.

% Copyright 2020 The MathWorks, Inc.

     % Empty cell array of parts to detect new parts
     coordinator.Parts = {};

     % Camera properties
     hfov = 1.581269;
     imageWidth = 480;
     focalLength = (imageWidth/2)/tan(hfov/2);
     inputSize = [224 224 3];
     original_size = [270, 480];

     % % Read image from simulated Gazebo camera
     % for i=1:50
     %    rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
     %    imwrite(rgbImg, strcat('image_hidding/hide',int2str(i), '.png'));
     %    i
     %    pause;
     % end
     
     rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
     depthImg = readImage(coordinator.ROSinfo.depthImgSub.LatestMessage);
     centerPixel = [round(size(rgbImg,1)/2), round(size(rgbImg,2)/2)]; % 135, 240

     % Detect parts and show labels
     % figure;
     imshow(rgbImg);
     imshow(depthImg);

     resizedImg = imresize(rgbImg, inputSize(1:2));
     [bboxes,~,labels] = detect(coordinator.DetectorModel,resizedImg,'executionEnvironment','cpu');
     resized_bbox = bboxes;
     resized_bbox(:, 1) = bboxes(:, 1) * original_size(1, 2) / inputSize(1, 2);
     resized_bbox(:, 2) = bboxes(:, 2) * original_size(1, 1) / inputSize(1, 1);
     resized_bbox(:, 3) = bboxes(:, 3) * original_size(1, 2) / inputSize(1, 2);
     resized_bbox(:, 4) = bboxes(:, 4) * original_size(1, 1) / inputSize(1, 1);
     bboxes = resized_bbox;
     % bboxes = [[110, 90, 66, 100]; [250, 100, 66, 100]];
     % labels = [['can'];['aaa']];

     % if ~isempty(labels)
     %    labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
     %    imshow(labeledImg);
     % end
     
     numObjects = size(bboxes,1);
     distances = zeros(numObjects,1);
     for i=1:numObjects
        distances(i) = depthImg(round(bboxes(i,2) + bboxes(i,4)/2), round(bboxes(i,1)+bboxes(i,3)/2));
     end
     [~, sorted_indices] = sort(distances);
     
     if ~isempty(labels)
        labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
        imshow(labeledImg);                
        allLabels =table(labels);
        skip = 0;
        for j=1:numObjects 
            i = sorted_indices(j);
            if bboxes(i,3)/bboxes(i,4) > 0.45 && bboxes(i,3)/bboxes(i,4)<0.65
                if allLabels.labels(i)=='red'
                    % Height of objects is known according to type
                    %part.Z = 0.052;
                    %part.X = 0.47;
                    part.type = 2;
                else 
                    %part.Z = 0.17;
                    %part.X = 0.45;
                    part.type = 1;
                end
                cameraTransf = getTransform(coordinator.Robot, coordinator.CurrentRobotJConfig, 'EndEffector_Link');
                xDistance = distances(i);
                centerBox = [round(bboxes(i,2)+bboxes(i,4)/2), round(bboxes(i,1)+bboxes(i,3)/2)];
                centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
                worldCenterBoxwrtCenterPixel = (xDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
                actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
                part.centerPoint = [actualCameraTransf(1,4)+xDistance,actualCameraTransf(2,4)-worldCenterBoxwrtCenterPixel(2), actualCameraTransf(3,4)-worldCenterBoxwrtCenterPixel(1)];
                coordinator.Parts{j-skip} = part;
            else
                skip = skip+1;
            end
        end
     end
    coordinator.NextPart = 0;
    if ~isempty(coordinator.Parts) && coordinator.NextPart<=length(coordinator.Parts)
        coordinator.DetectedParts = coordinator.Parts;
        % Trigger event 'partsDetected' on Stateflow
        coordinator.FlowChart.partsDetected;
        return;
    end
    
    coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;
    
    % Trigger event 'noPartsDetected' on Stateflow
    coordinator.FlowChart.noPartsDetected; 
   
end