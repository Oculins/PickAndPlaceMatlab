% Transform the dataset in order to match the table in YOLOV2 example.
clear;

load("PickPlaceYOLOChallengeHiddenIgnore\ChallengeTaskHiddenIgnore.mat");
trainingData = table();
trainingData.imageFilename = gTruth.DataSource.Source;

for row=1:size(trainingData, 1)
    full_path = trainingData.imageFilename(row, :);
    full_path = full_path{1, 1};
    relative_path = strsplit(full_path, 'Robotics&Control\');
    relative_path = relative_path(1, 2);

    trainingData.imageFilename(row, :) = relative_path;
end
trainingData = [trainingData, gTruth.LabelData];

data_struct = struct("canTrainingData", trainingData);
save("PickPlaceYOLOChallengeHiddenIgnore\ChallengeTaskHiddenIgnore_T.mat", "data_struct");
