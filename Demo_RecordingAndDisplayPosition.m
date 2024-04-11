clc

%% Initialize the device
TouchXcontroller_mex('InitDevice');

%% Start recording infinitely
TouchXcontroller_mex('RecordPositionAsynchronous');

t = tic();
while TouchXcontroller_mex('isRecording')

    % Get and display the current position
    pos = TouchXcontroller_mex('GetPosition');
    fprintf('pos: %f %f %f\n', pos(1), pos(2), pos(3))
    pause(0.1);
    if toc(t) > 5
        % stop recording using TouchXcontroller_mex('StopRecording')
        TouchXcontroller_mex('StopRecording');
    end
end

%% Get recorded positions
data = TouchXcontroller_mex('GetRecordedPositions');

% print the size of the data
fprintf('size(data): %d %d\n', size(data, 1), size(data, 2));

%% Disable the device
TouchXcontroller_mex('DisableDevice');

