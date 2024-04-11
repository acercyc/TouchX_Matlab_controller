clc
% 10 seconds recording samples
n = 10 * 1000;

%% Initialize the device
TouchXcontroller_mex('InitDevice');

positions = cell(2, 1);
for i = [1, 2]
    %% Record positions for 10000 samples
    % countdown 3 seconds
    fprintf('countdown 3 seconds\n')
    for j = 1:3
        fprintf('countdown: %d\n', 3-j)
        pause(1)
    end

    % start ith recording
    fprintf('Start recording %d\n', i);
    TouchXcontroller_mex('RecordPositionAsynchronous_nSample', n);
    while TouchXcontroller_mex('isRecording')

        % Get and display the current position
        pos = TouchXcontroller_mex('GetPosition');
        fprintf('pos: %f %f %f\n', pos(1), pos(2), pos(3))
        pause(0.1);
    end

    %% Get recorded positions
    positions{i} = TouchXcontroller_mex('GetRecordedPositions');
    % print the size of the data
    fprintf('size(data): %d %d\n', size(positions{i}, 1), size(positions{i}, 2));

end


%% calculate the force
forces = cell(2, 1);
stiffness = 0.8;
for i = 1:2
    forces{i} = TouchXcontroller_mex('Pos2Force', positions{i}, stiffness);
end

%% mix two forces with weight 0.5
force = (forces{1} + forces{2}) / 2;

%% Play the force
% preloading the force
TouchXcontroller_mex('ForceBufferPreload', force);
fprintf('Ready to play the force\n')
% countdown 3 seconds
for i = 1:3
    fprintf('countdown: %d\n', 3-i)
    pause(1)
end

% start playing the force
fprintf('Start playing\n')
TouchXcontroller_mex('PlayForceAsynchronous');
t = tic;
while TouchXcontroller_mex('isPlayingForce')
    % print the current time
    fprintf('time: %f\n', toc(t))
    pause(0.1);
end
fprintf('Stop playing\n')

%% Disable the device
TouchXcontroller_mex('DisableDevice');