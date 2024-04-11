clc

%% Initialize the device
TouchXcontroller_mex('InitDevice');

% ---------------------------------------------------------------------------- %
%                                record position                               %
% ---------------------------------------------------------------------------- %
%% record position for 10 seconds
% countdown 3 seconds
for i = 1:3
    fprintf('countdown: %d\n', 3-i)
    pause(1)
end

% start recording
fprintf('Start recording\n')
TouchXcontroller_mex('RecordPositionAsynchronous', 10);
t = tic;
while TouchXcontroller_mex('isRecording')
    % print the current time and position
    pos = TouchXcontroller_mex('GetPosition');
    fprintf('time: %f, pos: %f %f %f\n', toc(t), pos(1), pos(2), pos(3))
    pause(0.1);
end

%% Get recorded positions
data = TouchXcontroller_mex('GetRecordedPositions');
% print the size of the data
fprintf('size(data): %d %d\n', size(data, 1), size(data, 2));

%% calculate the force
stiffness = 0.8;
force = TouchXcontroller_mex('Pos2Force', data, stiffness);


% ---------------------------------------------------------------------------- %
%                                play the force                                %
% ---------------------------------------------------------------------------- %
%% play the force
fprintf('Ready to play the force\n')
% countdown 3 seconds
for i = 1:3
    fprintf('countdown: %d\n', 3-i)
    pause(1)
end

fprintf('Start playing\n')
TouchXcontroller_mex('PlayForceAsynchronous', force);
t = tic;
while TouchXcontroller_mex('isPlayingForce')
    % print the current time
    fprintf('time: %f\n', toc(t))
    pause(0.1);
end
fprintf('Stop playing\n')

%% Disable the device
TouchXcontroller_mex('DisableDevice');



