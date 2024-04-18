%% Copy OpenHaptics library files to the lib folder
openHapticsPath = 'C:\OpenHaptics\Developer\3.5.0';
folders = {'lib\x64\Release', 'include', 'QuickHaptics\lib\x64\Release', 'QuickHaptics\header', 'utilities\lib\x64\Release', 'utilities\include'};

for i = 1:length(folders)
    folder = fullfile(openHapticsPath, folders{i});
    copyfile(folder, 'lib');
    % print the copied folder done
    fprintf('Copied %s to lib folder\n', folder);
end

%% compile the mex file
mex -DWIN32...
    -I"lib"...
    -L"lib" -lhd -lhl...
    "-Llib"...
    "-lhd" "-lhl" "-lhdu" "-lhlu"...   
    "TouchXcontroller_mex.cpp"
