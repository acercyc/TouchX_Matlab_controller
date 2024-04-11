
### Command: `InitDevice`

- **Usage**: `InitDevice`
- **Functionality**: Initializes the haptic device. No input parameters are required. It returns the device handle as output.

### Command: `StopScheduler`

- **Usage**: `StopScheduler`
- **Functionality**: Stops the haptic device's scheduler if it is currently running. No input or output parameters.

### Command: `DisableDevice`

- **Usage**: `DisableDevice`
- **Functionality**: Disables the haptic device and stops the scheduler if it is running. No input or output parameters.

### Command: `GetPosition`

- **Usage**: `GetPosition`
- **Functionality**: Retrieves the current position of the haptic device's stylus. Returns a 1x3 matrix of the stylus' position.

### Command: `GetTime`

- **Usage**: `GetTime`
- **Functionality**: Returns the elapsed time in seconds since the system started.

### Command: `GetUpdateRate`

- **Usage**: `GetUpdateRate`
- **Functionality**: Fetches the current update rate of the haptic device's scheduler.

### Command: `SetUpdateRate`

- **Usage**: `SetUpdateRate, rate`
- **Functionality**: Sets the scheduler's update rate for the haptic device. Requires the desired rate as an input.

### Command: `RecordPositionAsynchronous`

- **Usage**: `RecordPositionAsynchronous, [duration]`
- **Functionality**: Starts recording the position of the device asynchronously. Optionally, a duration can be specified for how long to record. If no duration is given, it records indefinitely.

### Command: `StopRecording`

- **Usage**: `StopRecording`
- **Functionality**: Stops the asynchronous recording of the device position.

### Command: `GetRecordedPositions`

- **Usage**: `GetRecordedPositions`
- **Functionality**: Returns all recorded positions as an Nx3 matrix, where N is the number of recorded positions.

### Command: `isRecording`

- **Usage**: `isRecording`
- **Functionality**: Returns a logical indicating if the device is currently recording positions.

### Command: `PlayForceAsynchronous`

- **Usage**: `PlayForceAsynchronous, [force_matrix]`
- **Functionality**: Plays back forces asynchronously from a buffer. If a force matrix is provided, it loads these forces into the buffer; otherwise, it plays forces already in the buffer.

### Command: `GetPlayedForcesBuffer`

- **Usage**: `GetPlayedForcesBuffer`
- **Functionality**: Retrieves the currently buffered forces as an Nx3 matrix.

### Command: `isPlayingForce`

- **Usage**: `isPlayingForce`
- **Functionality**: Checks if the device is currently playing back forces asynchronously.

### Command: `ForceBufferPreload`

- **Usage**: `ForceBufferPreload, forces`
- **Functionality**: Preloads a buffer with force vectors provided as an Nx3 matrix.

### Command: `Pos2Force`

- **Usage**: `Pos2Force, positions, stiffness`
- **Functionality**: Converts a series of positions into forces based on the specified stiffness. Returns these forces as an Nx3 matrix.

### Command: `RecordedPositionsToPlayedForcesBuffer`

- **Usage**: `RecordedPositionsToPlayedForcesBuffer`
- **Functionality**: Computes forces from recorded positions and loads them into the force playback buffer.

These commands provide a comprehensive control set for interacting with a TouchX haptic device via MATLAB, facilitating various operations from initialization and data capture to force feedback simulations.
