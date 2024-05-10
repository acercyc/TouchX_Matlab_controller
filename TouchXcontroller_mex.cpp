#include "mex.h"
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>

// Initialisation
HHD hHD = -1;
using Clock = std::chrono::steady_clock;
std::chrono::steady_clock::time_point system_start_time = Clock::now();
std::vector<hduVector3Dd> recordedPositions;
std::vector<hduVector3Dd> playedForcesBuffer;
std::chrono::steady_clock::time_point startTime;
HDSchedulerHandle gRecordingCallbackHandle = 0;
HDSchedulerHandle gPlayForcesCallbackHandle = 0;
hduVector3Dd currentPosition;
double duration = 0;
int nSamples = 0;
bool isScheduled = false;
bool isRecording = false;
bool isPlayingForce = false;

// Function to convert a MATLAB matrix to an array of hduVector3Dd
mxArray *vec2mat(const hduVector3Dd &vector)
{
    // Create a new mxArray
    mxArray *mat = mxCreateDoubleMatrix(1, 3, mxREAL);
    double *outputMatrix = mxGetPr(mat);

    // Copy the hduVector3Dd to the output matrix
    outputMatrix[0] = vector[0]; // X component
    outputMatrix[1] = vector[1]; // Y component
    outputMatrix[2] = vector[2]; // Z component

    return mat;
}

mxArray *vecs2mat(const std::vector<hduVector3Dd> &vectors)
{
    // Create a new mxArray
    mxArray *mat = mxCreateDoubleMatrix(vectors.size(), 3, mxREAL);
    double *outputMatrix = mxGetPr(mat);

    // Copy each hduVector3Dd to the output matrix
    for (size_t i = 0; i < vectors.size(); ++i)
    {
        outputMatrix[i] = vectors[i][0];                      // X component
        outputMatrix[i + vectors.size()] = vectors[i][1];     // Y component
        outputMatrix[i + 2 * vectors.size()] = vectors[i][2]; // Z component
    }

    return mat;
}

hduVector3Dd mat2vec(const mxArray *mat)
{
    // Ensure the input is of correct type and dimensions
    if (!mxIsDouble(mat) || mxIsComplex(mat) || mxGetM(mat) != 1 ||
        mxGetN(mat) != 3)
    {
        mexErrMsgIdAndTxt("MATLAB:forceVector:inputNotRealMatrix3D",
                          "Input must be an 1x3 matrix of real doubles.");
    }

    double *inputMatrix = mxGetPr(mat);
    size_t numRows = mxGetM(mat);

    // Vector to hold converted vectors
    hduVector3Dd vector;

    // import to hduVector3Dd
    vector[0] = inputMatrix[0]; // X component
    vector[1] = inputMatrix[1]; // Y component
    vector[2] = inputMatrix[2]; // Z component

    return vector;
}

std::vector<hduVector3Dd> mat2vecs(const mxArray *mat)
{
    // Ensure the input is of correct type and dimensions
    if (!mxIsDouble(mat) || mxIsComplex(mat) ||
        mxGetN(mat) != 3)
    {
        mexErrMsgIdAndTxt("MATLAB:forceVector:inputNotRealMatrix3D",
                          "Input must be an Nx3 matrix of real doubles.");
    }

    double *inputMatrix = mxGetPr(mat);
    size_t numRows = mxGetM(mat);

    // Vector to hold converted vectors
    std::vector<hduVector3Dd> vectors(numRows);

    // Convert each row of the input matrix to an hduVector3Dd
    for (size_t i = 0; i < numRows; ++i)
    {
        vectors[i][0] = inputMatrix[i];               // X component
        vectors[i][1] = inputMatrix[i + numRows];     // Y component
        vectors[i][2] = inputMatrix[i + 2 * numRows]; // Z component
    }

    return vectors;
}

// calculate Spring force from position temporal difference
std::vector<hduVector3Dd> pos2force(std::vector<hduVector3Dd> positions, double stiffness)
{
    std::vector<hduVector3Dd> forces;
    for (int i = 0; i < positions.size() - 1; i++)
    {
        hduVector3Dd force = (positions[i + 1] - positions[i]) * stiffness;
        forces.push_back(force);
    }
    return forces;
}

// Initialize the haptic device
void InitDevice()
{
    if (hHD != -1)
    {
        mexPrintf("**Haptic device has already initialized.**\n");
        return;
    }

    HDErrorInfo error;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        mexErrMsgIdAndTxt("HapticDevice:InitializationFailed",
                          "Failed to initialize haptic device: %s", hdGetErrorString(error.errorCode));
    }
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);
    hdStartScheduler();
    isScheduled = true;
    mexPrintf("***Haptic device initialized***\n");
}

// Disable the haptic device
void DisableDevice()
{

    if (hHD == -1)
    {
        mexPrintf("**Haptic device has already disabled.**\n");
        return;
    }

    HDErrorInfo error;
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        mexErrMsgIdAndTxt("HapticDevice:ShutdownFailed",
                          "Failed to shutdown the haptic device: %s", hdGetErrorString(error.errorCode));
    }

    mexPrintf("***Haptic device disabled***\n");
    hHD = -1;
}

hduVector3Dd GetPosition()
{
    hduVector3Dd position;
    // check scheduled
    if (isScheduled)
    {
        hdGetDoublev(HD_CURRENT_POSITION, position);
    }
    else
    {
        hdBeginFrame(hHD);
        hdGetDoublev(HD_CURRENT_POSITION, position);
        hdEndFrame(hHD);
    }
    return position;
}

double GetTime()
{
    auto elapsed_time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - system_start_time).count();
    double elapsed_time_seconds = elapsed_time_microseconds / 1e6;
    return elapsed_time_seconds;
}

double GetUpdateRate()
{
    HDdouble updateRate;
    hdGetDoublev(HD_UPDATE_RATE, &updateRate);
    mexPrintf("Update rate: %f\n", updateRate);
    return updateRate;
}

void SetUpdateRate(double updateRate)
{
    hdSetSchedulerRate(updateRate);
    mexPrintf("Update rate set to: %f\n", updateRate);
}

HDCallbackCode HDCALLBACK RecordPositionAsynchronous(void *data)
{
    // if duration is zero, record indefinitely
    if (duration == 0)
    {
        // record position
        hduVector3Dd position;
        hdBeginFrame(hHD);
        hdGetDoublev(HD_CURRENT_POSITION, position);
        recordedPositions.push_back(position);
        hdEndFrame(hHD);
        return HD_CALLBACK_CONTINUE;
    }
    else
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - startTime).count();
        if (elapsed >= duration)
        {
            isRecording = false;
            duration = 0;
            return HD_CALLBACK_DONE;
        }

        // record position
        hduVector3Dd position;
        hdBeginFrame(hHD);
        hdGetDoublev(HD_CURRENT_POSITION, position);
        recordedPositions.push_back(position);
        hdEndFrame(hHD);

        return HD_CALLBACK_CONTINUE;
    }
}

HDCallbackCode HDCALLBACK RecordPositionAsynchronous_nSample(void *data)
{
    // check if the number of samples is reached
    if (recordedPositions.size() >= nSamples)
    {
        isRecording = false;
        nSamples = 0;
        return HD_CALLBACK_DONE;
    }

    // record position
    hduVector3Dd position;
    hdBeginFrame(hHD);
    hdGetDoublev(HD_CURRENT_POSITION, position);
    recordedPositions.push_back(position);
    hdEndFrame(hHD);

    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK PlayForceAsynchronous(void *data)
{
    if (playedForcesBuffer.empty())
    {
        // set force to zero
        hduVector3Dd force(0, 0, 0);
        hdBeginFrame(hHD);
        hdSetDoublev(HD_CURRENT_FORCE, force);
        hdEndFrame(hHD);
        isPlayingForce = false;
        return HD_CALLBACK_DONE;
    }
    hduVector3Dd force = playedForcesBuffer[0];
    hdBeginFrame(hHD);
    hdSetDoublev(HD_CURRENT_FORCE, force);
    playedForcesBuffer.erase(playedForcesBuffer.begin());
    hdEndFrame(hHD);

    return HD_CALLBACK_CONTINUE;
}

// The gateway function for MATLAB
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check for proper number of arguments (at least one input required)
    if (nrhs < 1)
    {
        mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "At least one input required.");
        return;
    }

    // Extract the command string
    char *cmd = mxArrayToString(prhs[0]);

    /* -------------------------------------------------------------------------- */
    /*                              Device operation                              */
    /* -------------------------------------------------------------------------- */

    // InitDevice
    if (strcmp(cmd, "InitDevice") == 0)
    {
        InitDevice();
        plhs[0] = mxCreateDoubleScalar((double)hHD);
        return;
    }

    // StopScheduler
    else if (strcmp(cmd, "StopScheduler") == 0)
    {
        if (isScheduled)
        {
            hdStopScheduler();
            isScheduled = false;
        }
    }

    // DisableDevice
    else if (strcmp(cmd, "DisableDevice") == 0)
    {
        if (isScheduled)
        {
            hdStopScheduler();
            isScheduled = false;
        }
        DisableDevice();
    }

    /* -------------------------------------------------------------------------- */
    /*                               Get information                              */
    /* -------------------------------------------------------------------------- */
    // GetPosition
    else if (strcmp(cmd, "GetPosition") == 0)
    {
        hduVector3Dd position = GetPosition();
        plhs[0] = vec2mat(position);
    }

    // GetTime
    else if (strcmp(cmd, "GetTime") == 0)
    {
        double time = GetTime();
        plhs[0] = mxCreateDoubleScalar(time);
    }

    // GetUpdateRate
    else if (strcmp(cmd, "GetUpdateRate") == 0)
    {
        double updateRate = GetUpdateRate();
        plhs[0] = mxCreateDoubleScalar(updateRate);
    }

    /* -------------------------------------------------------------------------- */
    /*                               Set parameters                               */
    /* -------------------------------------------------------------------------- */
    else if (strcmp(cmd, "SetUpdateRate") == 0)
    {
        if (nrhs < 2)
        {
            mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "Update rate required.");
            return;
        }
        double updateRate = mxGetScalar(prhs[1]);
        SetUpdateRate(updateRate);
    }

    /* -------------------------------------------------------------------------- */
    /*                                  Recording                                 */
    /* -------------------------------------------------------------------------- */
    // record position asynchronously with input recording duration
    else if (strcmp(cmd, "RecordPositionAsynchronous") == 0)
    {
        // if only one input is provided, set duration to 0 (record indefinitely)
        if (nrhs == 1)
        {
            duration = 0;
        }

        // if the second input is provided, record for the specified duration
        else if (nrhs == 2)
        {
            // get recording duration
            duration = mxGetScalar(prhs[1]);
        }
        else
        {
            mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "Invalid input.");
            return;
        }

        // clear recorded positions
        recordedPositions.clear();

        // schedule asynchronous callback
        gRecordingCallbackHandle = hdScheduleAsynchronous(RecordPositionAsynchronous, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

        // start time
        startTime = Clock::now();

        isScheduled = true;
        isRecording = true;
    }

    // record position for n samples
    else if (strcmp(cmd, "RecordPositionAsynchronous_nSample") == 0)
    {
        if (nrhs < 2)
        {
            mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "Number of samples required.");
            return;
        }

        // get number of samples
        nSamples = mxGetScalar(prhs[1]);

        // clear recorded positions
        recordedPositions.clear();

        // record position
        gRecordingCallbackHandle = hdScheduleAsynchronous(RecordPositionAsynchronous_nSample, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

        isScheduled = true;
        isRecording = true;
    }

    // Stop recording
    else if (strcmp(cmd, "StopRecording") == 0)
    {
        if (isRecording)
        {
            hdUnschedule(gRecordingCallbackHandle);
            isRecording = false;
        }
    }

    // get recorded positions
    else if (strcmp(cmd, "GetRecordedPositions") == 0)
    {
        plhs[0] = vecs2mat(recordedPositions);
        return;
    }

    // isRecording
    else if (strcmp(cmd, "isRecording") == 0)
    {
        plhs[0] = mxCreateLogicalScalar(isRecording);
        return;
    }
    /* -------------------------------------------------------------------------- */
    /*                                 Play force                                 */
    /* -------------------------------------------------------------------------- */
    // PlayForceAsynchronous
    else if (strcmp(cmd, "PlayForceAsynchronous") == 0)
    {
        // if only one input is provided, play forces in the buffer, else play input forces
        if (nrhs == 2)
        // fill buffer with input forces
        {
            playedForcesBuffer = mat2vecs(prhs[1]);
        }
        else if (nrhs == 1)
        {
            // if buffer is empty show error
            if (playedForcesBuffer.empty())
            {
                mexErrMsgIdAndTxt("MATLAB:TouchXcontroller:PlayForceAsynchronous", "No forces in the buffer to play.");
                return;
            }
        }

        // schedule asynchronous callback
        gPlayForcesCallbackHandle = hdScheduleAsynchronous(PlayForceAsynchronous, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

        // start scheduler
        // hdStartScheduler();
        isScheduled = true;
        isPlayingForce = true;
    }

    // get played forces buffer
    else if (strcmp(cmd, "GetPlayedForcesBuffer") == 0)
    {
        plhs[0] = vecs2mat(playedForcesBuffer);
        return;
    }

    // isPlayingForce
    else if (strcmp(cmd, "isPlayingForce") == 0)
    {
        plhs[0] = mxCreateLogicalScalar(isPlayingForce);
        return;
    }

    // Force buffer preload
    else if (strcmp(cmd, "ForceBufferPreload") == 0)
    {
        if (nrhs < 2)
        {
            mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "Forces required.");
            return;
        }

        // get forces
        playedForcesBuffer = mat2vecs(prhs[1]);
    }

    /* -------------------------------------------------------------------------- */
    /*                              Utility functions                             */
    /* -------------------------------------------------------------------------- */
    // pos2force(positions, stiffness)
    else if (strcmp(cmd, "Pos2Force") == 0)
    {
        if (nrhs < 3)
        {
            mexErrMsgIdAndTxt("HapticDevice:InvalidInput", "Recorded positions and stiffness required.");
            return;
        }

        // get positions
        std::vector<hduVector3Dd> positions = mat2vecs(prhs[1]);

        // get stiffness
        double stiffness = mxGetScalar(prhs[2]);
        std::vector<hduVector3Dd> forces = pos2force(positions, stiffness);
        plhs[0] = vecs2mat(forces);
    }

    // recordedPositions to playedForcesBuffer
    else if (strcmp(cmd, "RecordedPositionsToPlayedForcesBuffer") == 0)
    {
        // compute force from position difference
        std::vector<hduVector3Dd> forces = pos2force(recordedPositions, 0.8);

        // clear played forces buffer
        playedForcesBuffer.clear();

        // fill played forces buffer with new forces
        playedForcesBuffer = forces;
    }
    else
    {
        mexErrMsgIdAndTxt("HapticDevice:InvalidCommand", "Invalid command.");
    }
}
