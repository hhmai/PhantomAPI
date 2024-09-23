#ifdef _WIN64
#pragma warning (disable:4996)
#endif

#include "pch.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <windows.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#ifdef __cplusplus
extern "C" {
#endif

    static const hduVector3Dd maxGimbalTorque(188.0, 188.0, 48.0); // mNm
    static const hduVector3Dd nominalBaseTorque(3000, 3000, 3000); // mNm
    static hduVector3Dd customBaseTorque(0, 0, 0); // mNm
    static hduVector3Dd customGimbalTorque(0.0, 0.0, 0.0); // mNm
    static hduVector3Dd wellPos(0, 0, 0);
    static bool TorqueMode = true;

    HDCallbackCode HDCALLBACK jointTorqueCallback(void* data);
    HDSchedulerHandle hGravityWell = HD_INVALID_HANDLE;
    HHD hHD = HD_INVALID_HANDLE;
	HHD hHD2 = HD_INVALID_HANDLE;

    __declspec(dllexport) int InitializeDevice()
    {
        HDErrorInfo error;

        hHD = hdInitDevice("Default PHANToM");
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Failed to initialize haptic device");
            return -1;
        }

        printf("Device initialized. Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

        hGravityWell = hdScheduleAsynchronous(jointTorqueCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
        hdEnable(HD_FORCE_OUTPUT);
        hdStartScheduler();

        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Failed to start scheduler");
            hdDisableDevice(hHD);
            return -1;
        }

        return 0;
    }

	__declspec(dllexport) void InitializeSecondDevice()
	{
		HDErrorInfo error;

		hHD2 = hdInitDevice("Default Device");
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Failed to initialize second haptic device");
			return;
		}

		printf("Second device initialized. Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
		hdEnable(HD_FORCE_OUTPUT);
	}

    __declspec(dllexport) void ShutdownDevice()
    {
        hdStopScheduler();
        hdUnschedule(hGravityWell);
        hdDisableDevice(hHD);
    }

	__declspec(dllexport) void ShutdownSecondDevice()
	{
		hdDisableDevice(hHD2);
	}

	__declspec(dllexport) void SetCustomTorque(double baseX, double baseY, double baseZ, double force)
	{
		customBaseTorque.set(baseX * force * 500, baseY * force * 250, baseZ * force * 500);
	}

    __declspec(dllexport) void SetCustomTorque(double baseX, double baseY, double baseZ, double force)
    {
        customBaseTorque.set(baseX * force * 500, baseY * force * 250, baseZ * force * 500);
    }

	__declspec(dllexport) void SetCustomTorqueRampUp(double baseX, double baseY, double baseZ, double force)
	{
		for (int i = 0; i < force * 3; i++)
		{
			customBaseTorque.set(baseX * force * 500/3, baseY * force * 500/3, baseZ * force * 500/3)
			wait(0.1); // Wait for a short duration to create a ramp-up effect
		}
	}

	__declspec(dllexport) void SetCustomTorqueRampDown(double baseX, double baseY, double baseZ, double force)
	{
		for (int i = force * 3; i > 0; i--)
		{
			customBaseTorque.set(baseX * force * 500/3, baseY * force * 500/3, baseZ * force * 500/3)
			wait(0.1); // Wait for a short duration to create a ramp-down effect
		}
		customBaseTorque.set(0, 0, 0); // Reset torque after ramp down
	}

	__declspec(dllexport) void SetTorqueMode(bool mode)
	{
		TorqueMode = mode;
	}

	__declspec(dllexport) void SetWellPosition(double x, double y, double z)
	{
		wellPos.set(x, y, z);
	}

	__declspec(dllexport) void GetCustomTorque(double* baseX, double* baseY, double* baseZ)
	{
		*baseX = customBaseTorque[0];
		*baseY = customBaseTorque[1];
		*baseZ = customBaseTorque[2];
	}

	__declspec(dllexport) void GetWellPosition(double* x, double* y, double* z)
	{
		*x = wellPos[0];
		*y = wellPos[1];
		*z = wellPos[2];
	}

    HDCallbackCode HDCALLBACK jointTorqueCallback(void* data)
    {
        const HDdouble kStiffness = 0.075; // N/mm
        const HDdouble kStylusTorqueConstant = 500; // mN.m/radian
        const HDdouble kJointTorqueConstant = 3000; // mN.m/radian

        const HDdouble kForceInfluence = 50; // mm
        const HDdouble kTorqueInfluence = 3.14; // radians

        static const hduVector3Dd stylusVirtualFulcrum(0.0, 0.0, 0.0); // In radians
        static const hduVector3Dd jointVirtualFulcrum(0.0, 0.0, 0.0); // In radians

        HDErrorInfo error;
        hduVector3Dd position;
        hduVector3Dd force;
        hduVector3Dd positionTwell;
        hduVector3Dd gimbalAngles;
        hduVector3Dd gimbalTorque;
        hduVector3Dd gimbalAngleOfTwist;
        hduVector3Dd jointAngles;
        hduVector3Dd jointTorque;
        hduVector3Dd jointAngleOfTwist;

        hdBeginFrame(hHD);

        hdGetDoublev(HD_CURRENT_POSITION, position);
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbalAngles);
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);

        memset(force, 0, sizeof(hduVector3Dd));

        jointTorque = customBaseTorque;

        // Clamp the base torques to the nominal values
        for (int i = 0; i < 3; i++)
        {
            if (jointTorque[i] > nominalBaseTorque[i])
                jointTorque[i] = nominalBaseTorque[i];
            else if (jointTorque[i] < -nominalBaseTorque[i])
                jointTorque[i] = -nominalBaseTorque[i];
        }

        // Clamp the gimbal torques to the max continuous values
        for (int i = 0; i < 3; i++)
        {
            if (gimbalTorque[i] > maxGimbalTorque[i])
                gimbalTorque[i] = maxGimbalTorque[i];
            else if (gimbalTorque[i] < -maxGimbalTorque[i])
                gimbalTorque[i] = -maxGimbalTorque[i];
        }

        if (!TorqueMode)
            hdSetDoublev(HD_CURRENT_FORCE, force);
        else
            hdSetDoublev(HD_CURRENT_JOINT_TORQUE, jointTorque);

        hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, gimbalTorque);

        hdEndFrame(hHD);

        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Error detected while rendering gravity well\n");
            if (hduIsSchedulerError(&error))
            {
                return HD_CALLBACK_DONE;
            }
        }

        return HD_CALLBACK_CONTINUE;
    }

#ifdef __cplusplus
}
#endif
