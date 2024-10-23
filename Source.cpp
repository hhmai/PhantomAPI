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
#include <iostream>
#include <chrono>

#ifdef __cplusplus
extern "C" {
#endif

	static hduVector3Dd customBaseTorque(0, 0, 0); // mNm
	static bool TorqueMode = true;
	static int timer = 0;
	static int x = 0;
	static int y = 0;
	static int z = 0;
	static int nForce = 0;
	static std::chrono::time_point<std::chrono::high_resolution_clock> rampStartTime;
	static bool rampUp = false;

	HDCallbackCode HDCALLBACK jointTorqueCallback(void* data);
	HHD hHD = HD_INVALID_HANDLE;

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

	__declspec(dllexport) void ShutdownDevice()
	{
		hdStopScheduler();
		hdDisableDevice(hHD);
	}

	__declspec(dllexport) void SetCustomTorque(double baseX, double baseY, double baseZ, double force)
	{
		customBaseTorque.set(baseX * force * 500, baseY * force * 250, baseZ * force * 500);
	}

	__declspec(dllexport) void SetCustomTorqueRampUp(double baseX, double baseY, double baseZ, double force)
	{
		timer = 0;
		x = baseX;
		y = baseY;
		z = baseZ;
		nForce = force;
		rampStartTime = std::chrono::high_resolution_clock::now();
		rampUp = true;
	}

	__declspec(dllexport) void SetCustomTorqueRampDown(double baseX, double baseY, double baseZ, double force)
	{
		rampUp = false;
		auto rampStartTime = std::chrono::high_resolution_clock::now();
		int i = force * 5;
		while (i > 0)
		{
			auto currentTime = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double, std::milli> elapsedTime = currentTime - rampStartTime;
			if (elapsedTime.count() >= 10)
			{
				customBaseTorque.set(baseX * force * 100 * i, baseY * force * 100 * i, baseZ * force * 100 * i);
				rampStartTime = std::chrono::high_resolution_clock::now();
				i -= 1;
			}
		}
		customBaseTorque.set(0, 0, 0);
	}

	__declspec(dllexport) void SetTorqueMode(bool mode)
	{
		TorqueMode = mode;
	}


	__declspec(dllexport) void GetCustomTorque(double* baseX, double* baseY, double* baseZ)
	{
		*baseX = customBaseTorque[0];
		*baseY = customBaseTorque[1];
		*baseZ = customBaseTorque[2];
	}

	HDCallbackCode HDCALLBACK jointTorqueCallback(void* data)
	{

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

		auto currentTime = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> elapsedTime = currentTime - rampStartTime;
		if (elapsedTime.count() >= 15 && timer < nForce * 5 && rampUp)
		{
			customBaseTorque.set(x * nForce * 100 * timer, y * nForce * 100 * timer, z * nForce * 100 * timer);
			rampStartTime = std::chrono::high_resolution_clock::now();
			timer += 1;
		}
		jointTorque = customBaseTorque;

		if (!TorqueMode)
			hdSetDoublev(HD_CURRENT_FORCE, force);
		else
			hdSetDoublev(HD_CURRENT_JOINT_TORQUE, jointTorque);

		hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, gimbalTorque);

		hdEndFrame(hHD);

		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Error detected\n");
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
