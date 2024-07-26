# Phantom Full Force 
A Unity3D Plugin for Phantom Haptic Devices, written in C++

This API was developed to utilize any Phantom Haptic Device (3DOF, 1.5 HF, etc) in order to call perturbation effects and send force signals. The main reason why I chose to develop this Dynamically Linked Library over the provided Dlls from OpenHaptics is because I felt that the perturbation and well features were poorly built, considering I was working with a HighForce device and not receiving force worthy enough to be called HighForce.

## To use in Unity3D
* Copy and import the `PhantomAPI.dll` file into your Assets/Plugins or Assets/Dlls folder.
* In Unity, select the Dll and ensure that 'Load on Startup' is checked off. Press apply.
* Create a C# script that imports the following methods from "PhantomAPI": 
